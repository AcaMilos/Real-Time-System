/**
 * @file    main.c
 * @author  Aleksandar Milosevic
 * @date    2021
 * @brief   SRV Project_28
 *          28 decimal =  011100 binary => A = 0, B = 1, C = 1, D = 1, E = 0, F = 0
 *
 *
 * Template to be used for lab work and projects
 */

/* Standard includes. */
#include <stdio.h>
#include <stdlib.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "event_groups.h"

/* Hardware includes. */
#include "msp430.h"

/* User's includes */
#include "ETF5529_HAL/hal_ETF_5529.h"

/* Event bit definitions*/
// Group events from xEventsGroupISR
#define  mainEVENT_BIT_ADC_ISR_READY     0x02    /* This is "ADC result is successfully written in queue during ISR" event bit mask */
// Group events from xEventsGroupTask2
#define  mainEVENT_BIT_BUTTON_S1         0x04    /* This is "Button S3 pressed" event bit mask */
#define  mainEVENT_BIT_BUTTON_S4         0x08    /* This is "Button S4 pressed" event bit mask */

/* Channels macros*/
#define ADC_CHANNEL_A0 0x00         // Channel A0 macro
#define ADC_CHANNEL_A1 0x01         // Channel A1 macro
#define ADC_CHANNEL_NONE 0x02

/* 7segm display signs*/
#define DISP_MINUS 16   // minus sign on display
#define DISP_OFF 17     // all segments are off

/* Time for multiplexing 7seg display in [ms]*/
#define MUX_7SEGM_PER_MS 5

/* 12-bit mask*/
#define MASK12_BITS 0x0FFF

/* ADC sample period*/
#define ADC_PERIOD_MS 1000                                      // ADC12 sampling period in [ms]

/** delay used for task synchronization */
#define mainTASK_SYNC_DELAY     ( pdMS_TO_TICKS( 100 ) )        // ( ( ( TickType_t ) 100 ) / portTICK_PERIOD_MS )

/* Number of elements in queues*/
#define mainADC_ISR_QUEUE_LENGTH    2
#define mainADC_QUEUE_LENGTH        1

/** define tasks priorities */
#define mainTASK1_PRIO        ( 4 )
#define mainTASK2_PRIO        ( 1 )
#define mainTASK3_PRIO        ( 2 )
#define mainTIMER_TASK_PRIO   ( 3 )

static void prvSetupHardware( void );
static void prvTask1( void *pvParameters );
static void prvTask2( void *pvParameters );
static void prvTask3( void *pvParameters );
static void prvTimerTask( void *pvParameters );
static void prv7segMUXTimerCallback(TimerHandle_t xTimer);

/* Event Group handlers */
EventGroupHandle_t  xEventsGroup;

/* Type of data that are sending through the queue from ISR to Task1*/
typedef struct QueueMessageType{
    uint8_t channel;
    uint16_t ADCresult;
} queue_message_t;

/* Semaphore handlers */
xSemaphoreHandle xSem7segMUX;

/* Queue handlers */
xQueueHandle xADCvalQueueISR;
xQueueHandle xADCvalQueue;

/* Software Timer handlers */
TimerHandle_t x7segMUXTimer;

/* Global variables*/
uint8_t ADC_A0_received = 0;    // Uses to store info in ADC12ISR: "Does the channel A0 result are stored in queue xADCvalQueueISR"
uint8_t ADC_A1_received = 0;    // Uses to store info in ADC12ISR: "Does the channel A1 result are stored in queue xADCvalQueueISR"

/**
 * @brief Task1
 *
 *
 */
static void prvTask1( void *pvParameters )
{
    /* Task1 initialisation on starting system*/
    EventBits_t eventValue;                     /* Value obtained from Event Group instance*/
    queue_message_t ADCval1FromISR;             // Uses for storing first value from queue in it
    queue_message_t ADCval2FromISR;             // Uses for storing second value from queue in it
    queue_message_t ADCvalToTask3;              // Stores the value that is needed to be transmitted to Task3
    uint16_t ADC_val_channel0 = 0;              // Uses for parsing values from queue xADCvalQueueISR -> which value comes from which channel
    uint16_t ADC_val_channel1 = 0;              // Uses for parsing values from queue xADCvalQueueISR -> which value comes from which channel
    uint8_t currentCHsending = ADC_CHANNEL_NONE;    // Channel which result are currently sending to Task3
    for ( ;; )
    {
        /* Task1 functionality*/
        /* Waiting for any of these three bits change */
        eventValue = xEventGroupWaitBits(xEventsGroup,
                                         mainEVENT_BIT_ADC_ISR_READY | mainEVENT_BIT_BUTTON_S1 | mainEVENT_BIT_BUTTON_S4,
                                         pdTRUE,
                                         pdFALSE,
                                         portMAX_DELAY);
        /* If ADC12 ISR has sent messages through the xADCvalQueueISR we need to read it and store it.
         * First Check this bit in EventsGroup. */
        if(eventValue & mainEVENT_BIT_ADC_ISR_READY){
            /*Then check does the booth values exist in queue xADCvalQueueISR. */
            if(xQueueReceive(xADCvalQueueISR, &ADCval1FromISR, 0) == pdTRUE && xQueueReceive(xADCvalQueueISR, &ADCval2FromISR, 0) == pdTRUE){
                /* Firstly disable interrupts so we can safely read values from queue (ADC12ISR wont write data in xADCvalQueueISR during we read it
                 * in this task)*/
                taskDISABLE_INTERRUPTS();
                /* Reading data from messages until we don't read both messages (if it's not known which one is first in queue).
                 * And parsing it to right variables*/
                switch(ADCval1FromISR.channel){
                case ADC_CHANNEL_A0:
                    ADC_val_channel0 = ADCval1FromISR.ADCresult;
                case ADC_CHANNEL_A1:
                    ADC_val_channel1 = ADCval2FromISR.ADCresult;
                default:
                    break;
                }
                switch(ADCval2FromISR.channel){
                case ADC_CHANNEL_A0:
                    ADC_val_channel0 = ADCval1FromISR.ADCresult;
                case ADC_CHANNEL_A1:
                    ADC_val_channel1 = ADCval2FromISR.ADCresult;
                default:
                    break;
                }
                /* After reading data from xADCvalQueueISR, we can enable ADC12ISR interrupt*/
                taskENABLE_INTERRUPTS();
            }
            /* If SW1/SW4 press was detected anytime before, specified channel result are sending to Task3*/
            if (currentCHsending != ADC_CHANNEL_NONE){
                /* Fill the queue xADCvalQueue (that sends ADC results to Task3) */
                switch(currentCHsending){
                case ADC_CHANNEL_A0:    //if last press was on SW4
                    ADCvalToTask3.channel = ADC_CHANNEL_A0;
                    ADCvalToTask3.ADCresult = ADC_val_channel0;
                    break;
                case ADC_CHANNEL_A1:    //if last press was on SW1
                    ADCvalToTask3.channel = ADC_CHANNEL_A1;
                    ADCvalToTask3.ADCresult = ADC_val_channel1;
                    break;
                default: break;
                }
                /* Sending specified channel ADC result to Task3 using queue xADCvalQueue*/
                xQueueSendToBack(xADCvalQueue, &ADCvalToTask3, portMAX_DELAY);
            }
        }
        /* Receiving info which button is pressed from Task2 using EventsGroup if the press has happened*/
        if((eventValue & mainEVENT_BIT_BUTTON_S1) || (eventValue & mainEVENT_BIT_BUTTON_S4)){
            switch(eventValue){
            case mainEVENT_BIT_BUTTON_S1:
                currentCHsending = ADC_CHANNEL_A1;
                break;
            case mainEVENT_BIT_BUTTON_S4:
                currentCHsending = ADC_CHANNEL_A0;
                break;
            default: break;
            }
        }
    }
}

/**
 * @brief Task2
 *
 *
 */
static void prvTask2( void *pvParameters )
{
    /* Task2 initialisation on starting system*/
    uint16_t i;
    /*Initial button states are 1 because of pull-up configuration*/
    uint8_t currentButtonState = 1;
    for ( ;; )
    {
        /* Task2 functionality*/
        /* Polling SW1 and SW4*/
        currentButtonState = ((P2IN & 0x02) >> 1) * ((P1IN & 0x20) >> 1);
        if(currentButtonState == 0){
            for(i = 0; i < 1000; i++);
            /*take button state*/
            /* check if button SW1 is pressed*/
            currentButtonState = ((P2IN & 0x02) >> 1);
            if(currentButtonState == 0){
                /* If SW1 is pressed set bit, defined with mainEVENT_BIT_BUTTON_S1 mask, in
                 * Event Group */
                xEventGroupSetBits(xEventsGroup, mainEVENT_BIT_BUTTON_S1);
                continue;
            }
            /* check if button SW4 is pressed*/
            currentButtonState = ((P1IN & 0x20) >> 5);
            if(currentButtonState == 0){
                /* If SW4 is pressed set bit, defined with mainEVENT_BIT_BUTTON_S4 mask, in
                 * Event Group */
                xEventGroupSetBits(xEventsGroup, mainEVENT_BIT_BUTTON_S4);
                continue;
            }
        }
    }
}

/**
 * @brief Task3
 *
 *
 */
static void prvTask3( void *pvParameters )
{
    /* Task3 initialisation on starting system*/
    uint16_t NewValueToShow = 0;
    uint16_t digitHigh = 0;
    uint16_t digitLow = 0;
    queue_message_t ADCvalue;
    uint16_t CH0val = 0;
    uint16_t CH1val = 0;
    uint8_t firstPackageReceived = 0;
    uint8_t newPackageReceived = 0;
    for ( ;; )
    {
        /* Task3 functionality*/
        /* Check if message is available, and if it is -> read it and store
         * result in right variable (depending on from which channel ADC value is)*/
        if(xQueueReceive(xADCvalQueue, &ADCvalue, 0) == pdTRUE){
            switch(ADCvalue.channel){
            case ADC_CHANNEL_A0:
                CH0val = ADCvalue.ADCresult;
                break;
            case ADC_CHANNEL_A1:
                CH1val = ADCvalue.ADCresult;
                break;
            default:
                break;
            }
            /* Set a flag that first package is received so the LED could be turned on*/
            firstPackageReceived = 1;
            /* Set a flag that we should calculate new value to be shown on display */
            newPackageReceived = 1;
        }

        /* If neither one package is received, display would be off*/
        if(firstPackageReceived == 0){
            digitHigh = DISP_OFF;
            digitLow = DISP_OFF;
        }
        /*If new results are received, calculate result and parse digits*/
        if(newPackageReceived == 1){
            if(CH0val >= CH1val){
                NewValueToShow = ((CH0val - CH1val) >> 1);  /* Sub channel values and form digits*/
                digitHigh = (NewValueToShow & 0xF0) >> 4;   /* Extract high digit*/
                digitLow = NewValueToShow & 0x0F;           /* Extract low digit*/
            }
            else{
                digitHigh = DISP_MINUS;     //show minus on disp1
                digitLow = (((CH1val - CH0val) >> 5) & 0x0F);  //show higher digit on disp2
            }
        }

        /* Reset a flag for new package so it can be useful next time*/
        newPackageReceived = 0;

        /* Showing number on display*/
        HAL_7SEG_DISPLAY_1_ON;
        HAL_7SEG_DISPLAY_2_OFF;
        vHAL7SEGWriteDigit(digitLow);
        xSemaphoreTake(xSem7segMUX, portMAX_DELAY);
        HAL_7SEG_DISPLAY_2_ON;
        HAL_7SEG_DISPLAY_1_OFF;
        vHAL7SEGWriteDigit(digitHigh);
        xSemaphoreTake(xSem7segMUX, portMAX_DELAY);
    }
}

/**
 * @brief Timer Task
 *
 *
 */
static void prvTimerTask( void *pvParameters )
{
    /* Timer Task initialisation on starting system*/
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for ( ;; )
    {
        /* Timer Task functionality*/
        xLastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(ADC_PERIOD_MS));
        ADC12CTL0 |= ADC12SC;
        /* Wait until receive all data from ADC12 to Task1 and than can continue*/
    }
}

/**
 * @brief 7SEG MUX timer callback function
 *
 *
 */
static void prv7segMUXTimerCallback(TimerHandle_t xTimer){
    xSemaphoreGive(xSem7segMUX);
}


/**
 * @brief main function
 */
void main( void )
{
    /* Configure peripherals */
    prvSetupHardware();

    /* Create tasks */
    xTaskCreate( prvTask1,                   // task function
                 "Task1",                    // task name
                 configMINIMAL_STACK_SIZE,   // stack size
                 NULL,                       // no parameter is passed
                 mainTASK1_PRIO,             // priority
                 NULL                        // we don't need handle
               );
    xTaskCreate( prvTask2,                   // task function
                 "Task2",                    // task name
                 configMINIMAL_STACK_SIZE,   // stack size
                 NULL,                       // no parameter is passed
                 mainTASK2_PRIO,             // priority
                 NULL                        // we don't need handle
               );
    xTaskCreate( prvTask3,                   // task function
                 "Task3",                    // task name
                 configMINIMAL_STACK_SIZE,   // stack size
                 NULL,                       // no parameter is passed
                 mainTASK3_PRIO,             // priority
                 NULL                        // we don't need handle
               );

    xTaskCreate( prvTimerTask,               // task function
                 "Timer Task",               // task name
                 configMINIMAL_STACK_SIZE,   // stack size
                 NULL,                       // no parameter is passed
                 mainTIMER_TASK_PRIO,        // priority
                 NULL                        // we don't need handle
               );


    /* Create timers */
    x7segMUXTimer = xTimerCreate("7SEG MUX timer",
                                 pdMS_TO_TICKS(pdMS_TO_TICKS(MUX_7SEGM_PER_MS)),
                                 pdTRUE,
                                 NULL,
                                 prv7segMUXTimerCallback);

    /* Create Event Groups*/
    xEventsGroup = xEventGroupCreate();

    /* Create Queues*/
    xADCvalQueueISR = xQueueCreate(mainADC_ISR_QUEUE_LENGTH, sizeof(queue_message_t));
    xADCvalQueue = xQueueCreate(mainADC_QUEUE_LENGTH, sizeof(queue_message_t));

    /* Create Semaphores*/
    xSem7segMUX   = xSemaphoreCreateBinary();

    /* Start timer with initial period */
    xTimerStart(x7segMUXTimer,portMAX_DELAY);

    /* Start the scheduler. */
    vTaskStartScheduler();


    /* If all is well then this line will never be reached.  If it is reached
    then it is likely that there was insufficient (FreeRTOS) heap memory space
    to create the idle task.  This may have been trapped by the malloc() failed
    hook function, if one is configured. */
    for( ;; );
}

/**
 * @brief Configure hardware upon boot
 */
static void prvSetupHardware( void )
{
    taskDISABLE_INTERRUPTS();

    /* Disable the watchdog. */
    WDTCTL = WDTPW + WDTHOLD;

    hal430SetSystemClock( configCPU_CLOCK_HZ, configLFXT_CLOCK_HZ );

    /* Init buttons SW1 (P2.1) and SW4(P1.5) */
    P1DIR &= ~0x20;
    P1REN |= 0x20;
    P1OUT |= 0x20;
    P2DIR &= ~0x02;
    P2REN |= 0x02;
    P2OUT |= 0x02;

    /*Initialize ADC */
    ADC12CTL0      &= ~ADC12ENC;                 // Disabling conversion before configuration ADC
    ADC12CTL0      = ADC12SHT02 | ADC12MSC | ADC12ON;       // Sampling time, ADC12 on
    ADC12CTL1      = ADC12CONSEQ_1 | ADC12SHP;   // ADC work in sequence-of-channels and use sampling timer
    ADC12IE        = 0x03;                       // Enable interrupt for MEM0 and MEM1 (memory buffers from ADC)
    ADC12MCTL0     |= ADC12INCH_0;               // Save results from channel A0 in MEM0 buffer
    ADC12MCTL1     |= ADC12EOS | ADC12INCH_1;    // Channel A1 is last of sequence channels that are converting and save results from channel A1 in MEM1 buffer
    ADC12CTL0      |= ADC12ENC;                  // Enabling conversion, and when SC signal appear conversion will start
    P6SEL          |= 0x03;                      // P6.0 and P6.1 ADC alternative options select

    /* initialize LEDs */
    vHALInitLED();                               // initializing pins for controlling LED to be output
    /* initialize display*/
    vHAL7SEGInit();                              // initializing pins for controlling 7seg display to be output
    /*enable global interrupts*/
    taskENABLE_INTERRUPTS();
}

void __attribute__ ( ( interrupt( ADC12_VECTOR  ) ) ) vADC12ISR( void )
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    BaseType_t xHigherPriorityTaskWoken_temp = pdFALSE;
    queue_message_t temp;
    switch(__even_in_range(ADC12IV, 34))
    {
        case  0: break;                           // Vector  0:  No interrupt
        case  2: break;                           // Vector  2:  ADC overflow
        case  4: break;                           // Vector  4:  ADC timing overflow
        case  6:                                  // Vector  6:  ADC12IFG0
            /* Scaling ADC value to fit on 9-bit representation*/
            temp.ADCresult = ((ADC12MEM0 & MASK12_BITS)>>3);  // saving result as 9-bits
            temp.channel = ADC_CHANNEL_A0;                    // channel number
            /* Packing result from channel A0 to queue*/
            xQueueSendToBackFromISR(xADCvalQueueISR, &temp, &xHigherPriorityTaskWoken);
            /* Setting flag that package from channel A0 was packed into the queue*/
            ADC_A0_received = 1;
            /* Checking if other channel result is packed already*/
            if(ADC_A1_received == 1){
                xHigherPriorityTaskWoken_temp = xHigherPriorityTaskWoken;
                /* If it's the true -> Setting bit mainEVENT_BIT_ADC_ISR_READY*/
                xEventGroupSetBitsFromISR(xEventsGroup, mainEVENT_BIT_ADC_ISR_READY, &xHigherPriorityTaskWoken);
            }
            break;
        case  8:                                  // Vector  8:  ADC12IFG1
            /* Scaling ADC value to fit on 9-bit representation*/
            temp.ADCresult = ((ADC12MEM1 & MASK12_BITS)>>3);  //saving result as 9-bits
            temp.channel = ADC_CHANNEL_A1;                    // channel number
            xQueueSendToBackFromISR(xADCvalQueueISR, &temp, &xHigherPriorityTaskWoken);
            /* Setting flag that package from channel A1 was packed into the queue*/
            ADC_A1_received = 1;
            /* Checking if other channel result is packed already*/
            if(ADC_A0_received == 1){
                xHigherPriorityTaskWoken_temp = xHigherPriorityTaskWoken;
                /* If it's the true -> Setting bit mainEVENT_BIT_ADC_ISR_READY*/
                xEventGroupSetBitsFromISR(xEventsGroup, mainEVENT_BIT_ADC_ISR_READY, &xHigherPriorityTaskWoken);
            }
            break;
        default: break;
    }
    if(xHigherPriorityTaskWoken_temp == pdFALSE){
        xHigherPriorityTaskWoken = pdFALSE;
    }
    /* Resetting flags*/
    if (ADC_A0_received == 1 && ADC_A1_received == 1){
        ADC_A0_received = 0;
        ADC_A1_received = 0;
    }
    /* trigger scheduler if higher priority task is woken */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
