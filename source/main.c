/*
 * FreeRTOS Kernel V10.4.3
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

#include "string.h"
/* TODO Add any manufacture supplied header files necessary for CMSIS functions
to be available here. */
/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "fsl_lpuart.h"
#include "fsl_lpadc.h"
#include "fsl_vref.h"
#include "fsl_spc.h"
#include "fsl_dac.h"
#include "fsl_flexcan.h"
#include "fsl_wwdt.h"
#include "fsl_flexio.h"
#include "fsl_common.h"
#include "fsl_ctimer.h"
#include "main.h"
#include "command.h"
#include "lcd.h"
#include "sensor.h"
#include "can.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Priorities at which the tasks are created.  The event semaphore task is
given the maximum priority of ( configMAX_PRIORITIES - 1 ) to ensure it runs as
soon as the semaphore is given. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY   (tskIDLE_PRIORITY + 2)
#define mainQUEUE_SEND_TASK_PRIORITY      (tskIDLE_PRIORITY + 1)
#define mainEVENT_SEMAPHORE_TASK_PRIORITY (configMAX_PRIORITIES - 1)

/* The rate at which data is sent to the queue, specified in milliseconds, and
converted to ticks using the portTICK_PERIOD_MS constant. */
#define mainQUEUE_SEND_PERIOD_MS (200 / portTICK_PERIOD_MS)

/* The period of the example software timer, specified in milliseconds, and
converted to ticks using the portTICK_PERIOD_MS constant. */
#define mainSOFTWARE_TIMER_PERIOD_MS (200 / portTICK_PERIOD_MS)

/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define mainQUEUE_LENGTH (1)

/* The rate at which data is sent to the queue, specified in milliseconds, and
converted to ticks using the portTICK_PERIOD_MS constant. */
#define LV_CAN_PERIOD 500



#define EXAMPLE_FLEXCAN_IRQn       CAN0_IRQn
#define EXAMPLE_FLEXCAN_IRQHandler CAN0_IRQHandler


#define LPUART_CLK_FREQ CLOCK_GetLPFlexCommClkFreq(4u)
#define BUFFER_SIZE          256
#define RX_BUFFER_SIZE 		256

#define BOARD_LED_GPIO     BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN BOARD_LED_RED_GPIO_PIN


#define LPADC_VREF_SOURCE           kLPADC_ReferenceVoltageAlt3

#define RX_MESSAGE_BUFFER_NUM (0)
#define TX_MESSAGE_BUFFER_NUM (1)
#define CAN_CLK_FREQ       CLOCK_GetFlexcanClkFreq(0U)
#define USE_IMPROVED_TIMING_CONFIG (1)

#define DEMO_LPUART_CLK_FREQ   CLOCK_GetLPFlexCommClkFreq(4u)
#define DEMO_LPUART_IRQn       LP_FLEXCOMM4_IRQn
#define LPUART_IRQHandler LP_FLEXCOMM4_IRQHandler

#define BOARD_SW3_NAME        "SW3"
#define BOARD_SW3_IRQ         GPIO00_IRQn
#define BOARD_SW3_IRQ_HANDLER GPIO00_IRQHandler


#define CTIMER          CTIMER0         /* Timer 0 */
#define CTIMER_MAT_OUT  kCTIMER_Match_0 /* Match output 0 */
#define CTIMER_CLK_FREQ CLOCK_GetCTimerClkFreq(0U)
#ifndef CTIMER_MAT_PWM_PERIOD_CHANNEL
#define CTIMER_MAT_PWM_PERIOD_CHANNEL kCTIMER_Match_3
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*
 * The queue send and receive tasks as described in the comments at the top of
 * this file.
 */
static void prvQueueReceiveTask(void *pvParameters);
static void prvQueueSendTask(void *pvParameters);
static void prvUartRxTask(void *pvParameters);
static void prvDisplayTask(void *pvParameters);
static void prvSensorTask(void *pvParameters);

/*
 * The callback function assigned to the example software timer as described at
 * the top of this file.
 */
static void vExampleTimerCallback(TimerHandle_t xTimer);

/*
 * The event semaphore task as described at the top of this file.
 */
static void prvEventSemaphoreTask(void *pvParameters);





/*
 * The event semaphore task as described at the top of this file.
 */
static void UART_Configure(void);
static void ADC_Configure(void);
static void DAC_Configure(void);
static void CAN_Configure(void);
static void WWDT_Configure(void);
static void PWM_Configure(void);
/*******************************************************************************
 * Globals
 ******************************************************************************/
/* The queue used by the queue send and queue receive tasks. */
static QueueHandle_t xQueue = NULL;

/* The semaphore (in this case binary) that is used by the FreeRTOS tick hook
 * function and the event semaphore task.
 */
static SemaphoreHandle_t xEventSemaphore = NULL;
/* The counters used by the various examples.  The usage is described in the
 * comments at the top of this file.
 */
static volatile uint32_t ulCountOfTimerCallbackExecutions = 0;
static volatile uint32_t ulCountOfItemsReceivedOnQueue    = 0;
static volatile uint32_t ulCountOfReceivedSemaphores      = 0;

volatile bool txComplete = false;
volatile bool rxComplete = false;
volatile bool wakenUp    = false;


lpuart_handle_t g_lpuartHandle;

uint8_t g_tipString[] =
    "Now please input command:\r\n";

// 环形缓冲区结构


volatile bool txOnGoing = false;
volatile bool rxOnGoing = false;

lpuart_config_t config;
lpuart_transfer_t xfer;
uint8_t ch;

/* Define the init structure for the output LED pin*/
gpio_pin_config_t gpio_config = {
    kGPIO_DigitalOutput,
    0,
};

circular_buffer_t g_rxBuffer = {0};
circular_buffer_t g_txBuffer = {0};





lpadc_config_t mLpadcConfigStruct;
lpadc_conv_trigger_config_t triggerConfigStruct[8];
lpadc_conv_command_config_t commandConfigStruct[8];
lpadc_conv_result_t resultStruct[8];
vref_config_t vrefConfig;
uint16_t adcValue[9];

dac_config_t dacConfigStruct;


const uint32_t g_LpadcFullRange   = 4096U;


/* Define the init structure for the can pin*/
flexcan_config_t flexcanConfig;
flexcan_rx_mb_config_t mbConfig;
flexcan_mb_transfer_t txXfer, rxXfer;
flexcan_handle_t flexcanHandle;
flexcan_frame_t frame,rxFrame;


/* 定义CAN消息队列结构 */
typedef struct {
    uint32_t id;
    uint8_t data[8];
    uint8_t length;
    uint32_t timestamp;
} can_message_t;

/* 全局变量 */
QueueHandle_t xCanRxQueue;
SemaphoreHandle_t xCanTxMutex;


/* 定义UART消息队列结构 */
typedef struct {
    uint8_t data[256];  // 接收到的数据
    uint16_t length;            // 数据长度
    uint32_t timestamp;         // 时间戳
} uart_message_t;

/* 全局变量 */
QueueHandle_t xUartRxQueue;     // UART接收队列
SemaphoreHandle_t xUartTxMutex; // UART发送互斥量

/* 任务函数原型 */
static void vTaskCANRx(void *pvParameters);
static void vTaskCANTx(void *pvParameters);

uint8_t demoRingBuffer[16];
volatile uint16_t txIndex; /* Index of the data to send out. */
volatile uint16_t rxIndex; /* Index of the memory to save new arrived data. */

volatile bool g_ButtonPress = false;

volatile uint32_t g_pwmPeriod   = 0U;
volatile uint32_t g_pulsePeriod = 0U;



/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Interrupt service fuction of switch.
 */
void BOARD_SW3_IRQ_HANDLER(void)
{
    /* Clear external interrupt flag. */
    GPIO_GpioClearInterruptFlags(BOARD_SW_GPIO, 1U << BOARD_SW_GPIO_PIN);
    /* Change state of button. */
    g_ButtonPress = true;
    SDK_ISR_EXIT_BARRIER;
}

// 环形缓冲区操作函数
static bool buffer_is_full(circular_buffer_t *buf) {
    return buf->count == BUFFER_SIZE;
}

static bool buffer_is_empty(circular_buffer_t *buf) {
    return buf->count == 0;
}

static bool buffer_push(circular_buffer_t *buf, uint8_t data) {
    if (buffer_is_full(buf)) {
        return false;
    }

    buf->buffer[buf->head] = data;
    buf->head = (buf->head + 1) % BUFFER_SIZE;
    __sync_fetch_and_add(&buf->count, 1);
    return true;
}

static bool buffer_pop(circular_buffer_t *buf, uint8_t *data) {
    if (buffer_is_empty(buf)) {
        return false;
    }

    *data = buf->buffer[buf->tail];
    buf->tail = (buf->tail + 1) % BUFFER_SIZE;
    __sync_fetch_and_sub(&buf->count, 1);
    return true;
}

void LPUART_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static uart_message_t msg = {0};
    static uint16_t bufIndex = 0;
    uint32_t statusFlags = LPUART_GetStatusFlags(LPUART4);

    /* 仅处理接收中断 */
    if (statusFlags & kLPUART_RxDataRegFullFlag)
    {
        uint8_t ch = LPUART_ReadByte(LPUART4);
        LPUART_ClearStatusFlags(LPUART4, kLPUART_RxDataRegFullFlag); // 清除接收标志
        
        /* 临界区保护（防止任务同时访问msg） */
         taskENTER_CRITICAL_FROM_ISR();
        
        // 缓冲区保护
        if (bufIndex < sizeof(msg.data) - 1) 
        {
            msg.data[bufIndex++] = ch;

            // 检测到行结束符
            if (ch == '\n' || ch == '\r') 
            {
                msg.length = bufIndex;
                msg.data[bufIndex] = '\0';
                
                if (xUartRxQueue != NULL) {
                    if (xQueueSendFromISR(xUartRxQueue, &msg, &xHigherPriorityTaskWoken) != pdPASS) 
                    {
                        // 队列满处理：记录错误或丢弃消息
                        PRINTF("UART Rx queue is full!\r\n");
                    }
                }
                bufIndex = 0;
                memset(msg.data,0,sizeof(msg.data));
            }
        } 
        else 
        {
            bufIndex = 0; // 缓冲区溢出保护
        }
        
        taskEXIT_CRITICAL_FROM_ISR(xHigherPriorityTaskWoken);
    }
}


/* LPUART user callback */
void LPUART_UserCallback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)
{
    PRINTF("UART received!\r\n");
    userData = userData;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static uart_message_t msg = {0};
    static uint16_t bufIndex = 0;

    if (kStatus_LPUART_TxIdle == status)
    {
        txOnGoing = false;
    }

    if (kStatus_LPUART_RxIdle == status)
    {
        rxOnGoing = false;
    }

    if (LPUART_GetStatusFlags(LPUART4) & kLPUART_RxDataRegFullFlag) {
	    uint8_t ch = LPUART_ReadByte(LPUART4);

        PRINTF("UART received! %d\r\n", ch);
        
        // 缓冲区保护
        if (bufIndex < sizeof(msg.data) - 1) {
            msg.data[bufIndex++] = ch;

            // 检测到行结束符
            if (ch == '\n' || ch == '\r') {
                msg.length = bufIndex;
                msg.data[bufIndex] = '\0';
                
                // 发送到队列
                if (xQueueSendFromISR(xUartRxQueue, &msg, &xHigherPriorityTaskWoken) != pdTRUE) {
                    PRINTF("UART queue full!\r\n");
                }
                
                bufIndex = 0; // 重置缓冲区
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        } else {
            bufIndex = 0; // 缓冲区溢出保护
        }
    }
}

/*!
 * @brief FlexCAN Call Back function
 */
static FLEXCAN_CALLBACK(flexcan_callback)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    can_message_t msg;
    switch (status)
    {
        case kStatus_FLEXCAN_RxIdle:
            if (RX_MESSAGE_BUFFER_NUM == result)
            {
                // 提取CAN帧数据
                msg.id = frame.id;
                msg.length = frame.length;
                uint8_t *dataPtr = msg.data;
                uint32_t word0 = frame.dataWord0;
                uint32_t word1 = frame.dataWord1;

                // 拆解32位字到字节数组
                for (uint8_t i = 0; i < 4; i++) {
                    if (msg.length > i) {
                        *dataPtr++ = (word0 >> (i * 8)) & 0xFF;
                    }
                }
                for (uint8_t i = 0; i < 4; i++) {
                    if (msg.length > (4 + i)) {
                        *dataPtr++ = (word1 >> (i * 8)) & 0xFF;
                    }
                }

                // msg.timestamp = frame.timestamp;

                // 发送消息到队列
                if (xQueueSendFromISR(xCanRxQueue, &msg, &xHigherPriorityTaskWoken) != pdTRUE)
                {
                    // 队列已满，可以添加错误处理
                }
                
                // 如果需要切换上下文
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                
                // 重新启动接收
                (void)FLEXCAN_TransferReceiveNonBlocking(CAN0, &flexcanHandle, &rxXfer);

                rxComplete = true;
                PRINTF("CAN receive successed!.\r\n");
            }
            break;

        case kStatus_FLEXCAN_TxIdle:
            // if (TX_MESSAGE_BUFFER_NUM == result)
            // {
            //     txComplete = true;
            // }
            break;

        case kStatus_FLEXCAN_WakeUp:
            wakenUp = true;
            break;

        default:
            break;
    }
}

void delayWwdtWindow(void)
{
    /* For the TV counter register value will decrease after feed watch dog,
     * we can use it to as delay. But in user scene, user need feed watch dog
     * in the time period after enter Window but before warning intterupt.
     */
    while (WWDT0->TV > WWDT0->WINDOW)
    {
        __NOP();
    }
}


/*!
 * @brief Main function
 */
int main(void)
{
    TimerHandle_t xExampleSoftwareTimer = NULL;

    /* Init board hardware. */
    /* attach FRO 12M to FLEXCOMM4 (debug console) */
    CLOCK_SetClkDiv(kCLOCK_DivFlexcom4Clk, 1u);
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
    /* enable clock for GPIO*/
    CLOCK_EnableClock(kCLOCK_Gpio0);
    /* attach FRO HF to ADC0 */
	CLOCK_SetClkDiv(kCLOCK_DivAdc0Clk, 1U);
	CLOCK_AttachClk(kFRO_HF_to_ADC0);
	/* attach FRO HF to ADC1 */
	CLOCK_SetClkDiv(kCLOCK_DivAdc1Clk, 1U);
	CLOCK_AttachClk(kFRO_HF_to_ADC1);
    /* attach FRO HF to DAC0 */
    CLOCK_SetClkDiv(kCLOCK_DivDac0Clk, 1u);
    CLOCK_AttachClk(kFRO_HF_to_DAC0);
    /* attach FRO HF to DAC1 */
    CLOCK_SetClkDiv(kCLOCK_DivDac1Clk, 1u);
    CLOCK_AttachClk(kFRO_HF_to_DAC1);
    /* attach PLLClk to FLEXCAN0 */
    CLOCK_SetClkDiv(kCLOCK_DivPllClk, 2U);
    CLOCK_SetClkDiv(kCLOCK_DivFlexcan0Clk, 1U);
    CLOCK_AttachClk(kPLL0_to_FLEXCAN0);
    /* Set clock divider for WWDT clock source. */
    CLOCK_SetClkDiv(kCLOCK_DivWdt0Clk, 1U);
    /* attach FRO HF to FLEXIO */
    CLOCK_SetClkDiv(kCLOCK_DivFlexioClk, 4u);
    CLOCK_AttachClk(kFRO_HF_to_FLEXIO);
    /* Use FRO HF clock for some of the Ctimers */
    CLOCK_SetClkDiv(kCLOCK_DivCtimer0Clk, 1u);
    CLOCK_AttachClk(kFRO_HF_to_CTIMER0);


    BOARD_InitPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();


    UART_Configure();
    ADC_Configure();
    DAC_Configure();
    CAN_Configure();
    WWDT_Configure();
    PWM_Configure();


    PRINTF("MCU init!\r\n");

    /* Create the queue used by the queue send and queue receive tasks. */
    xQueue = xQueueCreate(/* The number of items the queue can hold. */
                          mainQUEUE_LENGTH,
                          /* The size of each item the queue holds. */
                          sizeof(uint32_t));

    /* Enable queue view in MCUX IDE FreeRTOS TAD plugin. */
    if (xQueue != NULL)
    {
        vQueueAddToRegistry(xQueue, "xQueue");
    }

    /* Create the semaphore used by the FreeRTOS tick hook function and the
    event semaphore task. */
    vSemaphoreCreateBinary(xEventSemaphore);

    /* Create the queue receive task as described in the comments at the top
    of this    file. */
    if (xTaskCreate(/* The function that implements the task. */
                    prvQueueReceiveTask,
                    /* Text name for the task, just to help debugging. */
                    "Rx",
                    /* The size (in words) of the stack that should be created
                    for the task. */
                    configMINIMAL_STACK_SIZE + 166,
                    /* A parameter that can be passed into the task.  Not used
                    in this simple demo. */
                    NULL,
                    /* The priority to assign to the task.  tskIDLE_PRIORITY
                    (which is 0) is the lowest priority.  configMAX_PRIORITIES - 1
                    is the highest priority. */
                    mainQUEUE_RECEIVE_TASK_PRIORITY,
                    /* Used to obtain a handle to the created task.  Not used in
                    this simple demo, so set to NULL. */
                    NULL) != pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;
    }

    /* Create the queue send task in exactly the same way.  Again, this is
    described in the comments at the top of the file. */
    if (xTaskCreate(prvQueueSendTask, "TX", configMINIMAL_STACK_SIZE + 166, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL) !=
        pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;
    }

    /* Create the queue send task in exactly the same way.  Again, this is
        described in the comments at the top of the file. */
	if (xTaskCreate(prvUartRxTask, "command", 512, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL) !=
		pdPASS)
	{
		PRINTF("Task creation failed!.\r\n");
		while (1)
			;
	}

    if (xTaskCreate(prvDisplayTask, "display", configMINIMAL_STACK_SIZE + 512, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL) !=
		pdPASS)
	{
		PRINTF("Task creation failed!.\r\n");
		while (1)
			;
	}

    if (xTaskCreate(prvSensorTask, "sensor", configMINIMAL_STACK_SIZE + 166, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL) !=
		pdPASS)
	{
		PRINTF("Task creation failed!.\r\n");
		while (1)
			;
	}


    xCanRxQueue = xQueueCreate(5, sizeof(can_message_t));  // 接收队列
    if (xCanRxQueue == NULL) {
        PRINTF("Error: Failed to create CAN Rx queue!\r\n");
        while(1); // 停止执行
        }

    /* 初始化接收队列 */
    xUartRxQueue = xQueueCreate(5, sizeof(uart_message_t));
    if (xUartRxQueue == NULL) {
        PRINTF("Error: Failed to create UART Rx queue!\r\n");
        while(1); // 停止执行
        }

    if (xTaskCreate(vTaskCANRx, "CAN_RX", configMINIMAL_STACK_SIZE + 166, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL) !=
		pdPASS)
	{
		PRINTF("Task creation failed!.\r\n");
		while (1)
			;
	}

    if (xTaskCreate(vTaskCANTx, "CAN_TX", configMINIMAL_STACK_SIZE + 166, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL) !=
		pdPASS)
	{
		PRINTF("Task creation failed!.\r\n");
		while (1)
			;
	}

    /* Create the task that is synchronised with an interrupt using the
    xEventSemaphore semaphore. */
    if (xTaskCreate(prvEventSemaphoreTask, "Sem", configMINIMAL_STACK_SIZE + 166, NULL,
                    mainEVENT_SEMAPHORE_TASK_PRIORITY, NULL) != pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;
    }


    /* Create the software timer as described in the comments at the top of
    this file. */
    xExampleSoftwareTimer = xTimerCreate(/* A text name, purely to help
                                       debugging. */
                                         "LEDTimer",
                                         /* The timer period, in this case
                                         1000ms (1s). */
                                         mainSOFTWARE_TIMER_PERIOD_MS,
                                         /* This is a periodic timer, so
                                         xAutoReload is set to pdTRUE. */
                                         pdTRUE,
                                         /* The ID is not used, so can be set
                                         to anything. */
                                         (void *)0,
                                         /* The callback function that switches
                                         the LED off. */
                                         vExampleTimerCallback);

    /* Start the created timer.  A block time of zero is used as the timer
    command queue cannot possibly be full here (this is the first timer to
    be created, and it is not yet running). */
    xTimerStart(xExampleSoftwareTimer, 0);

    /* Start the tasks and timer running. */
    vTaskStartScheduler();

    for (;;)
        ;
}

/*!
 * @brief Timer callback.
 */
static void vExampleTimerCallback(TimerHandle_t xTimer)
{
    /* The timer has expired.  Count the number of times this happens.  The
    timer that calls this function is an auto re-load timer, so it will
    execute periodically. */
    ulCountOfTimerCallbackExecutions++;
    GPIO_PortToggle(DBG_LED_GPIO, 1u << DBG_LED0_PIN);
    GPIO_PortToggle(DBG_LED_GPIO, 1u << DBG_LED1_PIN);
    GPIO_PortToggle(DBG_LED_GPIO, 1u << DBG_LED2_PIN);
    GPIO_PortToggle(CAN_LED_GPIO, 1u << CAN_LED_RUN_PIN);
    GPIO_PortToggle(CAN_LED_GPIO, 1u << CAN_LED_ERROR_PIN);


    WWDT_Refresh(WWDT0);
    
}

/*!
 * @brief Task prvQueueSendTask periodically sending message.
 */
static void prvQueueSendTask(void *pvParameters)
{
    TickType_t xNextWakeTime;
    const uint32_t ulValueToSend = 100UL;

    /* Initialise xNextWakeTime - this only needs to be done once. */
    xNextWakeTime = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&xNextWakeTime, mainQUEUE_SEND_PERIOD_MS);

        xQueueSend(xQueue, &ulValueToSend, 0);
    }
}

/*!
 * @brief Task prvQueueReceiveTask waiting for message.
 */
static void prvQueueReceiveTask(void *pvParameters)
{
    uint32_t ulReceivedValue = 0L;

    for (;;)
    {
        /* Wait until something arrives in the queue - this task will block
        indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
        FreeRTOSConfig.h. */
        xQueueReceive(xQueue, &ulReceivedValue, portMAX_DELAY);

        /*  To get here something must have been received from the queue, but
        is it the expected value?  If it is, increment the counter. */
        if (ulReceivedValue == 100UL)
        {
            /* Count the number of items that have been received correctly. */
            ulCountOfItemsReceivedOnQueue++;
            // PRINTF("Receive message counter: %d.\r\n", ulCountOfItemsReceivedOnQueue);
        }
    }
}

/*!
 * @brief task prvEventSemaphoreTask is waiting for semaphore.
 */
static void prvEventSemaphoreTask(void *pvParameters)
{
    for (;;)
    {
        /* Block until the semaphore is 'given'. */
        if (xSemaphoreTake(xEventSemaphore, portMAX_DELAY) != pdTRUE)
        {
            PRINTF("Failed to take semaphore.\r\n");
        }

        /* Count the number of times the semaphore is received. */
        ulCountOfReceivedSemaphores++;

    }
}

/*!
 * @brief tick hook is executed every tick.
 */
void vApplicationTickHook(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static uint32_t ulCount             = 0;

    /* The RTOS tick hook function is enabled by setting configUSE_TICK_HOOK to
    1 in FreeRTOSConfig.h.

    "Give" the semaphore on every 500th tick interrupt. */
    ulCount++;
    if (ulCount >= 500UL)
    {
        /* This function is called from an interrupt context (the RTOS tick
        interrupt),    so only ISR safe API functions can be used (those that end
        in "FromISR()".

        xHigherPriorityTaskWoken was initialised to pdFALSE, and will be set to
        pdTRUE by xSemaphoreGiveFromISR() if giving the semaphore unblocked a
        task that has equal or higher priority than the interrupted task. */
        xSemaphoreGiveFromISR(xEventSemaphore, &xHigherPriorityTaskWoken);
        ulCount = 0UL;
    }

    /* If xHigherPriorityTaskWoken is pdTRUE then a context switch should
    normally be performed before leaving the interrupt (because during the
    execution of the interrupt a task of equal or higher priority than the
    running task was unblocked).  The syntax required to context switch from
    an interrupt is port dependent, so check the documentation of the port you
    are using.

    In this case, the function is running in the context of the tick interrupt,
    which will automatically check for the higher priority task to run anyway,
    so no further action is required. */
}

/*!
 * @brief Malloc failed hook.
 */
void vApplicationMallocFailedHook(void)
{
    /* The malloc failed hook is enabled by setting
    configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

    Called if a call to pvPortMalloc() fails because there is insufficient
    free memory available in the FreeRTOS heap.  pvPortMalloc() is called
    internally by FreeRTOS API functions that create tasks, queues, software
    timers, and semaphores.  The size of the FreeRTOS heap is set by the
    configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

    PRINTF("Memory allocation failed!\r\n");
    for (;;)
        ;
}

/*!
 * @brief Stack overflow hook.
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)pcTaskName;
    (void)xTask;

    /* Run time stack overflow checking is performed if
    configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected.  pxCurrentTCB can be
    inspected in the debugger if the task name passed into this function is
    corrupt. */
    for (;;)
        ;
}

/*!
 * @brief Idle hook.
 */
void vApplicationIdleHook(void)
{
    volatile size_t xFreeStackSpace;

    /* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
    FreeRTOSConfig.h.

    This function is called on each cycle of the idle task.  In this case it
    does nothing useful, other than report the amount of FreeRTOS heap that
    remains unallocated. */
    xFreeStackSpace = xPortGetFreeHeapSize();

    if (xFreeStackSpace > 100)
    {
        /* By now, the kernel has allocated everything it is going to, so
        if there is a lot of heap remaining unallocated then
        the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
        reduced accordingly. */
    }
}




/*!
 * @brief Task prvQueueSendTask periodically sending message.
 */
static void prvUartRxTask(void *pvParameters)
{
    uart_message_t rxMsg;
    PRINTF("UART receive task started\r\n"); // 调试输出，确认任务启动

    for (;;)
    {
        if (xQueueReceive(xUartRxQueue, &rxMsg, portMAX_DELAY) == pdTRUE) 
        {
            /* 验证数据有效性 */
            if (rxMsg.length > 0 && rxMsg.length <= sizeof(rxMsg.data)) 
            {
                processReceivedCommand((char*)rxMsg.data);
            }
        }
    }
}


/*!
 * @brief Task prvQueueSendTask periodically sending message.
 */
static void prvDisplayTask(void *pvParameters)
{
    TickType_t tick_start;
	TickType_t tick_end;
	TickType_t delay_target;

    lv_port_disp_init();


    for (;;)
    {
        tick_start = xTaskGetTickCount();

    	Display();

    	tick_end = xTaskGetTickCount();

		if (tick_end - tick_start >= pdMS_TO_TICKS(LV_DISP_DEF_REFR_PERIOD)) {
			/* The task takes too long to finish, use minimum delay target */
			delay_target = 1;
		} else {
			delay_target = pdMS_TO_TICKS(LV_DISP_DEF_REFR_PERIOD) - (tick_end - tick_start);
		}

		vTaskDelay(delay_target);
    }
}

/*!
 * @brief Task prvQueueSendTask periodically sending message.
 */
static void prvSensorTask(void *pvParameters)
{
    for (;;)
    {
    	readSensor();
    }
}


static void vTaskCANRx(void *pvParameters)
{
    can_message_t rxMsg;
    PRINTF("CAN_RX Task started\r\n");

    for (;;) {
        // 使用带超时的接收，防止永久阻塞
        if (xQueueReceive(xCanRxQueue, &rxMsg, pdMS_TO_TICKS(1000)))
        {
            // 打印接收到的完整数据
            PRINTF("RX ID: 0x%03X, Length: %d, Data: ", 
                  (rxMsg.id >> CAN_ID_STD_SHIFT), rxMsg.length);
            for (uint8_t i = 0; i < rxMsg.length; i++) 
            {
                PRINTF("0x%02X ", rxMsg.data[i]);
            }
            PRINTF("\r\n");
            
            // 处理完成后可以添加适当的延迟
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        else
        {
            // 超时处理，可以添加一些维护代码
        }
    }
}

static void vTaskCANTx(void *pvParameters)
{

    TickType_t tick_start;
	TickType_t tick_end;
	TickType_t delay_target;

    for (;;)
    {
        tick_start = xTaskGetTickCount();

    	sendCAN();

        tick_end = xTaskGetTickCount();

		if (tick_end - tick_start >= pdMS_TO_TICKS(LV_CAN_DEF_REFR_PERIOD)) {
			/* The task takes too long to finish, use minimum delay target */
			delay_target = 1;
		} else {
			delay_target = pdMS_TO_TICKS(LV_CAN_DEF_REFR_PERIOD) - (tick_end - tick_start);
		}

		vTaskDelay(delay_target);
    }
}





void ADC_Configure(void)
{
	/* enable VREF */
	SPC_EnableActiveModeAnalogModules(SPC0, kSPC_controlVref);


	VREF_GetDefaultConfig(&vrefConfig);
	vrefConfig.bufferMode = kVREF_ModeBandgapOnly;
	/* Initialize VREF module, the VREF module is only used to supply the bias current for LPADC. */
	VREF_Init(VREF0, &vrefConfig);
	LPADC_GetDefaultConfig(&mLpadcConfigStruct);
	mLpadcConfigStruct.enableAnalogPreliminary = true;
	mLpadcConfigStruct.referenceVoltageSource = LPADC_VREF_SOURCE;
	mLpadcConfigStruct.conversionAverageMode = kLPADC_ConversionAverage128;

	LPADC_Init(ADC0, &mLpadcConfigStruct);
	/* Request LPADC calibration. */
	LPADC_DoOffsetCalibration(ADC0);
	/* Request auto calibration (including gain error calibration and linearity error calibration). */
	LPADC_DoAutoCalibration(ADC0);

	LPADC_Init(ADC1, &mLpadcConfigStruct);
	/* Request LPADC calibration. */
	LPADC_DoOffsetCalibration(ADC1);
	/* Request auto calibration (including gain error calibration and linearity error calibration). */
	LPADC_DoAutoCalibration(ADC1);


    /* Set conversion CMD configuration. */
    LPADC_GetDefaultConvCommandConfig(&commandConfigStruct[ADC0_A0B0]);
    commandConfigStruct[ADC0_A0B0].channelNumber = 0U;
    commandConfigStruct[ADC0_A0B0].enableChannelB = true;
    commandConfigStruct[ADC0_A0B0].channelBNumber = 0;
    commandConfigStruct[ADC0_A0B0].sampleChannelMode = kLPADC_SampleChannelDualSingleEndBothSide;
    LPADC_SetConvCommandConfig(ADC0, 5, &commandConfigStruct[ADC0_A0B0]);
    /* Set trigger configuration. */
    LPADC_GetDefaultConvTriggerConfig(&triggerConfigStruct[ADC0_A0B0]);
    triggerConfigStruct[ADC0_A0B0].targetCommandId       = 5U;
    triggerConfigStruct[ADC0_A0B0].enableHardwareTrigger = false;
    triggerConfigStruct[ADC0_A0B0].channelBFIFOSelect = 1;
    triggerConfigStruct[ADC0_A0B0].channelAFIFOSelect = 0;


    /* Set conversion CMD configuration. */
    LPADC_GetDefaultConvCommandConfig(&commandConfigStruct[ADC1_A0B0]);
    commandConfigStruct[ADC1_A0B0].channelNumber = 0U;
    commandConfigStruct[ADC1_A0B0].enableChannelB = true;
    commandConfigStruct[ADC1_A0B0].channelBNumber = 0;
    commandConfigStruct[ADC1_A0B0].sampleChannelMode = kLPADC_SampleChannelDualSingleEndBothSide;
    LPADC_SetConvCommandConfig(ADC1, 6, &commandConfigStruct[ADC1_A0B0]);
    /* Set trigger configuration. */
    LPADC_GetDefaultConvTriggerConfig(&triggerConfigStruct[ADC1_A0B0]);
    triggerConfigStruct[ADC1_A0B0].targetCommandId       = 6U;
    triggerConfigStruct[ADC1_A0B0].enableHardwareTrigger = false;
    triggerConfigStruct[ADC1_A0B0].channelBFIFOSelect = 1;
    triggerConfigStruct[ADC1_A0B0].channelAFIFOSelect = 0;

    /* Set conversion CMD configuration. */
	LPADC_GetDefaultConvCommandConfig(&commandConfigStruct[ADC0_A3]);
	commandConfigStruct[ADC0_A3].channelNumber = 3;
	commandConfigStruct[ADC0_A3].sampleChannelMode = kLPADC_SampleChannelSingleEndSideA;
	LPADC_SetConvCommandConfig(ADC0, 11, &commandConfigStruct[ADC0_A3]);
	/* Set trigger configuration. */
	LPADC_GetDefaultConvTriggerConfig(&triggerConfigStruct[ADC0_A3]);
	triggerConfigStruct[ADC0_A3].targetCommandId       = 11U;
	triggerConfigStruct[ADC0_A3].enableHardwareTrigger = false;
	triggerConfigStruct[ADC0_A3].channelAFIFOSelect = 0;

    /* Set conversion CMD configuration. */
	LPADC_GetDefaultConvCommandConfig(&commandConfigStruct[ADC0_A7]);
	commandConfigStruct[ADC0_A7].channelNumber = 7;
	commandConfigStruct[ADC0_A7].sampleChannelMode = kLPADC_SampleChannelSingleEndSideA;
	LPADC_SetConvCommandConfig(ADC0, 12, &commandConfigStruct[ADC0_A7]);
	/* Set trigger configuration. */
	LPADC_GetDefaultConvTriggerConfig(&triggerConfigStruct[ADC0_A7]);
	triggerConfigStruct[ADC0_A7].targetCommandId       = 12U;
	triggerConfigStruct[ADC0_A7].enableHardwareTrigger = false;
	triggerConfigStruct[ADC0_A7].channelAFIFOSelect = 0;





    /* Set conversion CMD configuration. */
    LPADC_GetDefaultConvCommandConfig(&commandConfigStruct[ADC0_A6B6]);
    commandConfigStruct[ADC0_A6B6].channelNumber = 6U;
    commandConfigStruct[ADC0_A6B6].enableChannelB = true;
    commandConfigStruct[ADC0_A6B6].channelBNumber = 6;
    commandConfigStruct[ADC0_A6B6].sampleChannelMode = kLPADC_SampleChannelDualSingleEndBothSide;
    LPADC_SetConvCommandConfig(ADC0, 1, &commandConfigStruct[ADC0_A6B6]);
    /* Set trigger configuration. */
    LPADC_GetDefaultConvTriggerConfig(&triggerConfigStruct[ADC0_A6B6]);
    triggerConfigStruct[ADC0_A6B6].targetCommandId       = 1U;
    triggerConfigStruct[ADC0_A6B6].enableHardwareTrigger = false;
    triggerConfigStruct[ADC0_A6B6].channelBFIFOSelect = 1;
    triggerConfigStruct[ADC0_A6B6].channelAFIFOSelect = 0;
    


/* Set conversion CMD configuration. */
	LPADC_GetDefaultConvCommandConfig(&commandConfigStruct[ADC0_B1]);
	commandConfigStruct[ADC0_B1].enableChannelB = true;
	commandConfigStruct[ADC0_B1].channelBNumber = 1;
	commandConfigStruct[ADC0_B1].sampleChannelMode = kLPADC_SampleChannelSingleEndSideB;
	LPADC_SetConvCommandConfig(ADC0, 2, &commandConfigStruct[ADC0_B1]);
	/* Set trigger configuration. */
	LPADC_GetDefaultConvTriggerConfig(&triggerConfigStruct[ADC0_B1]);
	triggerConfigStruct[ADC0_B1].targetCommandId       = 2U;
	triggerConfigStruct[ADC0_B1].enableHardwareTrigger = false;
	triggerConfigStruct[ADC0_B1].channelBFIFOSelect = 0;

    /* Set conversion CMD configuration. */
	LPADC_GetDefaultConvCommandConfig(&commandConfigStruct[ADC1_A6]);
	commandConfigStruct[ADC1_A6].channelNumber = 6;
	commandConfigStruct[ADC1_A6].sampleChannelMode = kLPADC_SampleChannelSingleEndSideA;
	LPADC_SetConvCommandConfig(ADC1, 3, &commandConfigStruct[ADC1_A6]);
	/* Set trigger configuration. */
	LPADC_GetDefaultConvTriggerConfig(&triggerConfigStruct[ADC1_A6]);
	triggerConfigStruct[ADC1_A6].targetCommandId       = 3U;
	triggerConfigStruct[ADC1_A6].enableHardwareTrigger = false;
	triggerConfigStruct[ADC1_A6].channelAFIFOSelect = 0;

    /* Set conversion CMD configuration. */
	LPADC_GetDefaultConvCommandConfig(&commandConfigStruct[ADC1_B5]);
	commandConfigStruct[ADC1_B5].enableChannelB = true;
	commandConfigStruct[ADC1_B5].channelBNumber = 5;
	commandConfigStruct[ADC1_B5].sampleChannelMode = kLPADC_SampleChannelSingleEndSideB;
	LPADC_SetConvCommandConfig(ADC1, 4, &commandConfigStruct[ADC1_B5]);
	/* Set trigger configuration. */
	LPADC_GetDefaultConvTriggerConfig(&triggerConfigStruct[ADC1_B5]);
	triggerConfigStruct[ADC1_B5].targetCommandId       = 4U;
	triggerConfigStruct[ADC1_B5].enableHardwareTrigger = false;
	triggerConfigStruct[ADC1_B5].channelBFIFOSelect = 0;

    	/* Set conversion CMD configuration. */
	LPADC_GetDefaultConvCommandConfig(&commandConfigStruct[ADC1_B8]);
	commandConfigStruct[ADC1_B8].enableChannelB = true;
	commandConfigStruct[ADC1_B8].channelBNumber = 8;
	commandConfigStruct[ADC1_B8].sampleChannelMode = kLPADC_SampleChannelSingleEndSideB;
	LPADC_SetConvCommandConfig(ADC1, 7, &commandConfigStruct[ADC1_B8]);
	/* Set trigger configuration. */
	LPADC_GetDefaultConvTriggerConfig(&triggerConfigStruct[ADC1_B8]);
	triggerConfigStruct[ADC1_B8].targetCommandId       = 7U;
	triggerConfigStruct[ADC1_B8].enableHardwareTrigger = false;
	triggerConfigStruct[ADC1_B8].channelBFIFOSelect = 0;

    /* Set conversion CMD configuration. */
	LPADC_GetDefaultConvCommandConfig(&commandConfigStruct[ADC1_B9]);
	commandConfigStruct[ADC1_B9].enableChannelB = true;
	commandConfigStruct[ADC1_B9].channelBNumber = 9;
	commandConfigStruct[ADC1_B9].sampleChannelMode = kLPADC_SampleChannelSingleEndSideB;
	LPADC_SetConvCommandConfig(ADC1, 8, &commandConfigStruct[ADC1_B9]);
	/* Set trigger configuration. */
	LPADC_GetDefaultConvTriggerConfig(&triggerConfigStruct[ADC1_B9]);
	triggerConfigStruct[ADC1_B9].targetCommandId       = 8U;
	triggerConfigStruct[ADC1_B9].enableHardwareTrigger = false;
	triggerConfigStruct[ADC1_B9].channelBFIFOSelect = 0;

    /* Set conversion CMD configuration. */
	LPADC_GetDefaultConvCommandConfig(&commandConfigStruct[ADC1_B10]);
	commandConfigStruct[ADC1_B10].enableChannelB = true;
	commandConfigStruct[ADC1_B10].channelBNumber = 10;
	commandConfigStruct[ADC1_B10].sampleChannelMode = kLPADC_SampleChannelSingleEndSideB;
	LPADC_SetConvCommandConfig(ADC1, 9, &commandConfigStruct[ADC1_B10]);
	/* Set trigger configuration. */
	LPADC_GetDefaultConvTriggerConfig(&triggerConfigStruct[ADC1_B10]);
	triggerConfigStruct[ADC1_B10].targetCommandId       = 9U;
	triggerConfigStruct[ADC1_B10].enableHardwareTrigger = false;
	triggerConfigStruct[ADC1_B10].channelBFIFOSelect = 0;

    /* Set conversion CMD configuration. */
	LPADC_GetDefaultConvCommandConfig(&commandConfigStruct[ADC1_B11]);
	commandConfigStruct[ADC1_B11].enableChannelB = true;
	commandConfigStruct[ADC1_B11].channelBNumber = 11;
	commandConfigStruct[ADC1_B11].sampleChannelMode = kLPADC_SampleChannelSingleEndSideB;
	LPADC_SetConvCommandConfig(ADC1, 10, &commandConfigStruct[ADC1_B11]);
	/* Set trigger configuration. */
	LPADC_GetDefaultConvTriggerConfig(&triggerConfigStruct[ADC1_B11]);
	triggerConfigStruct[ADC1_B11].targetCommandId       = 10U;
	triggerConfigStruct[ADC1_B11].enableHardwareTrigger = false;
	triggerConfigStruct[ADC1_B11].channelBFIFOSelect = 0;

}


void DAC_Configure(void)
{
    /* enable DAC0 DAC1 and VREF */
    SPC0->ACTIVE_CFG1 |= 0x31;

    /* Configure the DAC. */
	DAC_GetDefaultConfig(&dacConfigStruct);
	dacConfigStruct.referenceVoltageSource = kDAC_ReferenceVoltageSourceAlt1;
	DAC_Init(DAC0, &dacConfigStruct);
	DAC_Enable(DAC0, true); /* Enable the logic and output. */

    /* Configure the DAC. */
	DAC_GetDefaultConfig(&dacConfigStruct);
	dacConfigStruct.referenceVoltageSource = kDAC_ReferenceVoltageSourceAlt1;
	DAC_Init(DAC1, &dacConfigStruct);
	DAC_Enable(DAC1, true); /* Enable the logic and output. */

	
}

void CAN_Configure(void)
{
    /* Get FlexCAN module default Configuration. */
    FLEXCAN_GetDefaultConfig(&flexcanConfig);

    /* 修改配置：禁用自我接收 */
    flexcanConfig.disableSelfReception = true;  // 禁用接收自己发送的报文

    flexcanConfig.bitRate = 500000U;

    flexcan_timing_config_t timing_config;
    memset(&timing_config, 0, sizeof(flexcan_timing_config_t));

    if (FLEXCAN_CalculateImprovedTimingValues(CAN0, flexcanConfig.bitRate, CAN_CLK_FREQ, &timing_config))
    {
        /* Update the improved timing configuration*/
        memcpy(&(flexcanConfig.timingConfig), &timing_config, sizeof(flexcan_timing_config_t));
    }
    else
    {
    	PRINTF("No found Improved Timing Configuration. Just used default configuration\r\n\r\n");
    }

    FLEXCAN_Init(CAN0, &flexcanConfig, CAN_CLK_FREQ);

    /* Create FlexCAN handle structure and set call back function. */
//    FLEXCAN_TransferCreateHandle(CAN0, &flexcanHandle, flexcan_callback, NULL);

    /* Set Rx Masking mechanism. */
    FLEXCAN_SetRxMbGlobalMask(CAN0, 0);

    /* Setup Rx Message Buffer. */
    mbConfig.format = kFLEXCAN_FrameFormatStandard;
    mbConfig.type   = kFLEXCAN_FrameTypeData;
    mbConfig.id     = FLEXCAN_ID_STD(0);

    FLEXCAN_SetRxMbConfig(CAN0, RX_MESSAGE_BUFFER_NUM, &mbConfig, true);

    FLEXCAN_SetTxMbConfig(CAN0, TX_MESSAGE_BUFFER_NUM, true);

/* Start receive data through Rx Message Buffer. */
    rxXfer.mbIdx = (uint8_t)RX_MESSAGE_BUFFER_NUM;

    rxXfer.frame = &frame;
    
    /* 启动接收 */
    if (FLEXCAN_TransferReceiveNonBlocking(CAN0, &flexcanHandle, &rxXfer) != kStatus_Success)
    {
        PRINTF("Failed to start CAN reception!\r\n");
    }
    
    /* 启用CAN中断 */
    FLEXCAN_EnableInterrupts(CAN0, kFLEXCAN_WakeUpInterruptEnable);
    NVIC_SetPriority(CAN0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
    NVIC_EnableIRQ(CAN0_IRQn);
	
}

void UART_Configure(void)
{
	// LPUART_GetDefaultConfig(&config);
	// config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
	// config.enableTx     = true;
	// config.enableRx     = true;

	// LPUART_Init(LPUART4, &config, LPUART_CLK_FREQ);
	// LPUART_TransferCreateHandle(LPUART4, &g_lpuartHandle, LPUART_UserCallback, NULL);

	// LPUART_EnableInterrupts(LPUART4, kLPUART_RxDataRegFullInterruptEnable);
    // NVIC_SetPriority(LP_FLEXCOMM4_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2); // 设置中断优先级（如优先级 3，需根据实际调整）
    // NVIC_EnableIRQ(LP_FLEXCOMM4_IRQn);       // 启用 NVIC 中断

	// /* Send g_tipString out. */
	// xfer.data     = g_tipString;
	// xfer.dataSize = sizeof(g_tipString) - 1;
	// txOnGoing     = true;
	// LPUART_TransferSendNonBlocking(LPUART4, &g_lpuartHandle, &xfer);


    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
    config.enableTx     = true;
    config.enableRx     = true;

    LPUART_Init(LPUART4, &config, DEMO_LPUART_CLK_FREQ);

    /* Send g_tipString out. */
    LPUART_WriteBlocking(LPUART4, g_tipString, sizeof(g_tipString) / sizeof(g_tipString[0]));

    /* Enable RX interrupt. */
    LPUART_EnableInterrupts(LPUART4, kLPUART_RxDataRegFullInterruptEnable);
    NVIC_SetPriority(DEMO_LPUART_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2); 
    EnableIRQ(DEMO_LPUART_IRQn);


}

void WWDT_Configure(void)
{
    wwdt_config_t config;
    uint32_t wdtFreq;

    /* Enable FRO 1M clock for WWDT module. */
    SYSCON->CLOCK_CTRL |= SYSCON_CLOCK_CTRL_FRO1MHZ_CLK_ENA_MASK;
    /* The WDT divides the input frequency into it by 4 */
    wdtFreq = CLOCK_GetWdtClkFreq(0) / 4;

    WWDT_GetDefaultConfig(&config);


    /*
	 * Set watchdog feed time constant to approximately 1s
	 * Set watchdog warning time to 512 ticks after feed time constant
	 * Set watchdog window time to 1s
	 */
	config.timeoutValue = wdtFreq * 1;
	/* Configure WWDT to reset on timeout */
	config.enableWatchdogReset = true;
	/* Setup watchdog clock frequency(Hz). */
	config.clockFreq_Hz = CLOCK_GetWdtClkFreq(0);
	WWDT_Init(WWDT0, &config);
}

void PWM_Configure(void)
{




}

