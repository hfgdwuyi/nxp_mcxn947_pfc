/*
 * main.c - FreeRTOS entry point
 *
 * Responsibilities:
 *   - Board/hardware initialization
 *   - IPC object creation (queues, semaphores, mutexes)
 *   - Task creation
 *   - FreeRTOS hook functions
 *
 * Hardware configuration and task implementations live in their
 * respective modules: command.c, can.c, sensor.c, lcd.c, key.c,
 * fan.c, pwm_config.c, power_state.c
 */

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/* Board & SDK */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "fsl_wwdt.h"
#include "fsl_spc.h"
#include "fsl_dac.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"

/* Application modules */
#include "main.h"
#include "command.h"
#include "lcd.h"
#include "sensor.h"
#include "can.h"
#include "key.h"
#include "fan.h"
#include "pwm_config.h"
#include "power_state.h"

/*========================================================================
 * Task priorities and periods
 *========================================================================*/
#define TASK_PRIORITY_SENSOR     4
#define TASK_PRIORITY_COMMAND    3
#define TASK_PRIORITY_CAN_RX     3
#define TASK_PRIORITY_CAN_TX     3
#define TASK_PRIORITY_KEY        2
#define TASK_PRIORITY_FAN        2
#define TASK_PRIORITY_DISPLAY    1
#define TASK_PRIORITY_SELFCHECK  1
#define TASK_PRIORITY_PFCCTRL    1
#define TASK_PRIORITY_DCCTRL     1

#define TASK_STACK_SENSOR     1024
#define TASK_STACK_COMMAND     512
#define TASK_STACK_CAN_RX     1024
#define TASK_STACK_CAN_TX     1024
#define TASK_STACK_KEY         125
#define TASK_STACK_FAN        1024
#define TASK_STACK_DISPLAY     512
#define TASK_STACK_SELFCHECK   256
#define TASK_STACK_PFCCTRL     256
#define TASK_STACK_DCCTRL      256

#define LED_TIMER_PERIOD_MS    200

/*========================================================================
 * Local functions
 *========================================================================*/
static void prvLedTimerCallback(TimerHandle_t xTimer);
static void prvWWDT_Configure(void);
static void prvDAC_Configure(void);

/*========================================================================
 * main() - Entry point
 *========================================================================*/
int main(void)
{
    TimerHandle_t xLedTimer = NULL;

    /* ---- Clock distribution ---- */
    CLOCK_SetClkDiv(kCLOCK_DivFlexcom4Clk, 1u);
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_SetClkDiv(kCLOCK_DivAdc0Clk, 1U);
    CLOCK_AttachClk(kFRO_HF_to_ADC0);
    CLOCK_SetClkDiv(kCLOCK_DivAdc1Clk, 1U);
    CLOCK_AttachClk(kFRO_HF_to_ADC1);
    CLOCK_SetClkDiv(kCLOCK_DivDac0Clk, 1u);
    CLOCK_AttachClk(kFRO_HF_to_DAC0);
    CLOCK_SetClkDiv(kCLOCK_DivDac1Clk, 1u);
    CLOCK_AttachClk(kFRO_HF_to_DAC1);
    CLOCK_SetClkDiv(kCLOCK_DivPllClk, 2U);
    CLOCK_SetClkDiv(kCLOCK_DivFlexcan0Clk, 1U);
    CLOCK_AttachClk(kPLL0_to_FLEXCAN0);
    CLOCK_SetClkDiv(kCLOCK_DivWdt0Clk, 1U);
    CLOCK_SetClkDiv(kCLOCK_DivFlexioClk, 4u);
    CLOCK_AttachClk(kFRO_HF_to_FLEXIO);
    CLOCK_SetClkDiv(kCLOCK_DivCtimer0Clk, 1u);
    CLOCK_AttachClk(kFRO_HF_to_CTIMER0);

    /* Enable PWM1 sub-clocks */
    SYSCON->PWM1SUBCTL |=
        (SYSCON_PWM1SUBCTL_CLK0_EN_MASK | SYSCON_PWM1SUBCTL_CLK1_EN_MASK
        | SYSCON_PWM1SUBCTL_CLK2_EN_MASK | SYSCON_PWM1SUBCTL_CLK3_EN_MASK);

    /* ---- Board init ---- */
    BOARD_InitPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    /* ---- Peripheral configuration ---- */
    UART_Configure();
    ADC_Configure();
    prvDAC_Configure();
    CAN_Configure();
    prvWWDT_Configure();
    PWM_Configure();

    PRINTF("MCU init complete\r\n");

    /* ---- IPC objects ---- */
    xCanRxQueue = xQueueCreate(5, sizeof(can_message_t));
    if (xCanRxQueue == NULL) {
        PRINTF("Error: Failed to create CAN Rx queue!\r\n");
        while (1);
    }

    xUartRxQueue = xQueueCreate(5, sizeof(uart_message_t));
    if (xUartRxQueue == NULL) {
        PRINTF("Error: Failed to create UART Rx queue!\r\n");
        while (1);
    }

    xKeyMessageQueue = xQueueCreate(10, sizeof(key_message_t));
    if (xKeyMessageQueue == NULL) {
        PRINTF("Error: Failed to create key message queue!\r\n");
        while (1);
    }

    /* ---- Task creation ---- */
    xTaskCreate(prvKeyTask,        "key",        TASK_STACK_KEY,        NULL, TASK_PRIORITY_KEY,        NULL);
    xTaskCreate(prvUartRxTask,     "command",    TASK_STACK_COMMAND,    NULL, TASK_PRIORITY_COMMAND,    NULL);
    xTaskCreate(prvDisplayTask,    "display",    TASK_STACK_DISPLAY,    NULL, TASK_PRIORITY_DISPLAY,    NULL);
    xTaskCreate(prvSensorTask,     "sensor",     TASK_STACK_SENSOR,     NULL, TASK_PRIORITY_SENSOR,     NULL);
    xTaskCreate(prvFanTask,        "fan",        TASK_STACK_FAN,        NULL, TASK_PRIORITY_FAN,        NULL);
    xTaskCreate(vTaskCANRx,        "CAN_RX",     TASK_STACK_CAN_RX,     NULL, TASK_PRIORITY_CAN_RX,     NULL);
    xTaskCreate(vTaskCANTx,        "CAN_TX",     TASK_STACK_CAN_TX,     NULL, TASK_PRIORITY_CAN_TX,     NULL);
    xTaskCreate(prvSelfCheckTask,  "SelfCheck",  TASK_STACK_SELFCHECK,  NULL, TASK_PRIORITY_SELFCHECK,  NULL);
    xTaskCreate(prvPFCControlTask, "PFCControl", TASK_STACK_PFCCTRL,    NULL, TASK_PRIORITY_PFCCTRL,    NULL);
    xTaskCreate(prvDCControlTask,  "DCControl",  TASK_STACK_DCCTRL,     NULL, TASK_PRIORITY_DCCTRL,     NULL);

    /* ---- LED / WWDT refresh timer ---- */
    xLedTimer = xTimerCreate("LEDTimer",
                             pdMS_TO_TICKS(LED_TIMER_PERIOD_MS),
                             pdTRUE,
                             (void *)0,
                             prvLedTimerCallback);
    if (xLedTimer != NULL) {
        xTimerStart(xLedTimer, 0);
    }

    /* ---- Start the scheduler ---- */
    vTaskStartScheduler();

    /* Should never reach here */
    for (;;);
}

/*========================================================================
 * LED Timer callback - toggles debug LEDs and refreshes WWDT
 *========================================================================*/
static void prvLedTimerCallback(TimerHandle_t xTimer)
{
    (void)xTimer;

    GPIO_PortToggle(DBG_LED_GPIO, 1u << DBG_LED0_PIN);
    GPIO_PortToggle(DBG_LED_GPIO, 1u << DBG_LED1_PIN);
    GPIO_PortToggle(DBG_LED_GPIO, 1u << DBG_LED2_PIN);
    GPIO_PortToggle(CAN_LED_GPIO, 1u << CAN_LED_RUN_PIN);
    GPIO_PortToggle(CAN_LED_GPIO, 1u << CAN_LED_ERROR_PIN);

    WWDT_Refresh(WWDT0);
}

/*========================================================================
 * DAC configuration
 *========================================================================*/
static void prvDAC_Configure(void)
{
    dac_config_t dacConfigStruct;

    SPC0->ACTIVE_CFG1 |= 0x31;

    DAC_GetDefaultConfig(&dacConfigStruct);
    dacConfigStruct.referenceVoltageSource = kDAC_ReferenceVoltageSourceAlt1;
    DAC_Init(DAC0, &dacConfigStruct);
    DAC_Enable(DAC0, true);

    DAC_GetDefaultConfig(&dacConfigStruct);
    dacConfigStruct.referenceVoltageSource = kDAC_ReferenceVoltageSourceAlt1;
    DAC_Init(DAC1, &dacConfigStruct);
    DAC_Enable(DAC1, true);
}

/*========================================================================
 * WWDT configuration (~1s timeout, reset on timeout)
 *========================================================================*/
static void prvWWDT_Configure(void)
{
    wwdt_config_t config;
    uint32_t wdtFreq;

    SYSCON->CLOCK_CTRL |= SYSCON_CLOCK_CTRL_FRO1MHZ_CLK_ENA_MASK;
    wdtFreq = CLOCK_GetWdtClkFreq(0) / 4;

    WWDT_GetDefaultConfig(&config);
    config.timeoutValue = wdtFreq * 1;
    config.enableWatchdogReset = true;
    config.clockFreq_Hz = CLOCK_GetWdtClkFreq(0);
    WWDT_Init(WWDT0, &config);
}

/*========================================================================
 * FreeRTOS Hook Functions
 *========================================================================*/

void vApplicationMallocFailedHook(void)
{
    PRINTF("Memory allocation failed!\r\n");
    for (;;);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)pcTaskName;
    (void)xTask;
    for (;;);
}

void vApplicationTickHook(void)
{
    /* Idle - not used; LED timer handles periodic tasks */
}

void vApplicationIdleHook(void)
{
    /* Idle - heap monitoring could go here */
}
