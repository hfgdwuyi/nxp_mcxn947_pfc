/*
 * command.c - UART command interface
 *
 * Handles UART RX/TX, command parsing, and the UART receive task.
 */

#include "command.h"
#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "fsl_lpuart.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_dac.h"
#include "fsl_pwm.h"
#include "board.h"
#include "clock_config.h"
#include "main.h"
#include "sensor.h"

#define LPUART_CLK_FREQ           CLOCK_GetLPFlexCommClkFreq(4u)
#define DEMO_LPUART_CLK_FREQ      CLOCK_GetLPFlexCommClkFreq(4u)
#define DEMO_LPUART_IRQn          LP_FLEXCOMM4_IRQn
#define DEMO_LPUART_IRQHandler    LP_FLEXCOMM4_IRQHandler

/* Command string constants */
#define CMD_SS_RLY_EN         "ss_rly_enable"
#define CMD_SS_RLY_DIS        "ss_rly_disable"
#define CMD_PFC1_EN           "pfc1_enable"
#define CMD_PFC1_DIS          "pfc1_disable"
#define CMD_PFC2_EN           "pfc2_enable"
#define CMD_PFC2_DIS          "pfc2_disable"
#define CMD_PFC1_FLT_DETECT   "pfc1_flt_detect"
#define CMD_PFC2_FLT_DETECT   "pfc2_flt_detect"
#define CMD_SYS_CFG0_DETECT   "sys_cfg0_detect"
#define CMD_SYS_CFG1_DETECT   "sys_cfg1_detect"
#define CMD_SYS_CFG2_DETECT   "sys_cfg2_detect"
#define CMD_FAN1_FB_DETECT    "fan1_fb_detect"
#define CMD_FAN2_FB_DETECT    "fan2_fb_detect"
#define CMD_FAN3_FB_DETECT    "fan3_fb_detect"
#define CMD_DCDC1_EN          "dcdc1_enable"
#define CMD_DCDC1_DIS         "dcdc1_disable"
#define CMD_DCDC2_EN          "dcdc2_enable"
#define CMD_DCDC2_DIS         "dcdc2_disable"
#define CMD_READ_ADC          "get_adc"
#define CMD_WRITE_DAC0        "set_dac0"
#define CMD_WRITE_DAC1        "set_dac1"
#define CMD_WRITE_PWM1        "set_pwm1"
#define CMD_WRITE_PWM2        "set_pwm2"
#define CMD_WRITE_PWM3        "set_pwm3"

/* Queue and mutex - owned by this module */
QueueHandle_t xUartRxQueue = NULL;
SemaphoreHandle_t xUartTxMutex = NULL;

static lpuart_config_t config;

/* Command handler prototypes */
static void handle_SS_RLY_EN_Command(char *ptr);
static void handle_SS_PLY_DIS_Command(char *ptr);
static void handle_PFC1_EN_Command(char *ptr);
static void handle_PFC1_DIS_Command(char *ptr);
static void handle_PFC2_EN_Command(char *ptr);
static void handle_PFC2_DIS_Command(char *ptr);
static void handle_PFC1_FLT_DETECT_Command(char *ptr);
static void handle_PFC2_FLT_DETECT_Command(char *ptr);
static void handle_SYS_CFG0_DETECT_Command(char *ptr);
static void handle_SYS_CFG1_DETECT_Command(char *ptr);
static void handle_SYS_CFG2_DETECT_Command(char *ptr);
static void handle_FAN1_FB_DETECT_Command(char *ptr);
static void handle_FAN2_FB_DETECT_Command(char *ptr);
static void handle_FAN3_FB_DETECT_Command(char *ptr);
static void handle_DCDC1_EN_Command(char *ptr);
static void handle_DCDC1_DIS_Command(char *ptr);
static void handle_DCDC2_EN_Command(char *ptr);
static void handle_DCDC2_DIS_Command(char *ptr);
static void handle_ReadAdc_Command(char *ptr);
static void handle_WriteDac0_Command(char *ptr);
static void handle_WriteDac1_Command(char *ptr);
static void handle_WritePWM1_Command(char *ptr);
static void handle_WritePWM2_Command(char *ptr);
static void handle_WritePWM3_Command(char *ptr);

typedef struct {
    const char *name;
    void (*handler)(char *ptr);
} Command;

static Command commands[] = {
    {CMD_SS_RLY_EN,         handle_SS_RLY_EN_Command},
    {CMD_SS_RLY_DIS,        handle_SS_PLY_DIS_Command},
    {CMD_PFC1_EN,           handle_PFC1_EN_Command},
    {CMD_PFC1_DIS,          handle_PFC1_DIS_Command},
    {CMD_PFC2_EN,           handle_PFC2_EN_Command},
    {CMD_PFC2_DIS,          handle_PFC2_DIS_Command},
    {CMD_PFC1_FLT_DETECT,   handle_PFC1_FLT_DETECT_Command},
    {CMD_PFC2_FLT_DETECT,   handle_PFC2_FLT_DETECT_Command},
    {CMD_SYS_CFG0_DETECT,   handle_SYS_CFG0_DETECT_Command},
    {CMD_SYS_CFG1_DETECT,   handle_SYS_CFG1_DETECT_Command},
    {CMD_SYS_CFG2_DETECT,   handle_SYS_CFG2_DETECT_Command},
    {CMD_FAN1_FB_DETECT,    handle_FAN1_FB_DETECT_Command},
    {CMD_FAN2_FB_DETECT,    handle_FAN2_FB_DETECT_Command},
    {CMD_FAN3_FB_DETECT,    handle_FAN3_FB_DETECT_Command},
    {CMD_DCDC1_EN,          handle_DCDC1_EN_Command},
    {CMD_DCDC1_DIS,         handle_DCDC1_DIS_Command},
    {CMD_DCDC2_EN,          handle_DCDC2_EN_Command},
    {CMD_DCDC2_DIS,         handle_DCDC2_DIS_Command},
    {CMD_READ_ADC,          handle_ReadAdc_Command},
    {CMD_WRITE_DAC0,        handle_WriteDac0_Command},
    {CMD_WRITE_DAC1,        handle_WriteDac1_Command},
    {CMD_WRITE_PWM1,        handle_WritePWM1_Command},
    {CMD_WRITE_PWM2,        handle_WritePWM2_Command},
    {CMD_WRITE_PWM3,        handle_WritePWM3_Command}
};

#define COMMAND_COUNT (sizeof(commands) / sizeof(commands[0]))

static uint32_t dacValue;
static uint32_t pwmValue;

/*================================================================
 * UART Configuration
 *================================================================*/
void UART_Configure(void)
{
    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
    config.enableTx     = true;
    config.enableRx     = true;

    LPUART_Init(LPUART4, &config, DEMO_LPUART_CLK_FREQ);

    LPUART_EnableInterrupts(LPUART4, kLPUART_RxDataRegFullInterruptEnable);
    NVIC_SetPriority(DEMO_LPUART_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);
    EnableIRQ(DEMO_LPUART_IRQn);
}

/*================================================================
 * UART ISR - minimal: receives byte, builds message, sends to queue
 *================================================================*/
void DEMO_LPUART_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static uart_message_t msg = {0};
    static uint16_t bufIndex = 0;
    uint32_t statusFlags = LPUART_GetStatusFlags(LPUART4);

    if (statusFlags & kLPUART_RxDataRegFullFlag) {
        uint8_t ch = LPUART_ReadByte(LPUART4);
        LPUART_ClearStatusFlags(LPUART4, kLPUART_RxDataRegFullFlag);

        taskENTER_CRITICAL_FROM_ISR();

        if (bufIndex < sizeof(msg.data) - 1) {
            msg.data[bufIndex++] = ch;

            if (ch == '\n' || ch == '\r') {
                msg.length = bufIndex;
                msg.data[bufIndex] = '\0';

                if (xUartRxQueue != NULL) {
                    if (xQueueSendFromISR(xUartRxQueue, &msg, &xHigherPriorityTaskWoken) != pdPASS) {
                        PRINTF("UART Rx queue is full!\r\n");
                    }
                }
                bufIndex = 0;
                memset(msg.data, 0, sizeof(msg.data));
            }
        } else {
            bufIndex = 0;
        }

        taskEXIT_CRITICAL_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*================================================================
 * UART RX Task - blocks on queue, dispatches to command handler
 *================================================================*/
void prvUartRxTask(void *pvParameters)
{
    (void)pvParameters;
    uart_message_t rxMsg;
    PRINTF("UART receive task started\r\n");

    for (;;) {
        if (xQueueReceive(xUartRxQueue, &rxMsg, portMAX_DELAY) == pdTRUE) {
            if (rxMsg.length > 0 && rxMsg.length <= sizeof(rxMsg.data)) {
                processReceivedCommand((char *)rxMsg.data);
            }
        }
    }
}

/*================================================================
 * Command parser
 *================================================================*/
void processReceivedCommand(const char *data)
{
    char temp[RX_BUFFER_SIZE] = {0};
    bool commandMatched = false;

    strncpy(temp, data, sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0';

    char *ptr = temp;
    size_t len = strlen(ptr);
    while (len > 0 && (ptr[len - 1] == '\r' || ptr[len - 1] == '\n')) {
        ptr[len - 1] = '\0';
        len--;
    }

    for (uint32_t i = 0; i < COMMAND_COUNT; i++) {
        if (strstr(ptr, commands[i].name) != NULL) {
            commands[i].handler(ptr);
            commandMatched = true;
            PRINTF("Command matched: %s\r\n", commands[i].name);
            break;
        }
    }

    if (!commandMatched) {
        PRINTF("Unknown command: %s\r\n", ptr);
    }
}

/*================================================================
 * Command handlers
 *================================================================*/
static void handle_SS_RLY_EN_Command(char *ptr) {
    (void)ptr;
    PRINTF("You enable SS-RLY\r\n");
    GPIO_PortSet(SS_RLY_EN_GPIO, 1u << SS_RLY_EN_GPIO_PIN);
}

static void handle_SS_PLY_DIS_Command(char *ptr) {
    (void)ptr;
    PRINTF("You disable SS-RLY\r\n");
    GPIO_PortClear(SS_RLY_EN_GPIO, 1u << SS_RLY_EN_GPIO_PIN);
}

static void handle_PFC1_FLT_DETECT_Command(char *ptr) {
    (void)ptr;
    uint32_t signalActive = GPIO_PinRead(PFC1_FLT_GPIO, PFC1_FLT_GPIO_PIN);
    PRINTF("PFC1_FLT is %s level\r\n", signalActive ? "high" : "low");
}

static void handle_PFC2_FLT_DETECT_Command(char *ptr) {
    (void)ptr;
    uint32_t signalActive = GPIO_PinRead(PFC2_FLT_GPIO, PFC2_FLT_GPIO_PIN);
    PRINTF("PFC2_FLT is %s level\r\n", signalActive ? "high" : "low");
}

static void handle_SYS_CFG0_DETECT_Command(char *ptr) {
    (void)ptr;
    uint32_t signalActive = GPIO_PinRead(SYS_CFG0_GPIO, SYS_CFG0_GPIO_PIN);
    PRINTF("SYS_CFG0 is %s level\r\n", signalActive ? "high" : "low");
}

static void handle_SYS_CFG1_DETECT_Command(char *ptr) {
    (void)ptr;
    uint32_t signalActive = GPIO_PinRead(SYS_CFG1_GPIO, SYS_CFG1_GPIO_PIN);
    PRINTF("SYS_CFG1 is %s level\r\n", signalActive ? "high" : "low");
}

static void handle_SYS_CFG2_DETECT_Command(char *ptr) {
    (void)ptr;
    uint32_t signalActive = GPIO_PinRead(SYS_CFG2_GPIO, SYS_CFG2_GPIO_PIN);
    PRINTF("SYS_CFG2 is %s level\r\n", signalActive ? "high" : "low");
}

static void handle_FAN1_FB_DETECT_Command(char *ptr) {
    (void)ptr;
    uint32_t signalActive = GPIO_PinRead(PWM1_FB_GPIO, PWM1_FB_GPIO_PIN);
    PRINTF("PWM1_FB is %s level\r\n", signalActive ? "high" : "low");
}

static void handle_FAN2_FB_DETECT_Command(char *ptr) {
    (void)ptr;
    uint32_t signalActive = GPIO_PinRead(PWM2_FB_GPIO, PWM2_FB_GPIO_PIN);
    PRINTF("PWM2_FB is %s level\r\n", signalActive ? "high" : "low");
}

static void handle_FAN3_FB_DETECT_Command(char *ptr) {
    (void)ptr;
    uint32_t signalActive = GPIO_PinRead(PWM3_FB_GPIO, PWM3_FB_GPIO_PIN);
    PRINTF("PWM3_FB is %s level\r\n", signalActive ? "high" : "low");
}

static void handle_PFC1_EN_Command(char *ptr) {
    (void)ptr;
    PRINTF("You enable PFC1\r\n");
    GPIO_PortSet(PFC1_EN_GPIO, 1u << PFC1_EN_GPIO_PIN);
}

static void handle_PFC1_DIS_Command(char *ptr) {
    (void)ptr;
    PRINTF("You disable PFC1\r\n");
    GPIO_PortClear(PFC1_EN_GPIO, 1u << PFC1_EN_GPIO_PIN);
}

static void handle_PFC2_EN_Command(char *ptr) {
    (void)ptr;
    PRINTF("You enable PFC2\r\n");
    GPIO_PortSet(PFC2_EN_GPIO, 1u << PFC2_EN_GPIO_PIN);
}

static void handle_PFC2_DIS_Command(char *ptr) {
    (void)ptr;
    PRINTF("You disable PFC2\r\n");
    GPIO_PortClear(PFC2_EN_GPIO, 1u << PFC2_EN_GPIO_PIN);
}

static void handle_DCDC1_EN_Command(char *ptr) {
    (void)ptr;
    PRINTF("You enable DCDC1\r\n");
    GPIO_PortSet(DCDC1_EN_GPIO, 1u << DCDC1_EN_GPIO_PIN);
}

static void handle_DCDC1_DIS_Command(char *ptr) {
    (void)ptr;
    PRINTF("You disable DCDC1\r\n");
    GPIO_PortClear(DCDC1_EN_GPIO, 1u << DCDC1_EN_GPIO_PIN);
}

static void handle_DCDC2_EN_Command(char *ptr) {
    (void)ptr;
    PRINTF("You enable DCDC2\r\n");
    GPIO_PortSet(DCDC2_EN_GPIO, 1u << DCDC2_EN_GPIO_PIN);
}

static void handle_DCDC2_DIS_Command(char *ptr) {
    (void)ptr;
    PRINTF("You disable DCDC2\r\n");
    GPIO_PortClear(DCDC2_EN_GPIO, 1u << DCDC2_EN_GPIO_PIN);
}

static void handle_WritePWM1_Command(char *ptr) {
    char *paramStart = strstr(ptr, ":");
    if (paramStart != NULL) {
        paramStart++;
        while (*paramStart == ' ' || *paramStart == '\t') paramStart++;
        pwmValue = atoi(paramStart);
        if (pwmValue > 100) {
            PRINTF("Invalid, out of PWM range\r\n");
            return;
        }
        PRINTF("Write PWM1 value: %d\r\n", pwmValue);
        PWM_UpdatePwmDutycycle(PWM1, kPWM_Module_3, kPWM_PwmA, kPWM_SignedCenterAligned, pwmValue);
        PWM_SetPwmLdok(PWM1, kPWM_Control_Module_3, true);
    }
}

static void handle_WritePWM2_Command(char *ptr) {
    char *paramStart = strstr(ptr, ":");
    if (paramStart != NULL) {
        paramStart++;
        while (*paramStart == ' ' || *paramStart == '\t') paramStart++;
        pwmValue = atoi(paramStart);
        if (pwmValue > 100) {
            PRINTF("Invalid, out of PWM range\r\n");
            return;
        }
        PRINTF("Write PWM2 value: %d\r\n", pwmValue);
        PWM_UpdatePwmDutycycle(PWM1, kPWM_Module_3, kPWM_PwmB, kPWM_SignedCenterAligned, pwmValue);
        PWM_SetPwmLdok(PWM1, kPWM_Control_Module_3, true);
    }
}

static void handle_WritePWM3_Command(char *ptr) {
    char *paramStart = strstr(ptr, ":");
    if (paramStart != NULL) {
        paramStart++;
        while (*paramStart == ' ' || *paramStart == '\t') paramStart++;
        pwmValue = atoi(paramStart);
        if (pwmValue > 100) {
            PRINTF("Invalid, out of PWM range\r\n");
            return;
        }
        PRINTF("Write PWM3 value: %d\r\n", pwmValue);
        PWM_UpdatePwmDutycycle(PWM1, kPWM_Module_0, kPWM_PwmB, kPWM_SignedCenterAligned, pwmValue);
        PWM_SetPwmLdok(PWM1, kPWM_Control_Module_0, true);
    }
}

static void handle_ReadAdc_Command(char *ptr) {
    (void)ptr;
    PRINTF("VOUT_DC2_ADC value: %d\r\n", adcValue[VOUT_DC2_ADC]);
    PRINTF("AMB_TEMP_ADC value: %d\r\n", adcValue[AMB_TEMP_ADC]);
    PRINTF("IOUT_DC2_ADC value: %d\r\n", adcValue[IOUT_DC2_ADC]);
    PRINTF("VOUT_DC1_ADC value: %d\r\n", adcValue[VOUT_DC1_ADC]);
    PRINTF("IOUT_DC1_ADC value: %d\r\n", adcValue[IOUT_DC1_ADC]);
    PRINTF("DCDC1_TEMP_ADC value: %d\r\n", adcValue[DCDC1_TEMP_ADC]);
    PRINTF("DCDC2_TEMP_ADC value: %d\r\n", adcValue[DCDC2_TEMP_ADC]);
    PRINTF("DCI1_ADC value: %d\r\n", adcValue[DCI1_ADC]);
    PRINTF("DCI2_ADC value: %d\r\n", adcValue[DCI2_ADC]);
    PRINTF("ANALOG_KEYBOARD_ADC value: %d\r\n", adcValue[ANALOG_KEYBOARD_ADC]);
    PRINTF("PFC_I_IN_ADC value: %d\r\n", adcValue[PFC_I_IN_ADC]);
    PRINTF("PFC_VIN_ADC value: %d\r\n", adcValue[PFC_VIN_ADC]);
    PRINTF("PFC_I_OUT_ADC value: %d\r\n", adcValue[PFC_I_OUT_ADC]);
    PRINTF("PFC_VOUT_ADC value: %d\r\n", adcValue[PFC_VOUT_ADC]);
    PRINTF("PFC1_TEMP_ADC value: %d\r\n", adcValue[PFC1_TEMP_ADC]);
    PRINTF("PFC2_TEMP_ADC value: %d\r\n", adcValue[PFC2_TEMP_ADC]);
}

static void handle_WriteDac0_Command(char *ptr) {
    char *paramStart = strstr(ptr, ":");
    if (paramStart != NULL) {
        paramStart++;
        while (*paramStart == ' ' || *paramStart == '\t') paramStart++;
        dacValue = atoi(paramStart);
        if (dacValue > 4095) {
            PRINTF("Invalid, out of DAC range\r\n");
            return;
        }
        PRINTF("Write DAC0 value: %d\r\n", dacValue);
        DAC_SetData(DAC0, dacValue);
    }
}

static void handle_WriteDac1_Command(char *ptr) {
    char *paramStart = strstr(ptr, ":");
    if (paramStart != NULL) {
        paramStart++;
        while (*paramStart == ' ' || *paramStart == '\t') paramStart++;
        dacValue = atoi(paramStart);
        if (dacValue > 4095) {
            PRINTF("Invalid, out of DAC range\r\n");
            return;
        }
        PRINTF("Write DAC1 value: %d\r\n", dacValue);
        DAC_SetData(DAC1, dacValue);
    }
}
