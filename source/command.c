/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "command.h"
#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "fsl_lpuart.h"
#include "command.h"
#include "board.h"
#include "main.h"
#include "fsl_device_registers.h"
#include "fsl_dac.h"
#include "fsl_debug_console.h"


#define CMD_SS_RLY_EN            "ss_rly_enable"
#define CMD_SS_RLY_DIS         "ss_rly_disable"
#define CMD_PFC1_EN            "pfc1_enable"
#define CMD_PFC1_DIS         "pfc1_disable"
#define CMD_PFC2_EN            "pfc2_enable"
#define CMD_PFC2_DIS         "pfc2_disable"
#define CMD_PFC1_FLT_DETECT           "pfc1_flt_detect"
#define CMD_PFC2_FLT_DETECT            "pfc2_flt_detect"
#define CMD_SYS_CFG0_DETECT           "sys_cfg0_detect"
#define CMD_SYS_CFG1_DETECT            "sys_cfg1_detect"
#define CMD_SYS_CFG2_DETECT            "sys_cfg2_detect"
#define CMD_FAN1_FB_DETECT           "fan1_fb_detect"
#define CMD_FAN2_FB_DETECT            "fan2_fb_detect"
#define CMD_FAN3_FB_DETECT            "fan3_fb_detect"
#define CMD_DCDC1_EN            "dcdc1_enable"
#define CMD_DCDC1_DIS         "dcdc1_disable"
#define CMD_DCDC2_EN            "dcdc2_enable"
#define CMD_DCDC2_DIS         "dcdc2_disable"
#define CMD_READ_ADC            "get_adc"
#define CMD_WRITE_DAC0            "set_dac0"
#define CMD_WRITE_DAC1            "set_dac1"
#define CMD_FAN1_EN            "fan1_enable"
#define CMD_FAN1_DIS         "fan1_disable"
#define CMD_FAN2_EN            "fan2_enable"
#define CMD_FAN2_DIS         "fan2_disable"
#define CMD_FAN3_EN            "fan3_enable"
#define CMD_FAN3_DIS         "fan3_disable"
#define RX_BUFFER_SIZE          256

void processReceivedCommand(const char *data);
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
static void handle_FAN1_EN_Command(char *ptr);
static void handle_FAN1_DIS_Command(char *ptr);
static void handle_FAN2_EN_Command(char *ptr);
static void handle_FAN2_DIS_Command(char *ptr);
static void handle_FAN3_EN_Command(char *ptr);
static void handle_FAN3_DIS_Command(char *ptr);


typedef struct {
    const char *name;
    void (*handler)(char *ptr);
} Command;

Command commands[] = {
    {CMD_SS_RLY_EN, handle_SS_RLY_EN_Command},
    {CMD_SS_RLY_DIS, handle_SS_PLY_DIS_Command},
    {CMD_PFC1_EN, handle_PFC1_EN_Command},
    {CMD_PFC1_DIS, handle_PFC1_DIS_Command},
    {CMD_PFC2_EN, handle_PFC2_EN_Command},
    {CMD_PFC2_DIS, handle_PFC2_DIS_Command},
    {CMD_PFC1_FLT_DETECT, handle_PFC1_FLT_DETECT_Command},
    {CMD_PFC2_FLT_DETECT, handle_PFC2_FLT_DETECT_Command},
    {CMD_SYS_CFG0_DETECT, handle_SYS_CFG0_DETECT_Command},
    {CMD_SYS_CFG1_DETECT, handle_SYS_CFG1_DETECT_Command},
    {CMD_SYS_CFG2_DETECT, handle_SYS_CFG2_DETECT_Command},
    {CMD_FAN1_FB_DETECT, handle_FAN1_FB_DETECT_Command},
    {CMD_FAN2_FB_DETECT, handle_FAN2_FB_DETECT_Command},
    {CMD_FAN3_FB_DETECT, handle_FAN3_FB_DETECT_Command},
    {CMD_DCDC1_EN, handle_DCDC1_EN_Command},
    {CMD_DCDC1_DIS, handle_DCDC1_DIS_Command},
    {CMD_DCDC2_EN, handle_DCDC2_EN_Command},
    {CMD_DCDC2_DIS, handle_DCDC2_DIS_Command},
    {CMD_READ_ADC, handle_ReadAdc_Command},
    {CMD_WRITE_DAC0, handle_WriteDac0_Command},
    {CMD_WRITE_DAC1, handle_WriteDac1_Command},
    {CMD_FAN1_EN, handle_FAN1_EN_Command},
    {CMD_FAN1_DIS, handle_FAN1_DIS_Command},
    {CMD_FAN2_EN, handle_FAN2_EN_Command},
    {CMD_FAN2_DIS, handle_FAN2_DIS_Command},
    {CMD_FAN3_EN, handle_FAN3_EN_Command},
    {CMD_FAN3_DIS, handle_FAN3_DIS_Command}
};

#define COMMAND_COUNT (sizeof(commands) / sizeof(commands[0]))
uint32_t dacValue;


void processReceivedCommand(const char *data) {
    char temp[RX_BUFFER_SIZE] = {0};
    bool commandMatched = false;

    // 安全复制数据到临时缓冲区，并确保字符串以'\0'结尾
    strncpy(temp, data, sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0'; // 确保字符串终止

    // 正确去除字符串末尾的换行符和回车符
    char *ptr = temp;
    size_t len = strlen(ptr);
    while (len > 0 && (ptr[len-1] == '\r' || ptr[len-1] == '\n')) {
        ptr[len-1] = '\0';
        len--;
    }

    // 调试输出：显示实际接收到的处理后的字符串
    // PRINTF("Processing command: [%s]\n", ptr);

    // 查找匹配的命令
    for (int i = 0; i < COMMAND_COUNT; i++) {
        if (strstr(ptr, commands[i].name) != NULL) {
            commands[i].handler(ptr);
            commandMatched = true;
            PRINTF("Command matched: %s\n", commands[i].name);
            break;
        }
    }

    // 无论命令是否匹配，处理完成后清空缓冲区
    g_rxBuffer.head = 0;
    g_rxBuffer.tail = 0;
    g_rxBuffer.count = 0;
	memset(g_rxBuffer.buffer, 0, sizeof(g_rxBuffer.buffer));

}




static void handle_SS_RLY_EN_Command(char *ptr) {
    PRINTF("You enable SS-RLY\r\n"); 
    GPIO_PortSet(SS_RLY_EN_GPIO, 1u << SS_RLY_EN_GPIO_PIN);
}

static void handle_SS_PLY_DIS_Command(char *ptr) {
    PRINTF("You disable SS-RLY\r\n"); 
    GPIO_PortClear(SS_RLY_EN_GPIO, 1u << SS_RLY_EN_GPIO_PIN);
}

static void handle_PFC1_FLT_DETECT_Command(char *ptr) {
    uint32_t signalActive = GPIO_PinRead(PFC1_FLT_GPIO, PFC1_FLT_GPIO_PIN);
    if(signalActive){
        PRINTF("PFC1_FLT is high level\r\n"); 
    }else{
        PRINTF("PFC1_FLT is low level\r\n"); 
    }
}

static void handle_PFC2_FLT_DETECT_Command(char *ptr) {
    uint32_t signalActive = GPIO_PinRead(PFC2_FLT_GPIO, PFC2_FLT_GPIO_PIN);
    if(signalActive){
        PRINTF("PFC2_FLT is high level\r\n"); 
    }else{
        PRINTF("PFC2_FLT is low level\r\n"); 
    }
}

static void handle_SYS_CFG0_DETECT_Command(char *ptr) {
    uint32_t signalActive = GPIO_PinRead(SYS_CFG0_GPIO, SYS_CFG0_GPIO_PIN);
    if(signalActive){
        PRINTF("SYS_CFG0 is high level\r\n"); 
    }else{
        PRINTF("SYS_CFG0 is low level\r\n"); 
    }
}

static void handle_SYS_CFG1_DETECT_Command(char *ptr) {
    uint32_t signalActive = GPIO_PinRead(SYS_CFG1_GPIO, SYS_CFG1_GPIO_PIN);
    if(signalActive){
        PRINTF("SYS_CFG1 is high level\r\n"); 
    }else{
        PRINTF("SYS_CFG1 is low level\r\n"); 
    }
}

static void handle_SYS_CFG2_DETECT_Command(char *ptr) {
    uint32_t signalActive = GPIO_PinRead(SYS_CFG2_GPIO, SYS_CFG2_GPIO_PIN);
    if(signalActive){
        PRINTF("SYS_CFG2 is high level\r\n"); 
    }else{
        PRINTF("SYS_CFG2 is low level\r\n"); 
    }
}

static void handle_FAN1_FB_DETECT_Command(char *ptr) {
    uint32_t signalActive = GPIO_PinRead(PWM1_FB_GPIO, PWM1_FB_GPIO_PIN);
    if(signalActive){
        PRINTF("PWM1_FB is high level\r\n"); 
    }else{
        PRINTF("PWM1_FB is low level\r\n"); 
    }
}

static void handle_FAN2_FB_DETECT_Command(char *ptr) {
    uint32_t signalActive = GPIO_PinRead(PWM2_FB_GPIO, PWM2_FB_GPIO_PIN);
    if(signalActive){
        PRINTF("PWM2_FB is high level\r\n"); 
    }else{
        PRINTF("PWM2_FB is low level\r\n"); 
    }
}

static void handle_FAN3_FB_DETECT_Command(char *ptr) {
    uint32_t signalActive = GPIO_PinRead(PWM3_FB_GPIO, PWM3_FB_GPIO_PIN);
    if(signalActive){
        PRINTF("PWM3_FB is high level\r\n"); 
    }else{
        PRINTF("PWM3_FB is low level\r\n"); 
    }
}



static void handle_PFC1_EN_Command(char *ptr) {
    PRINTF("You enable PFC1 FLT\r\n"); 
    GPIO_PortSet(PFC1_EN_GPIO, 1u << PFC1_EN_GPIO_PIN);
}

static void handle_PFC1_DIS_Command(char *ptr) {
    PRINTF("You disable PFC1 FLT\r\n"); 
    GPIO_PortClear(PFC1_EN_GPIO, 1u << PFC1_EN_GPIO_PIN);
}

static void handle_PFC2_EN_Command(char *ptr) {
    PRINTF("You enable PFC2 FLT\r\n"); 
    GPIO_PortSet(PFC2_EN_GPIO, 1u << PFC2_EN_GPIO_PIN);
}

static void handle_PFC2_DIS_Command(char *ptr) {
    PRINTF("You disable PFC2 FLT\r\n"); 
    GPIO_PortClear(PFC2_EN_GPIO, 1u << PFC2_EN_GPIO_PIN);
}


static void handle_DCDC1_EN_Command(char *ptr) {
    PRINTF("You enable DCDC1\r\n"); 
    GPIO_PortSet(DCDC1_EN_GPIO, 1u << DCDC1_EN_GPIO_PIN);
}

static void handle_DCDC1_DIS_Command(char *ptr) {
    PRINTF("You disable DCDC1\r\n"); 
    GPIO_PortClear(DCDC1_EN_GPIO, 1u << DCDC1_EN_GPIO_PIN);
}

static void handle_DCDC2_EN_Command(char *ptr) {
    PRINTF("You enable DCDC2\r\n"); 
    GPIO_PortSet(DCDC2_EN_GPIO, 1u << DCDC2_EN_GPIO_PIN);
}

static void handle_DCDC2_DIS_Command(char *ptr) {
    PRINTF("You disable DCDC2\r\n"); 
    GPIO_PortClear(DCDC2_EN_GPIO, 1u << DCDC2_EN_GPIO_PIN);
}


static void handle_FAN1_EN_Command(char *ptr) {
    PRINTF("You enable FAN1\r\n"); 
    GPIO_PortSet(PWM1_GPIO, 1u << PWM1_GPIO_PIN);
}

static void handle_FAN1_DIS_Command(char *ptr) {
    PRINTF("You disable FAN1\r\n"); 
    GPIO_PortClear(PWM1_GPIO, 1u << PWM1_GPIO_PIN);
}


static void handle_FAN2_EN_Command(char *ptr) {
    PRINTF("You enable FAN2\r\n"); 
    GPIO_PortSet(PWM2_GPIO, 1u << PWM2_GPIO_PIN);
}

static void handle_FAN2_DIS_Command(char *ptr) {
    PRINTF("You disable FAN2\r\n"); 
    GPIO_PortClear(PWM2_GPIO, 1u << PWM2_GPIO_PIN);
}

static void handle_FAN3_EN_Command(char *ptr) {
    PRINTF("You enable FAN3\r\n"); 
    GPIO_PortSet(PWM3_GPIO, 1u << PWM3_GPIO_PIN);
}

static void handle_FAN3_DIS_Command(char *ptr) {
    PRINTF("You disable FAN3\r\n"); 
    GPIO_PortClear(PWM3_GPIO, 1u << PWM3_GPIO_PIN);
}



static void handle_ReadAdc_Command(char *ptr){
    PRINTF("VOUT_DC2_ADC value: %d\r\n", (adcValue[VOUT_DC2_ADC]));
    PRINTF("AMB_TEMP_ADC value: %d\r\n", (adcValue[AMB_TEMP_ADC]));
    PRINTF("IOUT_DC2_ADC value: %d\r\n", (adcValue[IOUT_DC2_ADC]));
    PRINTF("VOUT_DC1_ADC value: %d\r\n", (adcValue[VOUT_DC1_ADC]));
    PRINTF("IOUT_DC1_ADC value: %d\r\n", (adcValue[IOUT_DC1_ADC]));
    PRINTF("DCDC1_TEMP_ADC value: %d\r\n", (adcValue[DCDC1_TEMP_ADC]));
    PRINTF("DCDC2_TEMP_ADC value: %d\r\n", (adcValue[DCDC2_TEMP_ADC]));
    PRINTF("DCI1_ADC value: %d\r\n", (adcValue[DCI1_ADC]));
    PRINTF("DCI2_ADC value: %d\r\n", (adcValue[DCI2_ADC]));

    PRINTF("PFC_I_IN_ADC value: %d\r\n", (adcValue[PFC_I_IN_ADC]));
    PRINTF("PFC_VIN_ADC value: %d\r\n", (adcValue[PFC_VIN_ADC]));
    PRINTF("PFC_I_OUT_ADC value: %d\r\n", (adcValue[PFC_I_OUT_ADC]));
    PRINTF("PFC_VOUT_ADC value: %d\r\n", (adcValue[PFC_VOUT_ADC]));
    PRINTF("PFC1_TEMP_ADC value: %d\r\n", (adcValue[PFC1_TEMP_ADC]));
    PRINTF("PFC2_TEMP_ADC value: %d\r\n", (adcValue[PFC2_TEMP_ADC]));
}

void handle_WriteDac0_Command(char *ptr){
    char *paramStart = strstr(ptr, ":"); // 查找冒号位置
    if (paramStart != NULL) {
        paramStart++; // 跳过冒号
        // 跳过空格
        while (*paramStart == ' ' || *paramStart == '\t') {
            paramStart++;
        }
        // 转换参数为整数
        dacValue = atoi(paramStart);
        if(dacValue > 4095)
        {
            PRINTF("Invalid, out of dac range\n");
            return;
        }
        PRINTF("Write DAC0 value: %d\n", dacValue);
        DAC_SetData(DAC0, dacValue);
    }
}

void handle_WriteDac1_Command(char *ptr){
    char *paramStart = strstr(ptr, ":"); // 查找冒号位置
    if (paramStart != NULL) {
        paramStart++; // 跳过冒号
        // 跳过空格
        while (*paramStart == ' ' || *paramStart == '\t') {
            paramStart++;
        }
        // 转换参数为整数
        dacValue = atoi(paramStart);
        if(dacValue > 4095)
        {
            PRINTF("Invalid, out of dac range\n");
            return;
        }
        PRINTF("Write DAC1 value: %d\n", dacValue);
        DAC_SetData(DAC1, dacValue);
    }
}
