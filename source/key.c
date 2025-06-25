/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "main.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "key.h"
#include "sensor.h"
#include "lcd.h"

#include "FreeRTOS.h"
#include "queue.h"

// 定义ADC范围结构体
typedef struct {
    uint16_t minValue;        // 下限值
    uint16_t maxValue;        // 上限值
    void (*action)(void);     // 范围对应的动作函数指针
} ADCRange_t;


key_status_t key_status;



void keyScan(void);
void actionForSW1(void);
void actionForSW2(void);
void actionForSW3(void);
void actionForSW4(void);
void actionForSW5(void);

const ADCRange_t adcRanges[6] = {
    {0, 100,       actionForSW1},  // 通道0范围及动作
    {500, 700,     actionForSW2},  // 通道1范围及动作
    {1200, 1400,   actionForSW3},  // 通道2范围及动作
    {1900, 2100,   actionForSW4},  // 通道3范围及动作
    {2800, 3000,   actionForSW5},   // 通道4范围及动作
	{3000, 4095,   NULL}
};

void keyScan(void){
	{
		uint16_t value = adcValue[ANALOG_KEYBOARD_ADC];
		for (int i = 0; i < 5; i++) {
        	if (value >= adcRanges[i].minValue && value <= adcRanges[i].maxValue) {
            	if (adcRanges[i].action != NULL && key_status.dataUpdated) {
                	adcRanges[i].action();
					key_status.dataUpdated = false;
            	}
				else{
					key_status.dataUpdated = true;
				}
            	break;
        	}
    	}
	}
}

void actionForSW1(void){
	key_message_t msg = {MSG_INCREASE_PAGE, false};
    xQueueSend(xKeyMessageQueue, &msg, 0);
}

void actionForSW2(void){
	key_message_t msg = {MSG_DECREASE_PAGE, false};
    xQueueSend(xKeyMessageQueue, &msg, 0);
}

void actionForSW3(void){
	key_message_t msg = {MSG_INCREASE_VALUE, false};
    xQueueSend(xKeyMessageQueue, &msg, 0);
}

void actionForSW4(void){
	key_message_t msg = {MSG_DECREASE_VALUE, false};
    xQueueSend(xKeyMessageQueue, &msg, 0);
}

void actionForSW5(void){
	key_message_t msg = {MSG_CONFIRM, true};
    xQueueSend(xKeyMessageQueue, &msg, 0);
}