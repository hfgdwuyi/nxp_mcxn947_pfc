/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// command.h
#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stdbool.h>

#include "fsl_lpuart.h"
#include "fsl_lpadc.h"
#include "fsl_flexcan.h"

#define BUFFER_SIZE 256
/* Get frequency of flexcan clock */
#define RX_MESSAGE_BUFFER_NUM      (0)
#define TX_MESSAGE_BUFFER_NUM      (1)
#define DLC 8


extern uint16_t adcValue[9];

typedef enum _adc_value_name
{
	VOUT_DC1_ADC,
	AMB_TEMP_ADC,
	VOUT_DC2_ADC,
	IOUT_DC2_ADC,
	IOUT_DC1_ADC,
	DCDC1_TEMP_ADC,
	DCDC2_TEMP_ADC,
	DCI1_ADC,
    DCI2_ADC,
	PFC_VIN_ADC,
	PFC_I_IN_ADC,
	PFC_I_OUT_ADC,
	PFC_VOUT_ADC,
	PFC1_TEMP_ADC,
	PFC2_TEMP_ADC,
} adc_value_name_t;

typedef enum _adc_channel_name
{
	ADC0_A0B0,
	ADC0_A3,
	ADC0_A7,
	ADC0_B1,
	ADC0_A6B6,
	ADC1_A0B0,
	ADC1_B5,
	ADC1_A6,
	ADC1_B8,
	ADC1_B9,
	ADC1_B10,
	ADC1_B11,
} adc_channel_name_t;


typedef struct {
    uint8_t buffer[BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t count;
} circular_buffer_t;

extern circular_buffer_t g_rxBuffer;
extern circular_buffer_t g_txBuffer;
extern lpadc_conv_trigger_config_t triggerConfigStruct[8];
extern lpadc_conv_command_config_t commandConfigStruct[8];
extern lpadc_conv_result_t resultStruct[8];

extern flexcan_handle_t flexcanHandle;
extern flexcan_frame_t frame,rxFrame;
extern flexcan_mb_transfer_t txXfer, rxXfer;
#endif


