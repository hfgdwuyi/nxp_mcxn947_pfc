/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// command.h
#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "fsl_lpadc.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*Default display refresh period in milliseconds. LVG will redraw changed areas with this period time*/
#define LV_SENSOR_DEF_REFR_PERIOD 1

extern uint16_t g_lq12GridVoltRms;
extern float fGridVoltRms;
extern float fGridVoltAdc;
extern volatile bool g_LpadcConversionCompletedFlag;
extern lpadc_conv_result_t g_LpadcResultConfigStruct;

typedef struct {
    uint8_t isPositivalHalfFlag;
    uint16_t adcValue;
    float sum;
    uint8_t sumCnts;
    float meanSqure;
    float rootMeanSqure;
    float measure;
    float real;
} rms_message_t;

typedef struct {
    uint16_t adcValue;
    float real;
} constant_message_t;

typedef struct {
    bool powerDataUpdated;
} sensor_status_t;

extern rms_message_t Vrms,Irms;
extern constant_message_t Vout,Iout;
extern sensor_status_t sensor_status;

extern void readSensor(void);

#endif



