/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "main.h"
#include "fsl_device_registers.h"
#include "fsl_lpadc.h"
#include "sensor.h"

const uint32_t g_LpadcResultShift = 3U;

void readSensor(void);


void readSensor(void)
{
    /* Trigger AMB_TEMP_ADC and VOUT_DC2_ADC convert then read ADC value. */
    LPADC_SetConvTriggerConfig(ADC0, 0U, &triggerConfigStruct[ADC0_A6B6]); /* Configurate the trigger0. */
    LPADC_DoSoftwareTrigger(ADC0, 1U); /* 1U is trigger0 mask. */
    while (!LPADC_GetConvResult(ADC0, &resultStruct[ADC0_A6B6], 1U))
    {

    }
    // PRINTF("ADC0_B6 value: %d\r\n", ((resultStruct[ADC0_A6B6].convValue) >> g_LpadcResultShift));
    adcValue[VOUT_DC2_ADC] = (resultStruct[ADC0_A6B6].convValue >> g_LpadcResultShift);

    while (!LPADC_GetConvResult(ADC0, &resultStruct[ADC0_A6B6], 0U))
    {

    }
    // PRINTF("ADC0_A6 value: %d\r\n", ((resultStruct[ADC0_A6B6].convValue) >> g_LpadcResultShift));
    adcValue[AMB_TEMP_ADC] = (resultStruct[ADC0_A6B6].convValue >> g_LpadcResultShift);

    /* Trigger OUT_DC2_ADC convert then read ADC value. */
    LPADC_SetConvTriggerConfig(ADC0, 0U, &triggerConfigStruct[ADC0_B1]); /* Configurate the trigger0. */
    LPADC_DoSoftwareTrigger(ADC0, 1U); /* 2U is trigger1 mask. */
	while (!LPADC_GetConvResult(ADC0, &resultStruct[ADC0_B1], 0U))
	{

	}
	// PRINTF("ADC0_B1 value: %d\r\n", ((resultStruct[ADC0_B1].convValue) >> g_LpadcResultShift));
    adcValue[IOUT_DC2_ADC] = (resultStruct[ADC0_B1].convValue >> g_LpadcResultShift);

    /* Trigger IOUT_DC1_ADC convert then read ADC value. */
    LPADC_SetConvTriggerConfig(ADC1, 0U, &triggerConfigStruct[ADC1_A6]); /* Configurate the trigger0. */
	LPADC_DoSoftwareTrigger(ADC1, 1U); /* 2U is trigger1 mask. */
	while (!LPADC_GetConvResult(ADC1, &resultStruct[ADC1_A6], 0U))
	{

	}
	// PRINTF("ADC1_A6 value: %d\r\n", ((resultStruct[ADC1_A6].convValue) >> g_LpadcResultShift));
    adcValue[IOUT_DC1_ADC] = (resultStruct[ADC1_A6].convValue >> g_LpadcResultShift);

    /* Trigger VOUT_DC1_ADC convert then read ADC value. */
    LPADC_SetConvTriggerConfig(ADC1, 0U, &triggerConfigStruct[ADC1_B5]); /* Configurate the trigger0. */
	LPADC_DoSoftwareTrigger(ADC1, 1U); /* 2U is trigger1 mask. */
	while (!LPADC_GetConvResult(ADC1, &resultStruct[ADC1_B5], 0U))
	{

	}
	// PRINTF("ADC1_B5 value: %d\r\n", ((resultStruct[ADC1_B5].convValue) >> g_LpadcResultShift));
    adcValue[VOUT_DC1_ADC] = (resultStruct[ADC1_B5].convValue >> g_LpadcResultShift);

    /* Trigger DCDC1_TEMP_ADC convert then read ADC value. */
    LPADC_SetConvTriggerConfig(ADC1, 0U, &triggerConfigStruct[ADC1_B8]); /* Configurate the trigger0. */
	LPADC_DoSoftwareTrigger(ADC1, 1U); /* 2U is trigger1 mask. */
	while (!LPADC_GetConvResult(ADC1, &resultStruct[ADC1_B8], 0U))
	{

	}
	// PRINTF("ADC1_B8 value: %d\r\n", ((resultStruct[ADC1_B8].convValue) >> g_LpadcResultShift));
    adcValue[DCDC1_TEMP_ADC] = (resultStruct[ADC1_B8].convValue >> g_LpadcResultShift);

    /* Trigger DCDC2_TEMP_ADC convert then read ADC value. */
    LPADC_SetConvTriggerConfig(ADC1, 0U, &triggerConfigStruct[ADC1_B9]); /* Configurate the trigger0. */
	LPADC_DoSoftwareTrigger(ADC1, 1U); /* 2U is trigger1 mask. */
	while (!LPADC_GetConvResult(ADC1, &resultStruct[ADC1_B9], 0U))
	{

	}
	// PRINTF("ADC1_B9 value: %d\r\n", ((resultStruct[ADC1_B9].convValue) >> g_LpadcResultShift));
    adcValue[DCDC2_TEMP_ADC] = (resultStruct[ADC1_B9].convValue >> g_LpadcResultShift);

    /* Trigger DCI1_ADC convert then read ADC value. */
    LPADC_SetConvTriggerConfig(ADC1, 0U, &triggerConfigStruct[ADC1_B10]); /* Configurate the trigger0. */
	LPADC_DoSoftwareTrigger(ADC1, 1U); /* 2U is trigger1 mask. */
	while (!LPADC_GetConvResult(ADC1, &resultStruct[ADC1_B10], 0U))
	{

	}
	// PRINTF("ADC1_B10 value: %d\r\n", ((resultStruct[ADC1_B10].convValue) >> g_LpadcResultShift));
    adcValue[DCI1_ADC] = (resultStruct[ADC1_B10].convValue >> g_LpadcResultShift);

    /* Trigger DCI2_ADC convert then read ADC value. */
    LPADC_SetConvTriggerConfig(ADC1, 0U, &triggerConfigStruct[ADC1_B11]); /* Configurate the trigger0. */
	LPADC_DoSoftwareTrigger(ADC1, 1U); /* 2U is trigger1 mask. */
	while (!LPADC_GetConvResult(ADC1, &resultStruct[ADC1_B11], 0U))
	{

	}
	// PRINTF("ADC1_B11 value: %d\r\n", ((resultStruct[ADC1_B11].convValue) >> g_LpadcResultShift));
    adcValue[DCI2_ADC] = (resultStruct[ADC1_B11].convValue >> g_LpadcResultShift);


    /* Trigger PFC_I_IN_ADC and PFC_VIN_ADC convert then read ADC value. */
    LPADC_SetConvTriggerConfig(ADC0, 0U, &triggerConfigStruct[ADC0_A0B0]); /* Configurate the trigger0. */
    LPADC_DoSoftwareTrigger(ADC0, 1U); /* 1U is trigger0 mask. */
    while (!LPADC_GetConvResult(ADC0, &resultStruct[ADC0_A0B0], 1U))
    {

    }
   
    adcValue[PFC_VIN_ADC] = (resultStruct[ADC0_A0B0].convValue >> g_LpadcResultShift);

    while (!LPADC_GetConvResult(ADC0, &resultStruct[ADC0_A0B0], 0U))
    {

    }
   
    adcValue[PFC_I_IN_ADC] = (resultStruct[ADC0_A0B0].convValue >> g_LpadcResultShift);


    /* Trigger PFC_I_OUT_ADC and PFC_VOUT_ADC convert then read ADC value. */
    LPADC_SetConvTriggerConfig(ADC1, 0U, &triggerConfigStruct[ADC1_A0B0]); /* Configurate the trigger0. */
    LPADC_DoSoftwareTrigger(ADC1, 1U); /* 1U is trigger0 mask. */
    while (!LPADC_GetConvResult(ADC1, &resultStruct[ADC1_A0B0], 1U))
    {

    }
   
    adcValue[PFC_VOUT_ADC] = (resultStruct[ADC1_A0B0].convValue >> g_LpadcResultShift);

    while (!LPADC_GetConvResult(ADC1, &resultStruct[ADC1_A0B0], 0U))
    {

    }
   
    adcValue[PFC_I_OUT_ADC] = (resultStruct[ADC1_A0B0].convValue >> g_LpadcResultShift);

    /* Trigger PFC1_TEMP_ADC convert then read ADC value. */
    LPADC_SetConvTriggerConfig(ADC0, 0U, &triggerConfigStruct[ADC0_A3]); /* Configurate the trigger0. */
	LPADC_DoSoftwareTrigger(ADC0, 1U); /* 2U is trigger1 mask. */
	while (!LPADC_GetConvResult(ADC0, &resultStruct[ADC0_A3], 0U))
	{

	}
	
    adcValue[PFC1_TEMP_ADC] = (resultStruct[ADC0_A3].convValue >> g_LpadcResultShift);

    /* Trigger PFC2_TEMP_ADC convert then read ADC value. */
    LPADC_SetConvTriggerConfig(ADC0, 0U, &triggerConfigStruct[ADC0_A7]); /* Configurate the trigger0. */
	LPADC_DoSoftwareTrigger(ADC0, 1U); /* 2U is trigger1 mask. */
	while (!LPADC_GetConvResult(ADC0, &resultStruct[ADC0_A7], 0U))
	{

	}
	
    adcValue[PFC2_TEMP_ADC] = (resultStruct[ADC0_A7].convValue >> g_LpadcResultShift);
}
