/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "main.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_lpadc.h"
#include "sensor.h"
#include "fsl_dac.h"
#include "math.h"
#include "board.h"

const uint32_t g_LpadcResultShift = 3U;
volatile bool g_LpadcConversionCompletedFlag = false;
lpadc_conv_result_t g_LpadcResultConfigStruct;

uint8_t g_u8IsPositivalHalfFlg = 0;
float g_lq2GridVoltRmsSum = 0.0;
float g_lq12GridVolt = 0.0;
float g_i32SumReciCnts = 0.0;
uint16_t g_lq12GridVoltRms = 0;
float fGridVoltRms = 0.0;
float fGridVoltAdc = 0.0;
uint8_t g_u16RMSSumCnts = 0;
uint16_t adcVal;

long iq12Temp = 0;
long iq12TempAccum = 0;
long iq12TempAvg = 0;

rms_message_t Vrms,Irms;
constant_message_t Vout,Iout;
sensor_status_t sensor_status;

void readSensor(void);
void caclulateVrms(void);
void caclulateIrms(void);
void caclulateVout(void);
void caclulateAmbientTemperature(void);


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

   /* Trigger Analogue keyboard convert then read ADC value. */
    LPADC_SetConvTriggerConfig(ADC1, 0U, &triggerConfigStruct[ADC1_B6]); /* Configurate the trigger0. */
  	LPADC_DoSoftwareTrigger(ADC1, 1U); /* 2U is trigger1 mask. */
  	while (!LPADC_GetConvResult(ADC1, &resultStruct[ADC1_B6], 0U))
  	{
  	}

    adcValue[ANALOG_KEYBOARD_ADC] = (resultStruct[ADC1_B6].convValue >> g_LpadcResultShift);

    caclulateVrms();
    caclulateIrms();

    sensor_status.powerDataUpdated = true;
}

void caclulateVrms(void){
	// adcValue[PFC_VIN_ADC] = adcValue[PFC_VIN_ADC] - 600;
	// if(adcValue[PFC_VIN_ADC] >= 2)
	// {

	// 	g_u8IsPositivalHalfFlg = 1;
	// }
	// else
	// {
	// 	g_u8IsPositivalHalfFlg = 0;
	// }

	// g_lq12GridVolt = adcValue[PFC_VIN_ADC];
	// if(g_u8IsPositivalHalfFlg == 1)
	// {
	// 	g_lq2GridVoltRmsSum += g_lq12GridVolt * g_lq12GridVolt;
	// 	g_u16RMSSumCnts++;
	// }
	// else
	// {
	// 	if(g_u16RMSSumCnts > 0)
	// 	{
	// 		g_i32SumReciCnts = g_lq2GridVoltRmsSum / g_u16RMSSumCnts;
	// 	}
	// 	g_u16RMSSumCnts = 0;
	// 	g_lq2GridVoltRmsSum = 0;
	// }

	// g_lq12GridVoltRms = sqrtf(g_i32SumReciCnts);
	//  iq12Temp = (long)g_lq12GridVoltRms;
	//  iq12TempAccum -= (iq12TempAvg);
	//  iq12TempAccum += iq12Temp;
	//  iq12TempAvg = iq12TempAccum;
	//  g_lq12GridVoltRms = iq12TempAvg;

	// fGridVoltAdc = (float)g_lq12GridVoltRms / 4096 * 3.3;
	// fGridVoltRms = (fGridVoltAdc) * 10000 / 19;




    Vrms.adcValue = adcValue[PFC_VIN_ADC];
    if(Vrms.adcValue >= 5)
	{
		Vrms.isPositivalHalfFlag = true;
	}
	else
	{
		Vrms.isPositivalHalfFlag = false;
	}

	if(Vrms.isPositivalHalfFlag)
	{
		Vrms.sum += Vrms.adcValue * Vrms.adcValue;
		Vrms.sumCnts++;
	}
	else
	{
		if(Vrms.sumCnts > 0)
		{
			Vrms.meanSqure = Vrms.sum / Vrms.sumCnts;
		}
		Vrms.sumCnts = 0;
		Vrms.sum = 0;
	}

	Vrms.rootMeanSqure = sqrtf(Vrms.meanSqure);
    iq12Temp = (long)Vrms.rootMeanSqure;
    iq12TempAccum -= (iq12TempAvg);
    iq12TempAccum += iq12Temp;
    iq12TempAvg = iq12TempAccum;
    Vrms.rootMeanSqure = iq12TempAvg;

    // #define FILTER_COEFFICIENT (0.1f)  // 原系数假设为0.5，现减小为0.1

    // // 滑动滤波实现
    // iq12Temp = (long)Vrms.rootMeanSqure;

    // // 使用指数滑动平均公式：new_avg = α*new_value + (1-α)*old_avg
    // iq12TempAvg = (long)((FILTER_COEFFICIENT * iq12Temp) + 
    //                     ((1.0f - FILTER_COEFFICIENT) * iq12TempAvg));

    // Vrms.rootMeanSqure = iq12TempAvg;


	Vrms.real = (float)Vrms.rootMeanSqure / 4096 * 3.3 * 10000 / 19;
}

void caclulateAmbientTemperature(void){

}

void caclulateIrms(void){
    Irms.adcValue = adcValue[PFC_I_IN_ADC];
    if(Irms.adcValue >= 5)
	{
		Irms.isPositivalHalfFlag = true;
	}
	else
	{
		Irms.isPositivalHalfFlag = false;
	}

	if(Irms.isPositivalHalfFlag)
	{
		Irms.sum += Irms.adcValue * Irms.adcValue;
		Irms.sumCnts++;
	}
	else
	{
		if(Irms.sumCnts > 0)
		{
			Irms.meanSqure = Irms.sum / Irms.sumCnts;
		}
		Irms.sumCnts = 0;
		Irms.sum = 0;
	}

	Irms.rootMeanSqure = sqrtf(Irms.meanSqure);
    iq12Temp = (long)Irms.rootMeanSqure;
    iq12TempAccum -= (iq12TempAvg);
    iq12TempAccum += iq12Temp;
    iq12TempAvg = iq12TempAccum;
    Irms.rootMeanSqure = iq12TempAvg;

	Irms.real = (float)Irms.rootMeanSqure / 4096 * 3.3 * 10000 / 19;
}

void caclulateVout(void){
    Vout.adcValue = adcValue[PFC_VOUT_ADC];
    Vout.real = 1.197 * (1800 + 1.197) / 8.2 * ((Vout.adcValue / 4096 * 3.3));
} 