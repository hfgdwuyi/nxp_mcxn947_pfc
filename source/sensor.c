/*
 * sensor.c - ADC sensor reading, calibration, RMS calculation
 *
 * Owns ADC0/ADC1 configuration, periodic sensor reading task,
 * and RMS calculations for voltage/current.
 */

#include "sensor.h"
#include "main.h"
#include "board.h"
#include "clock_config.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_lpadc.h"
#include "fsl_vref.h"
#include "fsl_spc.h"
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

const uint32_t g_LpadcResultShift = 3U;

/* ADC config structures */
lpadc_conv_trigger_config_t triggerConfigStruct[13];
lpadc_conv_command_config_t commandConfigStruct[13];
lpadc_conv_result_t resultStruct[13];

/* Shared ADC values */
uint16_t adcValue[16];

/* RMS calculation state */
static long iq12Temp = 0;
static long iq12TempAccum = 0;
static long iq12TempAvg = 0;

rms_message_t Vrms, Irms;
constant_message_t Vout, Iout;
sensor_status_t sensor_status;

/* Local config */
static vref_config_t vrefConfig;
static lpadc_config_t mLpadcConfigStruct;

static void caclulateVrms(void);
static void caclulateIrms(void);

/*================================================================
 * ADC Configuration
 *================================================================*/
void ADC_Configure(void)
{
    SPC_EnableActiveModeAnalogModules(SPC0, kSPC_controlVref);

    VREF_GetDefaultConfig(&vrefConfig);
    vrefConfig.bufferMode = kVREF_ModeBandgapOnly;
    VREF_Init(VREF0, &vrefConfig);

    LPADC_GetDefaultConfig(&mLpadcConfigStruct);
    mLpadcConfigStruct.enableAnalogPreliminary = true;
    mLpadcConfigStruct.referenceVoltageSource = LPADC_VREF_SOURCE;
    mLpadcConfigStruct.conversionAverageMode = kLPADC_ConversionAverage128;

    LPADC_Init(ADC0, &mLpadcConfigStruct);
    LPADC_DoOffsetCalibration(ADC0);
    LPADC_DoAutoCalibration(ADC0);

    LPADC_Init(ADC1, &mLpadcConfigStruct);
    LPADC_DoOffsetCalibration(ADC1);
    LPADC_DoAutoCalibration(ADC1);

    /* ADC0_A0B0 - PFC VIN + I_IN (dual single-end both sides) */
    LPADC_GetDefaultConvCommandConfig(&commandConfigStruct[ADC0_A0B0]);
    commandConfigStruct[ADC0_A0B0].channelNumber = 0U;
    commandConfigStruct[ADC0_A0B0].enableChannelB = true;
    commandConfigStruct[ADC0_A0B0].channelBNumber = 0;
    commandConfigStruct[ADC0_A0B0].sampleChannelMode = kLPADC_SampleChannelDualSingleEndBothSide;
    LPADC_SetConvCommandConfig(ADC0, 5, &commandConfigStruct[ADC0_A0B0]);
    LPADC_GetDefaultConvTriggerConfig(&triggerConfigStruct[ADC0_A0B0]);
    triggerConfigStruct[ADC0_A0B0].targetCommandId       = 5U;
    triggerConfigStruct[ADC0_A0B0].enableHardwareTrigger = false;
    triggerConfigStruct[ADC0_A0B0].channelBFIFOSelect = 1;
    triggerConfigStruct[ADC0_A0B0].channelAFIFOSelect = 0;

    /* ADC1_A0B0 - PFC VOUT + I_OUT */
    LPADC_GetDefaultConvCommandConfig(&commandConfigStruct[ADC1_A0B0]);
    commandConfigStruct[ADC1_A0B0].channelNumber = 0U;
    commandConfigStruct[ADC1_A0B0].enableChannelB = true;
    commandConfigStruct[ADC1_A0B0].channelBNumber = 0;
    commandConfigStruct[ADC1_A0B0].sampleChannelMode = kLPADC_SampleChannelDualSingleEndBothSide;
    LPADC_SetConvCommandConfig(ADC1, 6, &commandConfigStruct[ADC1_A0B0]);
    LPADC_GetDefaultConvTriggerConfig(&triggerConfigStruct[ADC1_A0B0]);
    triggerConfigStruct[ADC1_A0B0].targetCommandId       = 6U;
    triggerConfigStruct[ADC1_A0B0].enableHardwareTrigger = false;
    triggerConfigStruct[ADC1_A0B0].channelBFIFOSelect = 1;
    triggerConfigStruct[ADC1_A0B0].channelAFIFOSelect = 0;

    /* ADC0_A3 - PFC1_TEMP */
    LPADC_GetDefaultConvCommandConfig(&commandConfigStruct[ADC0_A3]);
    commandConfigStruct[ADC0_A3].channelNumber = 3;
    commandConfigStruct[ADC0_A3].sampleChannelMode = kLPADC_SampleChannelSingleEndSideA;
    LPADC_SetConvCommandConfig(ADC0, 11, &commandConfigStruct[ADC0_A3]);
    LPADC_GetDefaultConvTriggerConfig(&triggerConfigStruct[ADC0_A3]);
    triggerConfigStruct[ADC0_A3].targetCommandId       = 11U;
    triggerConfigStruct[ADC0_A3].enableHardwareTrigger = false;
    triggerConfigStruct[ADC0_A3].channelAFIFOSelect = 0;

    /* ADC0_A7 - PFC2_TEMP */
    LPADC_GetDefaultConvCommandConfig(&commandConfigStruct[ADC0_A7]);
    commandConfigStruct[ADC0_A7].channelNumber = 7;
    commandConfigStruct[ADC0_A7].sampleChannelMode = kLPADC_SampleChannelSingleEndSideA;
    LPADC_SetConvCommandConfig(ADC0, 12, &commandConfigStruct[ADC0_A7]);
    LPADC_GetDefaultConvTriggerConfig(&triggerConfigStruct[ADC0_A7]);
    triggerConfigStruct[ADC0_A7].targetCommandId       = 12U;
    triggerConfigStruct[ADC0_A7].enableHardwareTrigger = false;
    triggerConfigStruct[ADC0_A7].channelAFIFOSelect = 0;

    /* ADC0_A6B6 - VOUT_DC2 + AMB_TEMP */
    LPADC_GetDefaultConvCommandConfig(&commandConfigStruct[ADC0_A6B6]);
    commandConfigStruct[ADC0_A6B6].channelNumber = 6U;
    commandConfigStruct[ADC0_A6B6].enableChannelB = true;
    commandConfigStruct[ADC0_A6B6].channelBNumber = 6;
    commandConfigStruct[ADC0_A6B6].sampleChannelMode = kLPADC_SampleChannelDualSingleEndBothSide;
    LPADC_SetConvCommandConfig(ADC0, 1, &commandConfigStruct[ADC0_A6B6]);
    LPADC_GetDefaultConvTriggerConfig(&triggerConfigStruct[ADC0_A6B6]);
    triggerConfigStruct[ADC0_A6B6].targetCommandId       = 1U;
    triggerConfigStruct[ADC0_A6B6].enableHardwareTrigger = false;
    triggerConfigStruct[ADC0_A6B6].channelBFIFOSelect = 1;
    triggerConfigStruct[ADC0_A6B6].channelAFIFOSelect = 0;

    /* ADC0_B1 */
    LPADC_GetDefaultConvCommandConfig(&commandConfigStruct[ADC0_B1]);
    commandConfigStruct[ADC0_B1].enableChannelB = true;
    commandConfigStruct[ADC0_B1].channelBNumber = 1;
    commandConfigStruct[ADC0_B1].sampleChannelMode = kLPADC_SampleChannelSingleEndSideB;
    LPADC_SetConvCommandConfig(ADC0, 2, &commandConfigStruct[ADC0_B1]);
    LPADC_GetDefaultConvTriggerConfig(&triggerConfigStruct[ADC0_B1]);
    triggerConfigStruct[ADC0_B1].targetCommandId       = 2U;
    triggerConfigStruct[ADC0_B1].enableHardwareTrigger = false;
    triggerConfigStruct[ADC0_B1].channelBFIFOSelect = 0;
    LPADC_SetConvTriggerConfig(ADC0, 0U, &triggerConfigStruct[ADC0_B1]);

    /* ADC1_A6 - IOUT_DC1 */
    LPADC_GetDefaultConvCommandConfig(&commandConfigStruct[ADC1_A6]);
    commandConfigStruct[ADC1_A6].channelNumber = 6;
    commandConfigStruct[ADC1_A6].sampleChannelMode = kLPADC_SampleChannelSingleEndSideA;
    LPADC_SetConvCommandConfig(ADC1, 3, &commandConfigStruct[ADC1_A6]);
    LPADC_GetDefaultConvTriggerConfig(&triggerConfigStruct[ADC1_A6]);
    triggerConfigStruct[ADC1_A6].targetCommandId       = 3U;
    triggerConfigStruct[ADC1_A6].enableHardwareTrigger = false;
    triggerConfigStruct[ADC1_A6].channelAFIFOSelect = 0;

    /* ADC1_B5 - VOUT_DC1 */
    LPADC_GetDefaultConvCommandConfig(&commandConfigStruct[ADC1_B5]);
    commandConfigStruct[ADC1_B5].enableChannelB = true;
    commandConfigStruct[ADC1_B5].channelBNumber = 5;
    commandConfigStruct[ADC1_B5].sampleChannelMode = kLPADC_SampleChannelSingleEndSideB;
    LPADC_SetConvCommandConfig(ADC1, 4, &commandConfigStruct[ADC1_B5]);
    LPADC_GetDefaultConvTriggerConfig(&triggerConfigStruct[ADC1_B5]);
    triggerConfigStruct[ADC1_B5].targetCommandId       = 4U;
    triggerConfigStruct[ADC1_B5].enableHardwareTrigger = false;
    triggerConfigStruct[ADC1_B5].channelBFIFOSelect = 0;

    /* ADC1_B6 - ANALOG_KEYBOARD */
    LPADC_GetDefaultConvCommandConfig(&commandConfigStruct[ADC1_B6]);
    commandConfigStruct[ADC1_B6].enableChannelB = true;
    commandConfigStruct[ADC1_B6].channelBNumber = 6;
    commandConfigStruct[ADC1_B6].sampleChannelMode = kLPADC_SampleChannelSingleEndSideB;
    LPADC_SetConvCommandConfig(ADC1, 13, &commandConfigStruct[ADC1_B6]);
    LPADC_GetDefaultConvTriggerConfig(&triggerConfigStruct[ADC1_B6]);
    triggerConfigStruct[ADC1_B6].targetCommandId       = 13U;
    triggerConfigStruct[ADC1_B6].enableHardwareTrigger = false;
    triggerConfigStruct[ADC1_B6].channelBFIFOSelect = 0;

    /* ADC1_B8 - DCDC1_TEMP */
    LPADC_GetDefaultConvCommandConfig(&commandConfigStruct[ADC1_B8]);
    commandConfigStruct[ADC1_B8].enableChannelB = true;
    commandConfigStruct[ADC1_B8].channelBNumber = 8;
    commandConfigStruct[ADC1_B8].sampleChannelMode = kLPADC_SampleChannelSingleEndSideB;
    LPADC_SetConvCommandConfig(ADC1, 7, &commandConfigStruct[ADC1_B8]);
    LPADC_GetDefaultConvTriggerConfig(&triggerConfigStruct[ADC1_B8]);
    triggerConfigStruct[ADC1_B8].targetCommandId       = 7U;
    triggerConfigStruct[ADC1_B8].enableHardwareTrigger = false;
    triggerConfigStruct[ADC1_B8].channelBFIFOSelect = 0;

    /* ADC1_B9 - DCDC2_TEMP */
    LPADC_GetDefaultConvCommandConfig(&commandConfigStruct[ADC1_B9]);
    commandConfigStruct[ADC1_B9].enableChannelB = true;
    commandConfigStruct[ADC1_B9].channelBNumber = 9;
    commandConfigStruct[ADC1_B9].sampleChannelMode = kLPADC_SampleChannelSingleEndSideB;
    LPADC_SetConvCommandConfig(ADC1, 8, &commandConfigStruct[ADC1_B9]);
    LPADC_GetDefaultConvTriggerConfig(&triggerConfigStruct[ADC1_B9]);
    triggerConfigStruct[ADC1_B9].targetCommandId       = 8U;
    triggerConfigStruct[ADC1_B9].enableHardwareTrigger = false;
    triggerConfigStruct[ADC1_B9].channelBFIFOSelect = 0;

    /* ADC1_B10 - DCI1 */
    LPADC_GetDefaultConvCommandConfig(&commandConfigStruct[ADC1_B10]);
    commandConfigStruct[ADC1_B10].enableChannelB = true;
    commandConfigStruct[ADC1_B10].channelBNumber = 10;
    commandConfigStruct[ADC1_B10].sampleChannelMode = kLPADC_SampleChannelSingleEndSideB;
    LPADC_SetConvCommandConfig(ADC1, 9, &commandConfigStruct[ADC1_B10]);
    LPADC_GetDefaultConvTriggerConfig(&triggerConfigStruct[ADC1_B10]);
    triggerConfigStruct[ADC1_B10].targetCommandId       = 9U;
    triggerConfigStruct[ADC1_B10].enableHardwareTrigger = false;
    triggerConfigStruct[ADC1_B10].channelBFIFOSelect = 0;

    /* ADC1_B11 - DCI2 */
    LPADC_GetDefaultConvCommandConfig(&commandConfigStruct[ADC1_B11]);
    commandConfigStruct[ADC1_B11].enableChannelB = true;
    commandConfigStruct[ADC1_B11].channelBNumber = 11;
    commandConfigStruct[ADC1_B11].sampleChannelMode = kLPADC_SampleChannelSingleEndSideB;
    LPADC_SetConvCommandConfig(ADC1, 10, &commandConfigStruct[ADC1_B11]);
    LPADC_GetDefaultConvTriggerConfig(&triggerConfigStruct[ADC1_B11]);
    triggerConfigStruct[ADC1_B11].targetCommandId       = 10U;
    triggerConfigStruct[ADC1_B11].enableHardwareTrigger = false;
    triggerConfigStruct[ADC1_B11].channelBFIFOSelect = 0;
}

/*================================================================
 * Sensor Task - periodic ADC read + RMS calculation
 *================================================================*/
void prvSensorTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t tick_start, tick_end, delay_target;

    for (;;) {
        tick_start = xTaskGetTickCount();

        readSensor();

        tick_end = xTaskGetTickCount();

        if (tick_end - tick_start >= pdMS_TO_TICKS(LV_SENSOR_DEF_REFR_PERIOD)) {
            delay_target = 1;
        } else {
            delay_target = pdMS_TO_TICKS(LV_SENSOR_DEF_REFR_PERIOD) - (tick_end - tick_start);
        }

        vTaskDelay(delay_target);
    }
}

/*================================================================
 * ADC Reading - software-trigger all channels
 *================================================================*/
void readSensor(void)
{
    /* Trigger AMB_TEMP_ADC and VOUT_DC2_ADC */
    LPADC_SetConvTriggerConfig(ADC0, 0U, &triggerConfigStruct[ADC0_A6B6]);
    LPADC_DoSoftwareTrigger(ADC0, 1U);
    while (!LPADC_GetConvResult(ADC0, &resultStruct[ADC0_A6B6], 1U)) {}
    adcValue[VOUT_DC2_ADC] = (resultStruct[ADC0_A6B6].convValue >> g_LpadcResultShift);

    while (!LPADC_GetConvResult(ADC0, &resultStruct[ADC0_A6B6], 0U)) {}
    adcValue[AMB_TEMP_ADC] = (resultStruct[ADC0_A6B6].convValue >> g_LpadcResultShift);

    /* IOUT_DC1_ADC */
    LPADC_SetConvTriggerConfig(ADC1, 0U, &triggerConfigStruct[ADC1_A6]);
    LPADC_DoSoftwareTrigger(ADC1, 1U);
    while (!LPADC_GetConvResult(ADC1, &resultStruct[ADC1_A6], 0U)) {}
    adcValue[IOUT_DC1_ADC] = (resultStruct[ADC1_A6].convValue >> g_LpadcResultShift);

    /* VOUT_DC1_ADC */
    LPADC_SetConvTriggerConfig(ADC1, 0U, &triggerConfigStruct[ADC1_B5]);
    LPADC_DoSoftwareTrigger(ADC1, 1U);
    while (!LPADC_GetConvResult(ADC1, &resultStruct[ADC1_B5], 0U)) {}
    adcValue[VOUT_DC1_ADC] = (resultStruct[ADC1_B5].convValue >> g_LpadcResultShift);

    /* DCDC1_TEMP_ADC */
    LPADC_SetConvTriggerConfig(ADC1, 0U, &triggerConfigStruct[ADC1_B8]);
    LPADC_DoSoftwareTrigger(ADC1, 1U);
    while (!LPADC_GetConvResult(ADC1, &resultStruct[ADC1_B8], 0U)) {}
    adcValue[DCDC1_TEMP_ADC] = (resultStruct[ADC1_B8].convValue >> g_LpadcResultShift);

    /* DCDC2_TEMP_ADC */
    LPADC_SetConvTriggerConfig(ADC1, 0U, &triggerConfigStruct[ADC1_B9]);
    LPADC_DoSoftwareTrigger(ADC1, 1U);
    while (!LPADC_GetConvResult(ADC1, &resultStruct[ADC1_B9], 0U)) {}
    adcValue[DCDC2_TEMP_ADC] = (resultStruct[ADC1_B9].convValue >> g_LpadcResultShift);

    /* DCI1_ADC */
    LPADC_SetConvTriggerConfig(ADC1, 0U, &triggerConfigStruct[ADC1_B10]);
    LPADC_DoSoftwareTrigger(ADC1, 1U);
    while (!LPADC_GetConvResult(ADC1, &resultStruct[ADC1_B10], 0U)) {}
    adcValue[DCI1_ADC] = (resultStruct[ADC1_B10].convValue >> g_LpadcResultShift);

    /* DCI2_ADC */
    LPADC_SetConvTriggerConfig(ADC1, 0U, &triggerConfigStruct[ADC1_B11]);
    LPADC_DoSoftwareTrigger(ADC1, 1U);
    while (!LPADC_GetConvResult(ADC1, &resultStruct[ADC1_B11], 0U)) {}
    adcValue[DCI2_ADC] = (resultStruct[ADC1_B11].convValue >> g_LpadcResultShift);

    /* PFC_VIN_ADC and PFC_I_IN_ADC */
    LPADC_SetConvTriggerConfig(ADC0, 0U, &triggerConfigStruct[ADC0_A0B0]);
    LPADC_DoSoftwareTrigger(ADC0, 1U);
    while (!LPADC_GetConvResult(ADC0, &resultStruct[ADC0_A0B0], 1U)) {}
    adcValue[PFC_VIN_ADC] = (resultStruct[ADC0_A0B0].convValue >> g_LpadcResultShift);

    while (!LPADC_GetConvResult(ADC0, &resultStruct[ADC0_A0B0], 0U)) {}
    adcValue[PFC_I_IN_ADC] = (resultStruct[ADC0_A0B0].convValue >> g_LpadcResultShift);

    /* PFC_I_OUT_ADC and PFC_VOUT_ADC */
    LPADC_SetConvTriggerConfig(ADC1, 0U, &triggerConfigStruct[ADC1_A0B0]);
    LPADC_DoSoftwareTrigger(ADC1, 1U);
    while (!LPADC_GetConvResult(ADC1, &resultStruct[ADC1_A0B0], 1U)) {}
    adcValue[PFC_VOUT_ADC] = (resultStruct[ADC1_A0B0].convValue >> g_LpadcResultShift);

    while (!LPADC_GetConvResult(ADC1, &resultStruct[ADC1_A0B0], 0U)) {}
    adcValue[PFC_I_OUT_ADC] = (resultStruct[ADC1_A0B0].convValue >> g_LpadcResultShift);

    /* PFC1_TEMP_ADC */
    LPADC_SetConvTriggerConfig(ADC0, 0U, &triggerConfigStruct[ADC0_A3]);
    LPADC_DoSoftwareTrigger(ADC0, 1U);
    while (!LPADC_GetConvResult(ADC0, &resultStruct[ADC0_A3], 0U)) {}
    adcValue[PFC1_TEMP_ADC] = (resultStruct[ADC0_A3].convValue >> g_LpadcResultShift);

    /* PFC2_TEMP_ADC */
    LPADC_SetConvTriggerConfig(ADC0, 0U, &triggerConfigStruct[ADC0_A7]);
    LPADC_DoSoftwareTrigger(ADC0, 1U);
    while (!LPADC_GetConvResult(ADC0, &resultStruct[ADC0_A7], 0U)) {}
    adcValue[PFC2_TEMP_ADC] = (resultStruct[ADC0_A7].convValue >> g_LpadcResultShift);

    /* ANALOG_KEYBOARD_ADC */
    LPADC_SetConvTriggerConfig(ADC1, 0U, &triggerConfigStruct[ADC1_B6]);
    LPADC_DoSoftwareTrigger(ADC1, 1U);
    while (!LPADC_GetConvResult(ADC1, &resultStruct[ADC1_B6], 0U)) {}
    adcValue[ANALOG_KEYBOARD_ADC] = (resultStruct[ADC1_B6].convValue >> g_LpadcResultShift);

    caclulateVrms();
    caclulateIrms();

    sensor_status.powerDataUpdated = true;
}

/*================================================================
 * RMS Calculations
 *================================================================*/
static void caclulateVrms(void)
{
    Vrms.adcValue = adcValue[PFC_VIN_ADC];
    if (Vrms.adcValue >= 5) {
        Vrms.isPositivalHalfFlag = true;
    } else {
        Vrms.isPositivalHalfFlag = false;
    }

    if (Vrms.isPositivalHalfFlag) {
        Vrms.sum += Vrms.adcValue * Vrms.adcValue;
        Vrms.sumCnts++;
    } else {
        if (Vrms.sumCnts > 0) {
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

    Vrms.real = (float)Vrms.rootMeanSqure / 4096 * 3.3 * 10000 / 19;
}

static void caclulateIrms(void)
{
    Irms.adcValue = adcValue[PFC_I_IN_ADC];
    if (Irms.adcValue >= 5) {
        Irms.isPositivalHalfFlag = true;
    } else {
        Irms.isPositivalHalfFlag = false;
    }

    if (Irms.isPositivalHalfFlag) {
        Irms.sum += Irms.adcValue * Irms.adcValue;
        Irms.sumCnts++;
    } else {
        if (Irms.sumCnts > 0) {
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
