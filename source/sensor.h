/*
 * sensor.h - ADC sensor reading and RMS calculation
 */

#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "fsl_lpadc.h"

#define LV_SENSOR_DEF_REFR_PERIOD 1
#define LPADC_VREF_SOURCE kLPADC_ReferenceVoltageAlt3

/* ADC value indices */
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
    ANALOG_KEYBOARD_ADC,
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
    ADC1_B6,
    ADC1_A6,
    ADC1_B8,
    ADC1_B9,
    ADC1_B10,
    ADC1_B11,
} adc_channel_name_t;

extern uint16_t adcValue[16];
extern lpadc_conv_trigger_config_t triggerConfigStruct[13];
extern lpadc_conv_command_config_t commandConfigStruct[13];
extern lpadc_conv_result_t resultStruct[13];

/* RMS output - only .real is valid for external consumers */
typedef struct {
    float real;
} rms_message_t;

typedef struct {
    float real;
} constant_message_t;

typedef struct {
    bool powerDataUpdated;
} sensor_status_t;

extern rms_message_t Vrms, Irms;
extern constant_message_t Vout, Iout;
extern sensor_status_t sensor_status;

void ADC_Configure(void);
void readSensor(void);
void prvSensorTask(void *pvParameters);

#endif /* SENSOR_H */
