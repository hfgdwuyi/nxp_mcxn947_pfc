/*
 * key.c - Analog keyboard (ADC-based)
 *
 * Scans 5 buttons through a resistive ladder on a single ADC channel.
 * Sends key events to the key message queue.
 */

#include "key.h"
#include "sensor.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

QueueHandle_t xKeyMessageQueue = NULL;

typedef struct {
    uint16_t minValue;
    uint16_t maxValue;
    void (*action)(void);
} ADCRange_t;

static key_status_t key_status;

static void actionForSW1(void);
static void actionForSW2(void);
static void actionForSW3(void);
static void actionForSW4(void);
static void actionForSW5(void);

static const ADCRange_t adcRanges[6] = {
    {0,    100,   actionForSW1},
    {500,  700,   actionForSW2},
    {1200, 1400,  actionForSW3},
    {1900, 2100,  actionForSW4},
    {2800, 3000,  actionForSW5},
    {3000, 4095,  NULL}
};

/*================================================================
 * Key Task - periodic key scan
 *================================================================*/
void prvKeyTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t tick_start, tick_end, delay_target;

    for (;;) {
        tick_start = xTaskGetTickCount();

        keyScan();

        tick_end = xTaskGetTickCount();

        if (tick_end - tick_start >= pdMS_TO_TICKS(LV_KEY_DEF_REFR_PERIOD)) {
            delay_target = 1;
        } else {
            delay_target = pdMS_TO_TICKS(LV_KEY_DEF_REFR_PERIOD) - (tick_end - tick_start);
        }

        vTaskDelay(delay_target);
    }
}

/*================================================================
 * Key scanning
 *================================================================*/
void keyScan(void)
{
    uint16_t value = adcValue[ANALOG_KEYBOARD_ADC];
    for (int i = 0; i < 5; i++) {
        if (value >= adcRanges[i].minValue && value <= adcRanges[i].maxValue) {
            if (adcRanges[i].action != NULL && key_status.dataUpdated) {
                adcRanges[i].action();
                key_status.dataUpdated = false;
            } else {
                key_status.dataUpdated = true;
            }
            break;
        }
    }
}

static void actionForSW1(void)
{
    key_message_t msg = {MSG_INCREASE_PAGE, false};
    xQueueSend(xKeyMessageQueue, &msg, 0);
}

static void actionForSW2(void)
{
    key_message_t msg = {MSG_DECREASE_PAGE, false};
    xQueueSend(xKeyMessageQueue, &msg, 0);
}

static void actionForSW3(void)
{
    key_message_t msg = {MSG_INCREASE_VALUE, false};
    xQueueSend(xKeyMessageQueue, &msg, 0);
}

static void actionForSW4(void)
{
    key_message_t msg = {MSG_DECREASE_VALUE, false};
    xQueueSend(xKeyMessageQueue, &msg, 0);
}

static void actionForSW5(void)
{
    key_message_t msg = {MSG_CONFIRM, true};
    xQueueSend(xKeyMessageQueue, &msg, 0);
}
