/*
 * key.h - Analog keyboard (ADC-based)
 */

#ifndef KEY_H
#define KEY_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "main.h"

#define LV_KEY_DEF_REFR_PERIOD 100

extern QueueHandle_t xKeyMessageQueue;

void keyScan(void);
void prvKeyTask(void *pvParameters);

#endif /* KEY_H */
