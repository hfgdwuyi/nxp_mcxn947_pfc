/*
 * fan.h - Fan speed control
 */

#ifndef FAN_H
#define FAN_H

#include <stdint.h>

#define LV_FAN_DEF_REFR_PERIOD 100

void fanSpeedControl(void);
void prvFanTask(void *pvParameters);

#endif /* FAN_H */
