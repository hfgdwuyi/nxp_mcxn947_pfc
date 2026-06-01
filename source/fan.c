/*
 * fan.c - Fan speed control (stub)
 */

#include "fan.h"
#include "fsl_debug_console.h"

#include "FreeRTOS.h"
#include "task.h"

/*================================================================
 * Fan Task - periodic fan speed control
 *================================================================*/
void prvFanTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t tick_start, tick_end, delay_target;

    for (;;) {
        tick_start = xTaskGetTickCount();

        fanSpeedControl();

        tick_end = xTaskGetTickCount();

        if (tick_end - tick_start >= pdMS_TO_TICKS(LV_FAN_DEF_REFR_PERIOD)) {
            delay_target = 1;
        } else {
            delay_target = pdMS_TO_TICKS(LV_FAN_DEF_REFR_PERIOD) - (tick_end - tick_start);
        }

        vTaskDelay(delay_target);
    }
}

/*================================================================
 * Fan speed control (stub - to be implemented)
 *================================================================*/
void fanSpeedControl(void)
{
    /* TODO: Implement fan speed control based on temperature */
}
