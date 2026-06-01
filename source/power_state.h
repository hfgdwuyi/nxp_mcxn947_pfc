/*
 * power_state.h - Power-on sequencing state machine
 *
 * Manages the startup sequence:
 *   INIT -> SELFCHECK -> SOFTSTART -> PFC_READY -> RUNNING
 */

#ifndef POWER_STATE_H
#define POWER_STATE_H

#include <stdbool.h>

typedef enum {
    POWER_STATE_INIT,
    POWER_STATE_SELFCHECK,
    POWER_STATE_SOFTSTART,
    POWER_STATE_PFC_READY,
    POWER_STATE_RUNNING
} PowerState_t;

/* Task entry points */
void prvSelfCheckTask(void *pvParameters);
void prvSoftStartTask(void *pvParameters);
void prvPFCControlTask(void *pvParameters);
void prvDCControlTask(void *pvParameters);

/* State query */
PowerState_t prvGetPowerState(void);
bool prvIsRelayEnabled(void);
bool prvIsPFCEnabled(void);

#endif /* POWER_STATE_H */
