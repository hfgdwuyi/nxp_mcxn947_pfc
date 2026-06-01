/*
 * can.h - CAN bus interface
 */

#ifndef CAN_H
#define CAN_H

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#define LV_CAN_DEF_REFR_PERIOD 1000
#define DLC 8

typedef struct {
    uint32_t id;
    uint8_t data[8];
    uint8_t length;
    uint32_t timestamp;
} can_message_t;

extern QueueHandle_t xCanRxQueue;
extern SemaphoreHandle_t xCanTxMutex;

void CAN_Configure(void);
void sendCAN(void);
void vTaskCANRx(void *pvParameters);
void vTaskCANTx(void *pvParameters);

#endif /* CAN_H */
