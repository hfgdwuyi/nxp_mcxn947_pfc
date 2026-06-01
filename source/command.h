/*
 * command.h - UART command interface
 */

#ifndef COMMAND_H
#define COMMAND_H

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#define RX_BUFFER_SIZE 256

typedef struct {
    uint8_t data[256];
    uint16_t length;
    uint32_t timestamp;
} uart_message_t;

extern QueueHandle_t xUartRxQueue;
extern SemaphoreHandle_t xUartTxMutex;

void UART_Configure(void);
void processReceivedCommand(const char *data);
void prvUartRxTask(void *pvParameters);

#endif /* COMMAND_H */
