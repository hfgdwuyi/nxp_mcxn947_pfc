/*
 * can.c - CAN bus interface
 *
 * Handles CAN configuration, ISR callback, TX/RX tasks.
 * Separate txFrame/rxFrame prevent TX/RX concurrency races.
 */

#include "can.h"
#include "board.h"
#include "clock_config.h"
#include "fsl_device_registers.h"
#include "fsl_flexcan.h"
#include "fsl_debug_console.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define RX_MESSAGE_BUFFER_NUM (0)
#define TX_MESSAGE_BUFFER_NUM (1)
#define CAN_CLK_FREQ          CLOCK_GetFlexcanClkFreq(0U)

/* IPC objects - exported */
QueueHandle_t xCanRxQueue = NULL;
SemaphoreHandle_t xCanTxMutex = NULL;

/* Module-local state */
static flexcan_config_t flexcanConfig;
static flexcan_rx_mb_config_t mbConfig;
static flexcan_handle_t flexcanHandle;
static flexcan_frame_t rxFrame;
static flexcan_frame_t txFrame;
static flexcan_mb_transfer_t txXfer, rxXfer;

/*================================================================
 * CAN ISR callback - minimal: extract frame, send to queue, yield
 *================================================================*/
static FLEXCAN_CALLBACK(flexcan_callback)
{
    (void)base;
    (void)handle;
    (void)userData;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    can_message_t msg;

    switch (status) {
    case kStatus_FLEXCAN_RxIdle:
        if (RX_MESSAGE_BUFFER_NUM == result) {
            msg.id = rxFrame.id;
            msg.length = rxFrame.length;
            uint8_t *dataPtr = msg.data;
            uint32_t word0 = rxFrame.dataWord0;
            uint32_t word1 = rxFrame.dataWord1;

            for (uint8_t i = 0; i < 4; i++) {
                if (msg.length > i) {
                    *dataPtr++ = (word0 >> (i * 8)) & 0xFF;
                }
            }
            for (uint8_t i = 0; i < 4; i++) {
                if (msg.length > (4 + i)) {
                    *dataPtr++ = (word1 >> (i * 8)) & 0xFF;
                }
            }

            if (xQueueSendFromISR(xCanRxQueue, &msg, &xHigherPriorityTaskWoken) != pdTRUE) {
                /* Queue full - frame dropped */
            }

            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

            (void)FLEXCAN_TransferReceiveNonBlocking(CAN0, &flexcanHandle, &rxXfer);
        }
        break;

    case kStatus_FLEXCAN_TxIdle:
        break;

    case kStatus_FLEXCAN_WakeUp:
        break;

    default:
        break;
    }
}

/*================================================================
 * CAN Configuration
 *================================================================*/
void CAN_Configure(void)
{
    FLEXCAN_GetDefaultConfig(&flexcanConfig);
    flexcanConfig.disableSelfReception = true;
    flexcanConfig.bitRate = 500000U;

    flexcan_timing_config_t timing_config;
    memset(&timing_config, 0, sizeof(flexcan_timing_config_t));

    if (FLEXCAN_CalculateImprovedTimingValues(CAN0, flexcanConfig.bitRate, CAN_CLK_FREQ, &timing_config)) {
        memcpy(&(flexcanConfig.timingConfig), &timing_config, sizeof(flexcan_timing_config_t));
    } else {
        PRINTF("No found Improved Timing Configuration. Just used default configuration\r\n");
    }

    FLEXCAN_Init(CAN0, &flexcanConfig, CAN_CLK_FREQ);
    FLEXCAN_TransferCreateHandle(CAN0, &flexcanHandle, flexcan_callback, NULL);

    FLEXCAN_SetRxMbGlobalMask(CAN0, 0);

    mbConfig.format = kFLEXCAN_FrameFormatStandard;
    mbConfig.type   = kFLEXCAN_FrameTypeData;
    mbConfig.id     = FLEXCAN_ID_STD(0);
    FLEXCAN_SetRxMbConfig(CAN0, RX_MESSAGE_BUFFER_NUM, &mbConfig, true);
    FLEXCAN_SetTxMbConfig(CAN0, TX_MESSAGE_BUFFER_NUM, true);

    rxXfer.mbIdx = (uint8_t)RX_MESSAGE_BUFFER_NUM;
    rxXfer.frame = &rxFrame;

    if (FLEXCAN_TransferReceiveNonBlocking(CAN0, &flexcanHandle, &rxXfer) != kStatus_Success) {
        PRINTF("Failed to start CAN reception!\r\n");
    }

    FLEXCAN_EnableInterrupts(CAN0, kFLEXCAN_WakeUpInterruptEnable);
    NVIC_SetPriority(CAN0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
    NVIC_EnableIRQ(CAN0_IRQn);
}

/*================================================================
 * CAN TX helper - uses txFrame (separate from rxFrame used by ISR)
 *================================================================*/
void sendCAN(void)
{
    txFrame.id     = FLEXCAN_ID_STD(0x123);
    txFrame.format = (uint8_t)kFLEXCAN_FrameFormatStandard;
    txFrame.type   = (uint8_t)kFLEXCAN_FrameTypeData;
    txFrame.length = (uint8_t)DLC;

    txXfer.mbIdx = (uint8_t)TX_MESSAGE_BUFFER_NUM;
    txXfer.frame = &txFrame;
    txFrame.dataByte0 = 0x11;
    txFrame.dataByte1 = 0x11;
    txFrame.dataByte2 = 0x11;
    txFrame.dataByte3 = 0x11;
    txFrame.dataByte4 = 0x11;
    txFrame.dataByte5 = 0x11;
    txFrame.dataByte6 = 0x11;
    txFrame.dataByte7 = 0x11;
    (void)FLEXCAN_TransferSendNonBlocking(CAN0, &flexcanHandle, &txXfer);
}

/*================================================================
 * CAN RX Task - blocks on queue, prints received frames
 *================================================================*/
void vTaskCANRx(void *pvParameters)
{
    (void)pvParameters;
    can_message_t rxMsg;
    PRINTF("CAN_RX Task started\r\n");

    for (;;) {
        if (xQueueReceive(xCanRxQueue, &rxMsg, pdMS_TO_TICKS(1000))) {
            PRINTF("RX ID: 0x%03X, Length: %d, Data: ",
                   (rxMsg.id >> CAN_ID_STD_SHIFT), rxMsg.length);
            for (uint8_t i = 0; i < rxMsg.length; i++) {
                PRINTF("0x%02X ", rxMsg.data[i]);
            }
            PRINTF("\r\n");
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

/*================================================================
 * CAN TX Task - periodic CAN transmit
 *================================================================*/
void vTaskCANTx(void *pvParameters)
{
    (void)pvParameters;
    TickType_t tick_start, tick_end, delay_target;

    for (;;) {
        tick_start = xTaskGetTickCount();

        sendCAN();

        tick_end = xTaskGetTickCount();

        if (tick_end - tick_start >= pdMS_TO_TICKS(LV_CAN_DEF_REFR_PERIOD)) {
            delay_target = 1;
        } else {
            delay_target = pdMS_TO_TICKS(LV_CAN_DEF_REFR_PERIOD) - (tick_end - tick_start);
        }

        vTaskDelay(delay_target);
    }
}
