/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "main.h"
#include "fsl_device_registers.h"
#include "fsl_flexcan.h"

void sendCAN(void);

void sendCAN(void)
{
    frame.id     = FLEXCAN_ID_STD(0x123);
    frame.format = (uint8_t)kFLEXCAN_FrameFormatStandard;
    frame.type   = (uint8_t)kFLEXCAN_FrameTypeData;
    frame.length = (uint8_t)DLC;

    txXfer.mbIdx = (uint8_t)TX_MESSAGE_BUFFER_NUM;
    txXfer.frame = &frame;
    frame.dataByte0 = 0x11;
    frame.dataByte1 = 0x11;
    frame.dataByte2 = 0x11;
    frame.dataByte3 = 0x11;
    frame.dataByte4 = 0x11;
    frame.dataByte5 = 0x11;
    frame.dataByte6 = 0x11;
    frame.dataByte7 = 0x11;
    (void)FLEXCAN_TransferSendNonBlocking(CAN0, &flexcanHandle, &txXfer);
}
