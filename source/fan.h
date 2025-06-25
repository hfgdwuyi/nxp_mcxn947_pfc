/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// fan.h
#ifndef FAN_H
#define FAN_H

#include <stdint.h>
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define LV_FAN_DEF_REFR_PERIOD 100

extern void fanSpeedControl(void);

#endif



