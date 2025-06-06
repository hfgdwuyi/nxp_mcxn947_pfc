/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// command.h
#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*Default display refresh period in milliseconds. LVG will redraw changed areas with this period time*/
#define LV_SENSOR_DEF_REFR_PERIOD 1000


extern void readSensor(void);

#endif



