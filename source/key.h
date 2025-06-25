/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// command.h
#ifndef KEY_H
#define KEY_H

#include <stdint.h>
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define LV_KEY_DEF_REFR_PERIOD 100

typedef struct {
    bool dataUpdated;
} key_status_t;

extern void keyScan(void);
#endif



