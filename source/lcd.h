/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef LCD_H
#define LCD_H

#include <stdint.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*Default display refresh period in milliseconds. LVG will redraw changed areas with this period time*/
#define LV_DISP_DEF_REFR_PERIOD 1000
/*******************************************************************************
 * API
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint8_t ubFontSize;
  uint8_t ubFontXAxis;
  uint8_t ubFontYAxis;
} LCD_FontTypeDef;

#if defined(__cplusplus)
}
#endif

extern void Display(void);
extern void lv_port_disp_init(void);
#endif /*LVGL_SUPPORT_H */
