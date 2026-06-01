/*
 * lcd.h - LCD 128x64 display driver
 */

#ifndef LCD_H
#define LCD_H

#include <stdint.h>

#define LV_DISP_DEF_REFR_PERIOD 1000

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t ubFontSize;
    uint8_t ubFontXAxis;
    uint8_t ubFontYAxis;
} LCD_FontTypeDef;

#if defined(__cplusplus)
}
#endif

extern uint32_t currentPage;
extern uint32_t currentValue;

void Display(void);
void lv_port_disp_init(void);
void prvDisplayTask(void *pvParameters);

#endif /* LCD_H */
