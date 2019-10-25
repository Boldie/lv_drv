/**
 * @file lv_templ.h
 *
 */

#ifndef ILI9341_H
#define ILI9341_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl/lvgl.h"

#define DISP_BUF_SIZE (LV_HOR_RES_MAX * 40)

/**
 * Initializes the library part of the display driver.
 * Needs to be called before use.
 */
void ili9341_init(void);

/**
 * Callback handler for the lvgl library to integrate the
 * flushing.
 */
void ili9341_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map);


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*ILI9341_H*/
