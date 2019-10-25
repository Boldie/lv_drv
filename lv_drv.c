/*
 * lv_drv.c
 *
 *  Created on: Oct 25, 2019
 *      Author: sven
 */

#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "esp_freertos_hooks.h"

#include "disp_spi.h"
#include "ili9341.h"
#include "xpt2046.h"

static void IRAM_ATTR lv_tick_task(void)
{
  lv_tick_inc(portTICK_RATE_MS);
}

void init_lv_drv()
{
  lvgl_drv_spi_init();
  ili9341_init();
  xpt2046_init();

  static lv_color_t buf1[DISP_BUF_SIZE];
  static lv_color_t buf2[DISP_BUF_SIZE];
  static lv_disp_buf_t disp_buf;
  lv_disp_buf_init(&disp_buf, buf1, buf2, DISP_BUF_SIZE);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.flush_cb = ili9341_flush;
  disp_drv.buffer = &disp_buf;
  lv_disp_drv_register(&disp_drv);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.read_cb = xpt2046_read;
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  lv_indev_drv_register(&indev_drv);

  esp_register_freertos_tick_hook(lv_tick_task);
}
