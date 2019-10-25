/**
 * @file XPT2046.c
 *
 */

#include "xpt2046.h"

#include "esp_log.h"
#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include <stddef.h>

#include "disp_spi.h"
#include "disp_spi_int.h"

#define Z_THRESHOLD 100

static const char* TAG = "xpt2046";

typedef struct
{
  float KX1, KX2, KX3, KY1, KY2, KY3;
} xpt2046CalibData;

static bool rawMode = false;
static xpt2046CalibData calibData = { 1, 0, 0, 0, 1, 0};
static bool validCalibration = false;

void xpt2046_init(void)
{
  ESP_LOGI(TAG, "Init");

  // This seems to be complicated, but only gpio_config can set every pin to function
  // Some pins may have JTAG as default and therefore will not work.
  gpio_config_t io_conf;
  //disable interrupt
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  //set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  //bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pin_bit_mask = ((1ULL<<CONFIG_LVGL_DRV_TP_SPI_CS));
  //disable pull-down mode
  io_conf.pull_down_en = 0;
  //disable pull-up mode
  io_conf.pull_up_en = 0;
  //configure GPIO with the given settings
  gpio_config(&io_conf);

  gpio_set_level(CONFIG_LVGL_DRV_TP_SPI_CS, 1);


  ESP_LOGI(TAG, "Loading calibration data from NVS ...");
  // Initialize NVS
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // NVS partition was truncated and needs to be erased
    // Retry nvs_flash_init
    ESP_LOGI(TAG, "NVS flash initialize ...");
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  nvs_handle my_handle;
  err = nvs_open("xpt2046", NVS_READONLY, &my_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
  } else {
    size_t len = sizeof(calibData);
    err = nvs_get_blob(my_handle, "Calibration", &calibData, &len);
    if (err == ESP_OK) {
      ESP_LOGI(TAG, "Loaded Calibration from NVS: KX1=%f, KX2=%f, KX3=%f, KY1=%f, KY2=%f, KY3=%f"
               , calibData.KX1, calibData.KX2, calibData.KX3
               , calibData.KY1, calibData.KY2, calibData.KY3 );
      validCalibration = true;
    }
    else {
      ESP_LOGE(TAG, "Unable to load calibration: %s", esp_err_to_name(err));
    }
  }
  nvs_close(my_handle);
}

bool xpt2046_hasValidCalbiration()
{
  return validCalibration;
}

void xpt2046_setRawMode( bool on )
{
  ESP_LOGV(TAG, "xpt2046 set raw mode to %d", (int)(on));
  rawMode = on;
}

// Algorithm was described and adapted from here:
// https://www.analog.com/media/en/technical-documentation/application-notes/AN-1021.pdf
bool xpt2046_calculateCalibration( CalibPoint* points, uint8_t count )
{
  int i;
  double a[3], b[3], c[3], d[3], k;

  if (count < 3) {
    return false;
  }
  else {
    if (count == 3) {
      for (i = 0; i < count; i++) {
        a[i] = (double) (points[i].raw.x);
        b[i] = (double) (points[i].raw.y);
        c[i] = (double) (points[i].screen.x);
        d[i] = (double) (points[i].screen.y);
      }
    } else if (count > 3) {
      for (i = 0; i < 3; i++) {
        a[i] = 0;
        b[i] = 0;
        c[i] = 0;
        d[i] = 0;
      }
      for (i = 0; i < count; i++) {
        a[2] = a[2] + (double) (points[i].raw.x);
        b[2] = b[2] + (double) (points[i].raw.y);
        c[2] = c[2] + (double) (points[i].screen.x);
        d[2] = d[2] + (double) (points[i].screen.y);
        a[0] = a[0] + (double) (points[i].raw.x) * (double) (points[i].raw.x);
        a[1] = a[1] + (double) (points[i].raw.x) * (double) (points[i].raw.y);
        b[0] = a[1];
        b[1] = b[1] + (double) (points[i].raw.y) * (double) (points[i].raw.y);
        c[0] = c[0] + (double) (points[i].raw.x) * (double) (points[i].screen.x);
        c[1] = c[1] + (double) (points[i].raw.y) * (double) (points[i].screen.x);
        d[0] = d[0] + (double) (points[i].raw.x) * (double) (points[i].screen.y);
        d[1] = d[1] + (double) (points[i].raw.y) * (double) (points[i].screen.y);
      }
      a[0] = a[0] / a[2];
      a[1] = a[1] / b[2];
      b[0] = b[0] / a[2];
      b[1] = b[1] / b[2];
      c[0] = c[0] / a[2];
      c[1] = c[1] / b[2];
      d[0] = d[0] / a[2];
      d[1] = d[1] / b[2];
      a[2] = a[2] / count;
      b[2] = b[2] / count;
      c[2] = c[2] / count;
      d[2] = d[2] / count;
    }

    k = (a[0] - a[2]) * (b[1] - b[2]) - (a[1] - a[2]) * (b[0] - b[2]);
    if (k == 0) {
      ESP_LOGE(TAG, "k-factor is zero: a[0]=%f, a[1]=%f, a[2]=%f, b[0]=%f, b[1]=%f, b[2]=%f"
               , a[0], a[1], a[2], b[0], b[1], b[2] );
      return false;
    }

    calibData.KX1 = ((c[0] - c[2]) * (b[1] - b[2]) - (c[1] - c[2]) * (b[0] - b[2])) / k;
    calibData.KX2 = ((c[1] - c[2]) * (a[0] - a[2]) - (c[0] - c[2]) * (a[1] - a[2])) / k;
    calibData.KX3 = (b[0] * (a[2] * c[1] - a[1] * c[2])
        + b[1] * (a[0] * c[2] - a[2] * c[0])
        + b[2] * (a[1] * c[0] - a[0] * c[1])) / k;
    calibData.KY1 = ((d[0] - d[2]) * (b[1] - b[2]) - (d[1] - d[2]) * (b[0] - b[2])) / k;
    calibData.KY2 = ((d[1] - d[2]) * (a[0] - a[2]) - (d[0] - d[2]) * (a[1] - a[2])) / k;
    calibData.KY3 = (b[0] * (a[2] * d[1] - a[1] * d[2])
        + b[1] * (a[0] * d[2] - a[2] * d[0])
        + b[2] * (a[1] * d[0] - a[0] * d[1])) / k;

    ESP_LOGI(TAG, "Calibration: KX1=%f, KX2=%f, KX3=%f, KY1=%f, KY2=%f, KY3=%f"
             , calibData.KX1, calibData.KX2, calibData.KX3
             , calibData.KY1, calibData.KY2, calibData.KY3 );

    validCalibration = true;

    ESP_LOGI(TAG, "Storing to NVS begin.");
    nvs_handle my_handle;
    esp_err_t err = nvs_open("xpt2046", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    } else {
      size_t len = sizeof(calibData);
      err = nvs_set_blob(my_handle, "Calibration", &calibData, len);
      if (err == ESP_OK) {
        nvs_commit(my_handle);
      }
      else {
        ESP_LOGE(TAG, "Unable to store calibration: %s", esp_err_to_name(err));
      }

      nvs_commit(my_handle);
    }
    nvs_close(my_handle);
    ESP_LOGI(TAG, "Storing to NVS done.");

  }

  return true;
}

int16_t xpt2046_bestofavg( int16_t x , int16_t y , int16_t z )
{
  int16_t d[3];

  d[0] = x > y ? x - y : y - x;
  d[1] = x > z ? x - z : z - x;
  d[2] = z > y ? z - y : y - z;

  if ( d[0] <= d[1] && d[0] <= d[2] ) return (x + y) >> 1;
  else if ( d[1] <= d[2] ) return (x + z) >> 1;
  else return (y + z) >> 1;
}

bool xpt2046_read(lv_indev_drv_t * drv, lv_indev_data_t * data)
{
  static int16_t lastValidX = 0;
  static int16_t lastValidY = 0;

  bool valid = false;
  int16_t x = 0;
  int16_t y = 0;

  disp_tp_spi_finished();

  gpio_set_level(CONFIG_LVGL_DRV_TP_SPI_CS, 0);
  int16_t z = tp_spi_xchg_xpt(0xB1) >> 3; // Z1

  if (z >= Z_THRESHOLD) {
    tp_spi_xchg_xpt(0x91);  // dummy, reduce noise

    int16_t tmpData[6];
    tmpData[0] = tp_spi_xchg_xpt(0xD1) >> 3; // Y
    tmpData[1] = tp_spi_xchg_xpt(0x91) >> 3; // X
    tmpData[2] = tp_spi_xchg_xpt(0xD1) >> 3; // Y
    tmpData[3] = tp_spi_xchg_xpt(0x91) >> 3; // X
    tmpData[4] = tp_spi_xchg_xpt(0xD1) >> 3; // Y
    tmpData[5] = tp_spi_xchg_xpt(0x90) >> 3; // Last X power down
    valid = true;

    x = xpt2046_bestofavg( tmpData[0], tmpData[2], tmpData[4] );
    y = xpt2046_bestofavg( tmpData[1], tmpData[3], tmpData[5] );

    if (!rawMode)
    {
      int16_t xRaw = x;
      int16_t yRaw = y;

      x = (int16_t)(calibData.KX1 * xRaw + calibData.KX2 * yRaw + calibData.KX3 + 0.5);
      y = (int16_t)(calibData.KY1 * xRaw + calibData.KY2 * yRaw + calibData.KY3 + 0.5);

      ESP_LOGV(TAG, "z=%5d x=%5d,  y=%5d -> x=%5d,  y=%5d ", z, xRaw, yRaw, x, y);
    }
    else
    {
      ESP_LOGV(TAG, "z=%5d x=%5d,  y=%5d", z, x, y);
    }

    lastValidX = x;
    lastValidY = y;
  }
  else
  {
    // dummy read to power down touch controller.
    tp_spi_xchg_xpt(0x90);
  }

  gpio_set_level(CONFIG_LVGL_DRV_TP_SPI_CS, 1);

  data->point.x = lastValidX;
  data->point.y = lastValidY;
  data->state = valid == false ? LV_INDEV_STATE_REL : LV_INDEV_STATE_PR;

  return false;
}
