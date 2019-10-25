/**
 * @file disp_spi.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "disp_spi.h"

#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "ili9341.h"
#include "freertos/task.h"
#include "lvgl/lvgl.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void IRAM_ATTR spi_ready (spi_transaction_t *trans);

/**********************
 *  STATIC VARIABLES
 **********************/
static spi_device_handle_t spi_disp;
static volatile bool spi_trans_in_progress = false;
static volatile bool spi_color_sent = false;

static spi_device_handle_t spi_touch;

static const char* TAG = "spi_drv";

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void lvgl_drv_spi_init(void)
{
  ESP_LOGI(TAG, "SPI init");
  esp_err_t ret;

  spi_bus_config_t buscfg = {
      .miso_io_num = CONFIG_LVGL_DRV_DISP_TP_SPI_MISO,
      .mosi_io_num = CONFIG_LVGL_DRV_DISP_TP_SPI_MOSI,
      .sclk_io_num = CONFIG_LVGL_DRV_DISP_TP_SPI_CLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = DISP_BUF_SIZE * 2
    };

  //Initialize the SPI bus
  ret = spi_bus_initialize(VSPI_HOST, &buscfg, 1);
  assert(ret == ESP_OK);

  ////////////////////////////////////////////
  // Display configuration section
  ///////////////////////////////////////////
  spi_device_interface_config_t devcfg_disp = {
      .clock_speed_hz = 40 * 1000 * 1000,
      .mode = 0,
      .spics_io_num = CONFIG_LVGL_DRV_DISP_SPI_CS,              //CS pin
      .queue_size = 1,
      .post_cb = spi_ready
      };

  //Attach the LCD to the SPI bus
  ret = spi_bus_add_device(VSPI_HOST, &devcfg_disp, &spi_disp);
  assert(ret == ESP_OK);

  ////////////////////////////////////////////
  // Touch configuration section
  ///////////////////////////////////////////
  spi_device_interface_config_t devcfg_touch = {
      .command_bits = 8,
      .clock_speed_hz = 2 * 1000 * 1000,
      .mode = 0,
      .spics_io_num = -1,  //TP_SPI_CS,              //CS pin
      .queue_size = 1,
      .pre_cb = NULL,
      .post_cb = NULL
    };

  //Attach the Touch to the SPI bus
  ret = spi_bus_add_device(VSPI_HOST, &devcfg_touch, &spi_touch);
  assert(ret == ESP_OK);
}

static spi_transaction_t transaction;

void disp_spi_send_data(uint8_t * data, uint16_t length)
{
  if (length == 0)
    return;

  while (spi_trans_in_progress)
    ;

  memset( &transaction, 0, sizeof(transaction) );
  transaction.length = length * 8;  // transaction length is in bits
  transaction.tx_buffer = data;

  spi_trans_in_progress = true;
  spi_color_sent = false;
  ESP_ERROR_CHECK( spi_device_queue_trans(spi_disp, &transaction, portMAX_DELAY) );
}

void disp_spi_send_colors(uint8_t * data, uint16_t length)
{
  if (length == 0)
    return;

  while (spi_trans_in_progress)
    ;

  memset( &transaction, 0, sizeof(transaction) );
  transaction.length = length * 8;  // transaction length is in bits
  transaction.tx_buffer = data;

  spi_trans_in_progress = true;
  spi_color_sent = true;
  ESP_ERROR_CHECK( spi_device_queue_trans(spi_disp, &transaction, portMAX_DELAY) );
}


uint16_t tp_spi_xchg_xpt(uint8_t writeData)
{
  while(spi_trans_in_progress)
    ;

  uint8_t data_recv[2] = {0,0};
  uint16_t data_send = 0;

  memset( &transaction, 0, sizeof(transaction) );
  transaction.cmd = writeData;
  transaction.length = 16; // length is in bits
  transaction.tx_buffer = &data_send;
  transaction.rx_buffer = &data_recv;

  ESP_ERROR_CHECK( spi_device_queue_trans(spi_touch, &transaction, portMAX_DELAY) );

  spi_transaction_t * rt;
  ESP_ERROR_CHECK( spi_device_get_trans_result(spi_touch, &rt, portMAX_DELAY) );

  return (uint16_t)(data_recv[0]) << 8 | (uint16_t)(data_recv[1]);
}

void disp_tp_spi_finished()
{
  while(spi_trans_in_progress)
    ;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void IRAM_ATTR spi_ready (spi_transaction_t *trans)
{
  if(spi_color_sent) {
    spi_color_sent = false;
    spi_trans_in_progress = false;
    lv_disp_t * disp = lv_refr_get_disp_refreshing();
    lv_disp_flush_ready(&disp->driver);
  }
  else {
    spi_trans_in_progress = false;
  }
}
