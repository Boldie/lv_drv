/**
 * @file disp_spi.h
 *
 */

#ifndef DISP_SPI_INT_H
#define DISP_SPI_INT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * Sends data to the display, used internally.
 */
void disp_spi_send_data(uint8_t * data, uint16_t length);

/**
 * Sends color to the display, used internally.
 */
void disp_spi_send_colors(uint8_t * data, uint16_t length);

/**
 * Exchanges data with the touch controller, used internally.
 */
uint16_t tp_spi_xchg_xpt(uint8_t writeData);

/**
 * Can be used to check if a SPI transfer is completed
 * and the bus is free, used internally.
 */
void disp_tp_spi_finished();

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*DISP_SPI_INT_H*/
