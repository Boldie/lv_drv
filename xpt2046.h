/**
 * @file XPT2046.h
 *
 */

#ifndef XPT2046_H
#define XPT2046_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "lvgl/lvgl.h"

/**
 * Initialize the XPT2046
 */
void xpt2046_init(void);

/**
 * Get the current position and state of the touchpad
 *
 * @param data store the read data here
 * @return false: because no more data to be read
 */
bool xpt2046_read(lv_indev_drv_t * drv, lv_indev_data_t * data);

/**
 * Sets the read function to raw mode which returns the raw and
 * uncorrected points from the touch screen.
 *
 * @param on: Set to true to turn this mode on.
 */
void xpt2046_setRawMode( bool on );

/**
 * This struct holds a calibration point.
 */
typedef struct{
  /// Point in touch screen coordinates from drivers raw mode.
  lv_point_t raw;
  /// Point from screen coordinates the user touched.
  lv_point_t screen;
} CalibPoint;

/**
 * Calculate a new calibration from the given points. It needs at least
 * 3 points from the user, more are possible.
 *
 * @param points Array of point pairs with the length of count.
 * @param count The number of points within the points array.
 * @return True if calibration was successful, otherwise false.
 */
bool xpt2046_calculateCalibration( CalibPoint* points, uint8_t count );

/**
 * Retrieve if the calibration is valid.
 */
bool xpt2046_hasValidCalbiration();

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* XPT2046_H */
