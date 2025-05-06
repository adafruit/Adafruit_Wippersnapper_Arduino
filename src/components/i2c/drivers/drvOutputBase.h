/*!
 * @file drvOutputBase.h
 *
 * Base implementation for I2C output device drivers.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef DRV_OUTPUT_BASE_H
#define DRV_OUTPUT_BASE_H
#include "drvBase.h"

/**************************************************************************/
/*!
    @brief  Base class for I2C Output Drivers.
*/
/**************************************************************************/
class drvOutputBase : public drvBase {

public:
  /*******************************************************************************/
  /*!
      @brief    Instantiates an I2C output device.
      @param    i2c
                The I2C hardware interface, default is Wire.
      @param    address
                The I2C device's unique address.
      @param    mux_channel
                An optional channel number used to address a device on a I2C
     MUX.
      @param    driver_name
                The name of the driver.
  */
  /*******************************************************************************/
  drvOutputBase(TwoWire *i2c, uint16_t address, uint32_t mux_channel,
                const char *driver_name)
      : drvBase(i2c, address, mux_channel, driver_name) {
    // TODO
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an I2C output device.
  */
  /*******************************************************************************/
  virtual ~drvOutputBase() {}

protected:
  // TODO
};
#endif // DRV_OUTPUT_BASE_H