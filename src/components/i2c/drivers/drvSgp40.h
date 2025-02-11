/*!
 * @file drvSgp40.h
 *
 * Device driver for the SGP40 VOC/gas sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2023 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef DRV_SGP40_H
#define DRV_SGP40_H

#include "drvBase.h"
#include <Adafruit_SGP40.h>
#include <Wire.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the SGP40 sensor.
*/
/**************************************************************************/
class drvSgp40 : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a SGP40 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  /*******************************************************************************/
  drvSgp40(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
           const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    _i2c = i2c;
    _address = sensorAddress;
    _i2c_mux_channel = mux_channel;
    strncpy(_name, driver_name, sizeof(_name) - 1);
    _name[sizeof(_name) - 1] = '\0';
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the SGP40 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    _sgp40 = new Adafruit_SGP40();
    if (!_sgp40->begin(_i2c)) {
      return false;
    }

    // TODO: update to use setCalibration() and pass in temp/humidity

    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the sensor's current raw unprocessed value.
      @param    rawEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRaw(sensors_event_t *rawEvent) {
    rawEvent->data[0] = (float)_sgp40->measureRaw();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SGP40's current VOC reading.
      @param    vocIndexEvent
                  Adafruit Sensor event for VOC Index (1-500, 100 is normal)
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventVOCIndex(sensors_event_t *vocIndexEvent) {
    vocIndexEvent->voc_index = (float)_sgp40->measureVocIndex();
    return true;
  }

protected:
  Adafruit_SGP40 *_sgp40; ///< SEN5X driver object
};

#endif // WipperSnapper_I2C_Driver_SEN5X
