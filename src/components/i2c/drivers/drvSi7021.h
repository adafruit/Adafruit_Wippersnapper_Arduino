/*!
 * @file drvSi7021.h
 *
 * Device driver for the SI7021 Temperature, and Humidity sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2022 for Adafruit Industries.
 * Copyright (c) Marni Brewster 2022 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef DRV_SI7021_H
#define DRV_SI7021_H

#include "drvBase.h"
#include <Adafruit_Si7021.h>
#include <Wire.h>

/*!
    @brief  Class that provides a driver interface for the SI7021 sensor.
*/
class drvSi7021 : public drvBase {

public:
  /*!
      @brief    Constructor for a SI7021 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvSi7021(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*!
      @brief    Destructor for an SI7021 sensor.
  */
  ~drvSi7021() {
    // Called when a Si7021 component is deleted.
    delete _si7021;
  }

  /*!
      @brief    Initializes the SI7021 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    _si7021 = new Adafruit_Si7021(_i2c);
    return _si7021->begin();
  }

  /*!
      @brief    Gets the SI7021's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    // check if sensor is enabled and data is available
    tempEvent->temperature = _si7021->readTemperature();
    return true;
  }

  /*!
      @brief    Gets the SI7021's current relative humidity reading.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    // check if sensor is enabled and data is available
    humidEvent->relative_humidity = _si7021->readHumidity();
    return true;
  }

protected:
  Adafruit_Si7021 *_si7021; ///< SI7021 driver object
};

#endif // drvSi7021