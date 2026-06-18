/*!
 * @file drvHtu31d.h
 *
 * Device driver for the HTU31D humidity and temperature sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) 2023 Tyeth Gundry for Adafruit Industries
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef DRV_HTU31D_H
#define DRV_HTU31D_H

#include "drvBase.h"
#include <Adafruit_HTU31D.h>

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the HTU31D humidity and
            temperature sensor.
*/
/**************************************************************************/
class drvHtu31d : public drvBase {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an HTU31D sensor.
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
  drvHtu31d(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an HTU31D sensor.
  */
  /*******************************************************************************/
  ~drvHtu31d() { delete _htu31d; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the HTU31D sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    // attempt to initialize the HTU31D using the I2C interface
    _htu31d = new Adafruit_HTU31D();
    return _htu31d->begin(_address, _i2c);

    // POTENTIAL CUSTOM SETTINGS (not yet exposed via the v2 properties API):
    //  - On-chip heater: enableHeater(bool) drives off condensation / aids the
    //    fast RH recovery test; users may want it toggleable.
    //  - Measurement resolution: the HTU31D supports selectable temperature and
    //    humidity oversampling (OSR) trading conversion time for resolution.
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the HTU31D's current temperature, in degrees Celsius.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    return _htu31d->getEvent(nullptr, tempEvent);
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the HTU31D's current relative humidity, in percent.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    return _htu31d->getEvent(humidEvent, nullptr);
  }

protected:
  Adafruit_HTU31D *_htu31d; ///< Pointer to an HTU31D object
};

#endif // DRV_HTU31D_H
