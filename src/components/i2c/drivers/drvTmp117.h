/*!
 * @file drvTmp117.h
 *
 * Device driver for the TMP117 Temperature sensor.
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
#ifndef DRV_TMP117_H
#define DRV_TMP117_H

#include "drvBase.h"
#include <Adafruit_TMP117.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a TMP117 sensor.
*/
/**************************************************************************/
class drvTmp117 : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a TMP117 sensor.
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
  drvTmp117(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an TMP117 sensor.
  */
  /*******************************************************************************/
  ~drvTmp117() {
    // Called when a TMP117 component is deleted.
    delete _tmp117;
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the TMP117 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    _tmp117 = new Adafruit_TMP117();
    return _tmp117->begin((uint8_t)_address, _i2c);
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the TMP117's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    return _tmp117->getEvent(tempEvent);
  }

  void ConfigureDefaultSensorTypes() override {
    _default_sensor_types_count = 2;
    _default_sensor_types[0] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE;
    _default_sensor_types[1] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE_FAHRENHEIT;
  }

protected:
  Adafruit_TMP117 *_tmp117; ///< Pointer to TMP117 temperature sensor object
};

#endif // drvTmp117