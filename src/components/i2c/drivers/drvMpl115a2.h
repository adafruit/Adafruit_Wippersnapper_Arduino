/*!
 * @file drvMpl115a2.h
 *
 * Device driver for a MPL115A2 pressure sensor breakout.
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

#ifndef DRV_MPL115A2_H
#define DRV_MPL115A2_H

#include "drvBase.h"
#include <Adafruit_MPL115A2.h>

/*!
    @brief  Class that provides a sensor driver for the MPL115A2 temperature
            and pressure sensor.
*/
class drvMpl115a2 : public drvBase {

public:
  /*!
      @brief    Constructor for an MPL115A2 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvMpl115a2(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
              const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*!
      @brief    Destructor for an MPL115A2 sensor.
  */
  ~drvMpl115a2() { delete _mpl115a2; }

  /*!
      @brief    Initializes the MPL115A2 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    _mpl115a2 = new Adafruit_MPL115A2();
    return _mpl115a2->begin(_address, _i2c);
  }

  /*!
      @brief    Gets the MPL115A2's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    tempEvent->temperature = _mpl115a2->getTemperature();
    return true;
  }

  /*!
      @brief    Reads a pressure sensor and converts
                the reading into the expected SI unit (hPa).
      @param    pressureEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  bool getEventPressure(sensors_event_t *pressureEvent) {
    pressureEvent->pressure = _mpl115a2->getPressure() * 10;
    return true;
  }

  void ConfigureDefaultSensorTypes() override {
    _default_sensor_types_count = 3;
    _default_sensor_types[0] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE;
    _default_sensor_types[1] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE_FAHRENHEIT;
    _default_sensor_types[2] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_PRESSURE;
  }

protected:
  Adafruit_MPL115A2 *_mpl115a2; ///< MPL115A2  object
};

#endif // drvMpl115a2