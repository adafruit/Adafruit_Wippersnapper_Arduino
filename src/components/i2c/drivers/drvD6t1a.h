/*!
 * @file drvD6t1a.h
 *
 * Device driver for the OMRON D6T-1A Non-contact Thermal sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2025 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef DRV_D6T1A_H
#define DRV_D6T1A_H
#include <OmronD6T.h>

#include "drvBase.h"

/*!
    @brief  Class that provides a sensor driver for the D6T1A temperature
   sensor.
*/
class drvD6t1a : public drvBase {
 public:
  /*!
      @brief    Constructor for a D6T1A sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvD6t1a(TwoWire* i2c, uint16_t sensorAddress, uint32_t mux_channel,
           const char* driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    _i2c = i2c;
    _address = sensorAddress;
    _i2c_mux_channel = mux_channel;
    strncpy(_name, driver_name, sizeof(_name) - 1);
    _name[sizeof(_name) - 1] = '\0';
    _deviceTemp = NAN;
    _objectTemp = NAN;
    _lastRead = 0;
    _d6t1a = nullptr;
  }

  /*!
      @brief    Destructor for a D6T1A sensor.
  */
  ~drvD6t1a() {
    delete _d6t1a;
  }

  /*!
      @brief    Initializes the D6T1A sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    _d6t1a = new OmronD6T(OmronD6T::D6T_1A, _i2c);
    if (!_d6t1a->begin(_address))
      return false;
    return true;
  }

  /*!
      @brief    Checks if sensor was read within last 200ms.
      @returns  True if the sensor was recently read, False otherwise.
  */
  bool HasBeenReadInLast200ms() {
    return _lastRead != 0 && (millis() - _lastRead < 200);
  }

  /*!
      @brief    Reads the sensor.
      @returns  True if the sensor was read successfully, False otherwise.
  */
  bool ReadSensorData() {
    if (HasBeenReadInLast200ms())
      return true;

    _d6t1a->read();
    _deviceTemp = (float)_d6t1a->ambientTempC();
    _objectTemp = (float)_d6t1a->objectTempC(0, 0);
    _lastRead = millis();
    return true;
  }

  /*!
      @brief    Gets the D6T1A's current ambient temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
     otherwise.
  */
  bool getEventAmbientTemp(sensors_event_t* tempEvent) {
    if (ReadSensorData() && !isnan(_deviceTemp)) {
      tempEvent->temperature = _deviceTemp;
      return true;
    }
    return false;
  }

  /*!
      @brief    Gets the D6T1A's object temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
     otherwise.
  */
  bool getEventObjectTemp(sensors_event_t* tempEvent) {
    if (ReadSensorData() && !isnan(_objectTemp)) {
      tempEvent->temperature = _objectTemp;
      return true;
    }
    return false;
  }

  void ConfigureDefaultSensorTypes() override {
    _default_sensor_types_count = 2;
    _default_sensor_types[0] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE;
    _default_sensor_types[1] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_OBJECT_TEMPERATURE;
  }

 protected:
  float _deviceTemp;  ///< Device temperature in Celsius
  float _objectTemp;  ///< Object temperature in Celsius
  uint32_t _lastRead; ///< Last time the sensor was read in milliseconds
  OmronD6T* _d6t1a;   ///< D6T1A object
};

#endif // DRV_D6T1A_H