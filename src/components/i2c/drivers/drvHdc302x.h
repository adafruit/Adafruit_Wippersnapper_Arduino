/*!
 * @file drvHdc302x.h
 *
 * Device driver for the HDC302X humidity and temperature sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) 2025 Tyeth Gundry for Adafruit Industries
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef DRV_HDC302X_H
#define DRV_HDC302X_H

#include "drvBase.h"
#include <Adafruit_HDC302x.h>

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the HDC302X humidity and
            temperature sensor. This implementation uses the 1 Hz data rate.
*/
/**************************************************************************/
class drvHdc302x : public drvBase {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an HDC302X sensor.
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
  drvHdc302x(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
             const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an HDC302X sensor.
  */
  /*******************************************************************************/
  ~drvHdc302x() { delete _hdc302x; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the HDC302X sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.

  */
  /*******************************************************************************/
  bool begin() {
    // attempt to initialize the HDC302X using the I2C interface
    _hdc302x = new Adafruit_HDC302x();
    if (!_hdc302x->begin(_address, _i2c))
      return false;

    // set the HDC302X's data rate to 1 Hz lowest noise
    _hdc302x->setAutoMode(EXIT_AUTO_MODE);
    // discard first reading (It returned -45c for me once)
    _hdc302x->readTemperatureHumidityOnDemand(_temp, _humidity,
                                              TRIGGERMODE_LP0);
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the HDC302X's temperature and humidity data.
      @returns  True if the data was read successfully, False otherwise.
  */
  /*******************************************************************************/
  bool ReadSensorData() {
    uint16_t status = _hdc302x->readStatus();
    if (status & 0x0010) {
      WS_DEBUG_PRINTLN(F("Device Reset Detected"));
      return false;
    }

    if (status & 0x0001) {
      WS_DEBUG_PRINTLN(
          F("Checksum Verification Fail (incorrect checksum received)"));
      return false;
    }

    if (!_hdc302x->readTemperatureHumidityOnDemand(_temp, _humidity,
                                                   TRIGGERMODE_LP0)) {
      WS_DEBUG_PRINTLN(F("Failed to read temperature and humidity."));
      return false;
    }
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the HDC302X's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    if (ReadSensorData() == false)
      return false;
    tempEvent->temperature = _temp;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the HDC302X's current humidity.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    if (ReadSensorData() == false)
      return false;
    humidEvent->relative_humidity = _humidity;
    return true;
  }

  void ConfigureDefaultSensorTypes() override {
    _default_sensor_types_count = 3;
    _default_sensor_types[0] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE;
    _default_sensor_types[1] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE_FAHRENHEIT;
    _default_sensor_types[2] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_RELATIVE_HUMIDITY;
  }

protected:
  Adafruit_HDC302x *_hdc302x; ///< Pointer to an HDC302X object
  double _temp = 0.0;     ///< Holds data for the HDC302X's temperature sensor
  double _humidity = 0.0; ///< Holds data for the HDC302X's humidity sensor
};
#endif // DRV_HDC302X_H