/*!
 * @file drvDps310.h
 *
 * Device driver the DPS310 barometric pressure sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2021-2025 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef DRV_DPS310_H
#define DRV_DPS310_H
#include "drvBase.h"
#include <Adafruit_DPS310.h>

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the DPS310 barometric
            pressure sensor.
*/
/**************************************************************************/
class drvDps310 : public drvBase {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a DPS310 sensor.
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
  drvDps310(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    _i2c = i2c;
    _address = sensorAddress;
    _i2c_mux_channel = mux_channel;
    strncpy(_name, driver_name, sizeof(_name) - 1);
    _name[sizeof(_name) - 1] = '\0';
    _last_read = 0;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an DPS310 sensor.
  */
  /*******************************************************************************/
  ~drvDps310() { delete _dps310; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the DPS310 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    // initialize DPS310
    _dps310 = new Adafruit_DPS310();
    if (!_dps310->begin_I2C((uint8_t)_address, _i2c)) {
      return false;
    }

    // init OK, perform sensor configuration
    _dps310->configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
    _dps310->configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    _dps_temp = _dps310->getTemperatureSensor();
    if (_dps_temp == NULL) {
      return false;
    }
    _dps_pressure = _dps310->getPressureSensor();
    if (_dps_pressure == NULL) {
      return false;
    }
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the DPS310's temperature and pressure.
      @returns  True if the measurements were read successfully, False
     otherwise.
  */
  /*******************************************************************************/
  bool alreadyRecentlyRead() {
    return (_last_read != 0 && (millis() - _last_read < 1000));
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the DPS310's temperature and pressure.
      @returns  True if the measurements were read successfully, False
     otherwise.
  */
  /*******************************************************************************/
  bool ReadMeasurements() {
    if (alreadyRecentlyRead())
      return true;

    while (!_dps310->temperatureAvailable() || !_dps310->pressureAvailable())
      return false;

    if (!_dps310->getEvents(&_temp_event, &_pressure_event))
      return false;

    _last_read = millis();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the DPS310's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    if (!ReadMeasurements()) {
      return false;
    }
    tempEvent->temperature = _temp_event.temperature;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the DPS310's pressure reading.
      @param    pressureEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the pressure was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventPressure(sensors_event_t *pressureEvent) {
    if (!ReadMeasurements()) {
      return false;
    }
    pressureEvent->pressure = _pressure_event.pressure;
    return true;
  }

  void ConfigureDefaultSensorTypes() override {
    _default_sensor_types_count = 3;
    _default_sensor_types[0] = wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE;
    _default_sensor_types[1] = wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE_FAHRENHEIT;
    _default_sensor_types[2] = wippersnapper_sensor_SensorType_SENSOR_TYPE_PRESSURE;
  }

protected:
  sensors_event_t _temp_event = {
      0}; ///< DPS310 sensor event for temperature readings
  sensors_event_t
      _pressure_event;      ///< DPS310 sensor event for pressure readings
  ulong _last_read;         ///< Last time the sensor was read
  Adafruit_DPS310 *_dps310; ///< DPS310 driver object
  Adafruit_Sensor *_dps_temp =
      NULL; ///< Holds data for the DPS310's temperature sensor
  Adafruit_Sensor *_dps_pressure =
      NULL; ///< Holds data for the DPS310's pressure sensor
};

#endif // drvDps310