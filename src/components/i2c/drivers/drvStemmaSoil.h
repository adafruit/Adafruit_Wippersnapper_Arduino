/*!
 * @file drvStemmaSoil.h
 *
 * Device driver for the STEMMA Soil Sensor
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Marcus Wu 2022
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef DRV_STEMMA_SOIL_H
#define DRV_STEMMA_SOIL_H

#include "drvBase.h"
#include <Adafruit_seesaw.h>

/*!
    @brief  Class that provides a driver interface for the STEMMA soil sensor.
*/
class drvStemmaSoil : public drvBase {
public:
  /*!
      @brief    Constructor for a STEMMA soil sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvStemmaSoil(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
                const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    _i2c = i2c;
    _address = sensorAddress;
    _i2c_mux_channel = mux_channel;
    strncpy(_name, driver_name, sizeof(_name) - 1);
    _name[sizeof(_name) - 1] = '\0';
    _seesaw = new Adafruit_seesaw(_i2c);
  }

  /*!
      @brief    Destructor for a STEMMA soil sensor.
  */
  ~drvStemmaSoil() { _seesaw = nullptr; }

  /*!
      @brief    Initializes the soil sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override { return _seesaw->begin(_address); }

  /*!
      @brief    Gets the sensor's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    tempEvent->temperature = _seesaw->getTemp();
    return true;
  }

  /*!
      @brief    Gets the sensor's current moisture sensor capacitance value.
      @param    rawEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  bool getEventRaw(sensors_event_t *rawEvent) {
    uint16_t touchData = _seesaw->touchRead(0);

    // seesaw->touchRead() will return 65535 on a read error. See more at
    // https://github.com/adafruit/Adafruit_Seesaw/blob/master/Adafruit_seesaw.cpp
    if (touchData == 65535) {
      rawEvent->data[0] = NAN;
    } else {
      // TODO: Update this should we add a capacitive moisture type to
      // adafruit_sensor
      rawEvent->data[0] = (float)touchData;
    }
    return true;
  }

  void ConfigureDefaultSensorTypes() override {
    _default_sensor_types_count = 3;
    _default_sensor_types[0] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE;
    _default_sensor_types[1] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE_FAHRENHEIT;
    _default_sensor_types[2] = wippersnapper_sensor_SensorType_SENSOR_TYPE_RAW;
  }

protected:
  Adafruit_seesaw *_seesaw = nullptr; ///< Seesaw object
};

#endif // DRV_STEMMA_SOIL_H