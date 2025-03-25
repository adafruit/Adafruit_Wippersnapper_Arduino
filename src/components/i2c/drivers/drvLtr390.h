/*!
 * @file drvLtr390.h
 *
 * Device driver for the LTR390 light sensor.
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
#ifndef DRV_LTR390_H
#define DRV_LTR390_H

#include "drvBase.h"
#include <Adafruit_LTR390.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a LTR390 sensor.
*/
/**************************************************************************/
class drvLtr390 : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a LTR390 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  /*******************************************************************************/
  drvLtr390(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an LTR390 sensor.
  */
  /*******************************************************************************/
  ~drvLtr390() { delete _ltr390; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the LTR390 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    _ltr390 = new Adafruit_LTR390();
    // Attempt to initialize LTR390
    if (!_ltr390->begin(_i2c))
      return false;

    // Configure LTR390 sensor
    // Note: This driver uses the default configuration from
    // https://github.com/adafruit/Adafruit_LTR390/blob/master/examples/ltr390_test/ltr390_test.ino
    _ltr390->setMode(LTR390_MODE_UVS);
    _ltr390->setGain(LTR390_GAIN_3);
    _ltr390->setResolution(LTR390_RESOLUTION_16BIT);
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Performs a light sensor read using the Adafruit
                Unified Sensor API.
      @param    lightEvent
                Light sensor reading, in lux.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventLight(sensors_event_t *lightEvent) {
    if (_ltr390->getMode() != LTR390_MODE_ALS) {
      _ltr390->setMode(LTR390_MODE_ALS);
      delay(110);
    }

    if (!_ltr390->newDataAvailable())
      return false;

    lightEvent->light = _ltr390->readALS();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the LTR390's UV value into an event.
      @param    rawEvent
                Pointer to an adafruit sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRaw(sensors_event_t *rawEvent) {
    if (_ltr390->getMode() != LTR390_MODE_UVS) {
      _ltr390->setMode(LTR390_MODE_UVS);
      delay(110);
    }

    if (!_ltr390->newDataAvailable())
      return false;

    rawEvent->data[0] = (float)_ltr390->readUVS();
    return true;
  }

  void ConfigureDefaultSensorTypes() override {
    _default_sensor_types_count = 2;
    _default_sensor_types[0] = wippersnapper_sensor_SensorType_SENSOR_TYPE_LIGHT;
    _default_sensor_types[1] = wippersnapper_sensor_SensorType_SENSOR_TYPE_RAW;
  }

protected:
  Adafruit_LTR390 *_ltr390; ///< Pointer to LTR390 light sensor object
};

#endif // drvLtr390