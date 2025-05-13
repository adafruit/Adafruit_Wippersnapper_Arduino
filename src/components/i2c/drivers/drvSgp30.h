/*!
 * @file drvSgp30.h
 *
 * Device driver for the SGP30 VOC/gas sensor.
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

#ifndef DRV_SGP30_H
#define DRV_SGP30_H

#include "drvBase.h"
#include <Adafruit_SGP30.h>
#include <Wire.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the SGP30 sensor.
*/
/**************************************************************************/
class drvSgp30 : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a SGP30 sensor.
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
  drvSgp30(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
           const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the SGP30 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    _sgp30 = new Adafruit_SGP30();
    if (!_sgp30->begin(_i2c)) {
      return false;
    }

    // TODO: update to use setCalibration() and pass in temp/humidity

    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the sensor's current equivalent/estimated CO2 value.
      @param    co2Event
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventECO2(sensors_event_t *co2Event) {
    bool result = _sgp30->IAQmeasure();
    if (result) {
      co2Event->eCO2 = (float)_sgp30->eCO2;
    }
    return result;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SGP30's current VOC reading.
      @param    vocIndexEvent
                  Adafruit Sensor event for VOC Index (1-500, 100 is normal)
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventVOCIndex(sensors_event_t *vocIndexEvent) {
    bool result = _sgp30->IAQmeasure();
    if (result) {
      vocIndexEvent->voc_index = (float)_sgp30->TVOC;
    }
    return result;
  }

  void ConfigureDefaultSensorTypes() override {
    _default_sensor_types_count = 2;
    _default_sensor_types[0] = wippersnapper_sensor_SensorType_SENSOR_TYPE_ECO2;
    _default_sensor_types[1] =
        wippersnapper_sensor_SensorType_SENSOR_TYPE_VOC_INDEX;
  }

protected:
  Adafruit_SGP30 *_sgp30; ///< SGP30 driver object
};

#endif // WipperSnapper_I2C_Driver_SGP30
