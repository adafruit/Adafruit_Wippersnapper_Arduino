/*!
 * @file WipperSnapper_I2C_Driver_STEMMA_Soil_Sensor.h
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

#ifndef WipperSnapper_I2C_Driver_STEMMA_Soil_Sensor_H
#define WipperSnapper_I2C_Driver_STEMMA_Soil_Sensor_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_seesaw.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the STEMMA soil sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_STEMMA_Soil_Sensor
    : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a STEMMA soil sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_STEMMA_Soil_Sensor(TwoWire *i2c,
                                              uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
    _seesaw = new Adafruit_seesaw(_i2c);
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for a STEMMA soil sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_STEMMA_Soil_Sensor() { delete _seesaw; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the soil sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() { return _seesaw->begin(_sensorAddress); }

  /*******************************************************************************/
  /*!
      @brief    Gets the sensor's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    tempEvent->temperature = _seesaw->getTemp();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the sensor's current moisture sensor capacitance value.
      @param    rawEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRaw(sensors_event_t *rawEvent) {
    uint16_t touchData = _seesaw->touchRead(0);

    // seesaw->touchRead() will return 65535 on a read error
    // see
    // https://github.com/adafruit/Adafruit_Seesaw/blob/master/Adafruit_seesaw.cpp
    if (touchData == 65535) {
      return false;
    }

    // TODO: Update this should we add a capacitive moisture type to
    // adafruit_sensor
    rawEvent->data[0] = (float)touchData;
    return true;
  }

protected:
  Adafruit_seesaw *_seesaw; ///< Seesaw object
};

#endif // WipperSnapper_I2C_Driver_STEMMA_Soil_Sensor_H