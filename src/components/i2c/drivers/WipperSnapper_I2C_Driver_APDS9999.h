/*!
 * @file WipperSnapper_I2C_Driver_APDS9999.h
 *
 * Device driver for the APDS-9999 proximity, lux light, and color sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry for Adafruit Industries 2026.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef WipperSnapper_I2C_Driver_APDS9999_H
#define WipperSnapper_I2C_Driver_APDS9999_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_APDS9999.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for an APDS-9999 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_APDS9999 : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an APDS-9999 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_APDS9999(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an APDS-9999 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_APDS9999() { delete _apds9999; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the APDS-9999 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _apds9999 = new Adafruit_APDS9999();
    if (!_apds9999->begin((uint8_t)_sensorAddress, _i2c))
      return false;

    // Enable light sensor and proximity sensor
    _apds9999->enableLightSensor(true);
    _apds9999->enableProximitySensor(true);

    // Enable RGB mode for color data
    _apds9999->setRGBMode(true);

    // Explicitly set light sensor defaults
    _apds9999->setLightGain(APDS9999_LIGHT_GAIN_3X);
    _apds9999->setLightResolution(APDS9999_LIGHT_RES_18BIT);
    _apds9999->setLightMeasRate(APDS9999_LIGHT_RATE_100MS);

    // Explicitly set proximity sensor defaults
    _apds9999->setProxResolution(APDS9999_PROX_RES_11BIT);
    _apds9999->setProxMeasRate(APDS9999_PROX_RATE_100MS);

    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the APDS-9999 sensor data.
      @returns  True if the sensor was read successfully, False otherwise.
  */
  /*******************************************************************************/
  bool ReadSensorData() {
    // Don't read sensor more than once per second
    if (_lastRead != 0 && (millis() - _lastRead < 1000))
      return true;

    uint32_t r, g, b, ir;
    if (!_apds9999->getRGBIRData(&r, &g, &b, &ir))
      return false;

    _cachedLight.light = _apds9999->calculateLux(g);

    uint16_t prox;
    if (!_apds9999->readProximity(&prox))
      return false;

    _cachedProximity.data[0] = (float)prox;
    _lastRead = millis();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the APDS-9999's current light reading in lux.
      @param    lightEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the light reading was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventLight(sensors_event_t *lightEvent) {
    if (!ReadSensorData())
      return false;

    *lightEvent = _cachedLight;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the APDS-9999's current proximity reading.
      @param    proximityEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the proximity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventProximity(sensors_event_t *proximityEvent) {
    if (!ReadSensorData())
      return false;

    *proximityEvent = _cachedProximity;
    return true;
  }

protected:
  Adafruit_APDS9999 *_apds9999 = nullptr; ///< APDS-9999 driver object
  unsigned long _lastRead = 0;            ///< Last sensor read time
  sensors_event_t _cachedLight = {0};     ///< Cached light reading
  sensors_event_t _cachedProximity = {0}; ///< Cached proximity reading
};

#endif // WipperSnapper_I2C_Driver_APDS9999_H
