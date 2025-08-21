/*!
 * @file WipperSnapper_I2C_Driver_AS5600.h
 *
 * Device driver for the AS5600 Magnetic Angle sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2024 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef WipperSnapper_I2C_Driver_AS5600_H
#define WipperSnapper_I2C_Driver_AS5600_H

#include <Adafruit_AS5600.h>

#include "WipperSnapper_I2C_Driver.h"
#include "Wippersnapper.h"

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a AS5600 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_AS5600 : public WipperSnapper_I2C_Driver {
 public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for the AS5600 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_AS5600(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
    _as5600 = nullptr;
    _angle = 0;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an AS5600 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_AS5600() { delete _as5600; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the AS5600 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _as5600 = new Adafruit_AS5600();
    if (!_as5600->begin((uint8_t)_sensorAddress, _i2c)) {
      WS_DEBUG_PRINTLN("Failed to find AS5600 chip");
      return false;
    }

    if (!configureSensor()) {
      WS_DEBUG_PRINTLN("Failed to configure AS5600 sensor");
      return false;
    }
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Configures the AS5600 sensor.
      @returns  True if the sensor was configured successfully, False otherwise.
  */
  /*******************************************************************************/
  bool configureSensor() {
    return _as5600->enableWatchdog(false) &&  
           // Normal (high) power mode
           _as5600->setPowerMode(AS5600_POWER_MODE_NOM) &&
           // No Hysteresis
           _as5600->setHysteresis(AS5600_HYSTERESIS_OFF) &&
           // analog output (0-VCC for 0-360 degrees)
           _as5600->setOutputStage(AS5600_OUTPUT_STAGE_ANALOG_FULL) &&
           // setup filters
           _as5600->setSlowFilter(AS5600_SLOW_FILTER_16X) &&
           _as5600->setFastFilterThresh(AS5600_FAST_FILTER_THRESH_SLOW_ONLY) &&
           // Reset position settings to defaults
           _as5600->setZPosition(0) && _as5600->setMPosition(4095) &&
           _as5600->setMaxAngle(4095);
  }

  bool readSensor() {
    if (!_as5600->isMagnetDetected()) {
      WS_DEBUG_PRINTLN("Magnet not detected!");
      return false;
    }

    // Continuously read and display angle values
    uint16_t rawAngle = _as5600->getRawAngle();
    uint16_t angle = _as5600->getAngle();

    WS_DEBUG_PRINT("AS5600 Raw: ");
    WS_DEBUG_PRINT(rawAngle);
    WS_DEBUG_PRINT(" (0x");
    WS_DEBUG_PRINT(rawAngle, HEX);
    WS_DEBUG_PRINT(") | Scaled: ");
    WS_DEBUG_PRINT(angle);
    WS_DEBUG_PRINT(" (0x");
    WS_DEBUG_PRINT(angle, HEX);
    WS_DEBUG_PRINT(")");

    // Check status conditions
    if (_as5600->isAGCminGainOverflow()) {
      WS_DEBUG_PRINTLN(" | MH: magnet too strong");
      return false;
    }
    if (_as5600->isAGCmaxGainOverflow()) {
      WS_DEBUG_PRINTLN(" | ML: magnet too weak");
      return false;
    }
    _angle = (float)angle;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the Angle sensor with short wait for data.
      @param    rawEvent
                Angle sensor reading
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRaw(sensors_event_t *rawEvent) {
    ulong start = millis();
    if (!readSensor()) {
      return false;
    }
    rawEvent->data[0] = _angle;
    return true;
  }

 protected:
  float _angle;              ///< Current angle reading from the AS5600 sensor
  Adafruit_AS5600 *_as5600;  ///< Pointer to AS5600 sensor object
};

#endif  // WipperSnapper_I2C_Driver_AS5600