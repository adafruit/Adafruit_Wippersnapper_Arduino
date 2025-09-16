/*!
 * @file drvAs5600.h
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
#ifndef DRV_AS5600_H
#define DRV_AS5600_H

#include "drvBase.h"
#include <Adafruit_AS5600.h>

/*!
    @brief  Class that provides a driver interface for an AS5600 sensor.
*/
class drvAs5600 : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for the AS5600 sensor.
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
  drvAs5600(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    _as5600 = nullptr;
    _angle = 0;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an AS5600 sensor.
  */
  /*******************************************************************************/
  ~drvAs5600() {
    if (_as5600) {
      delete _as5600;
      _as5600 = nullptr;
    }
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the AS5600 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    _as5600 = new Adafruit_AS5600();
    if (!_as5600->begin((uint8_t)_address, _i2c)) {
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

  /*******************************************************************************/
  /*!
      @brief    Reads the Angle sensor.
      @returns  True if the sensor was read successfully, False otherwise.
  */
  /*******************************************************************************/
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
    WS_DEBUG_PRINT(" | Scaled: ");
    WS_DEBUG_PRINT(angle);

    // Check status conditions
    if (_as5600->isAGCminGainOverflow()) {
      WS_DEBUG_PRINTLN(" | MH: magnet too strong");
      return false;
    }
    if (_as5600->isAGCmaxGainOverflow()) {
      WS_DEBUG_PRINTLN(" | ML: magnet too weak");
      return false;
    }
    _angle = ((float)angle / 4095.0) * 360.0;
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
    if (!readSensor()) {
      return false;
    }
    rawEvent->data[0] = _angle;
    return true;
  }

  void ConfigureDefaultSensorTypes() override {
    _default_sensor_types_count = 1;
    _default_sensor_types[0] = wippersnapper_sensor_SensorType_SENSOR_TYPE_RAW;
  }

protected:
  float _angle;             ///< Current angle reading from the AS5600 sensor
  Adafruit_AS5600 *_as5600; ///< Pointer to AS5600 sensor object
};

#endif // DRV_AS5600_H
