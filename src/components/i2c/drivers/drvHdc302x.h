/*!
 * @file drvHdc302x.h
 *
 * Device driver for the HDC302X humidity and temperature sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) 2026 Tyeth Gundry for Adafruit Industries
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
  bool begin() override {
    // attempt to initialize the HDC302X using the I2C interface
    _hdc302x = new Adafruit_HDC302x();
    if (!_hdc302x->begin(_address, _i2c))
      return false;

    // discard first reading (It returned -45c for me once); also serves as a
    // comms sanity check
    _hdc302x->readTemperatureHumidityOnDemand(_temp, _humidity,
                                              TRIGGERMODE_LP0);
    return true;

    // Note: measurement mode/rate is intentionally NOT exposed as a setting -
    // auto-mode is disabled and the driver does a single-shot read per the
    // user's chosen polling period (like most sensors).
  }

  /*******************************************************************************/
  /*!
      @brief    Configures the HDC302X sensor with default settings.
      @returns  True if configured successfully, False otherwise.
  */
  /*******************************************************************************/
  bool configureDefaults() override {
    // use on-demand single-shot reads rather than continuous auto-mode
    _hdc302x->setAutoMode(EXIT_AUTO_MODE);
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the on-chip heater power setting to the driver. The
                heater clears condensation from the sensor.
      @param    heater
                The heater index from the broker
                (0=Off, 1=Quarter power, 2=Half power, 3=Full power).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setHeater(const ws_config_Value &heater) override {
    if (heater.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    HDC302x_HeaterPower power;
    switch (heater.value.int_value) {
    case 0:
      power = HEATER_OFF;
      break;
    case 1:
      power = HEATER_QUARTER_POWER;
      break;
    case 2:
      power = HEATER_HALF_POWER;
      break;
    case 3:
      power = HEATER_FULL_POWER;
      break;
    default:
      return false;
    }
    return _hdc302x->heaterEnable(power);
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the HDC302X's temperature and humidity in one on-demand
                conversion so both metrics reflect the same sample. Serves the
                cached sample if the last read was under one second ago.
      @returns  True if a valid sample is cached, False otherwise.
  */
  /*******************************************************************************/
  bool ReadSensorData() override {
    if (HasBeenReadInLastSecond())
      return _have_data;

    uint16_t status = _hdc302x->readStatus();
    if (status & 0x0010) {
      WS_DEBUG_PRINTLN("Device Reset Detected");
      return false;
    }

    if (status & 0x0001) {
      WS_DEBUG_PRINTLN(
          "Checksum Verification Fail (incorrect checksum received)");
      return false;
    }

    if (!_hdc302x->readTemperatureHumidityOnDemand(_temp, _humidity,
                                                   TRIGGERMODE_LP0)) {
      WS_DEBUG_PRINTLN("Failed to read temperature and humidity.");
      return _have_data;
    }
    _last_read = millis();
    _have_data = true;
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

protected:
  Adafruit_HDC302x *_hdc302x = nullptr; ///< Pointer to an HDC302X object
  double _temp = NAN;     ///< Holds data for the HDC302X's temperature sensor
  double _humidity = NAN; ///< Holds data for the HDC302X's humidity sensor
};
#endif // DRV_HDC302X_H
