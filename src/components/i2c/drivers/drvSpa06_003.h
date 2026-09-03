/*!
 * @file drvSpa06_003.h
 *
 * Device driver for an SPA06-003 Pressure and Temperature sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2026 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef DRV_SPA06_003_H
#define DRV_SPA06_003_H

#include "drvBase.h"
#include <Adafruit_SPA06_003.h>

#define SPA06_003_TEMP_MIN -40.0 ///< Minimum valid temperature reading
#define SPA06_003_TEMP_MAX 85.0  ///< Maximum valid temperature reading
#define SPA06_003_PRESSURE_MIN                                                 \
  300.0 ///< Minimum valid pressure (9km above sea level)
#define SPA06_003_PRESSURE_MAX                                                 \
  1100.0 ///< Maximum valid pressure (500m below sea level)

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the SPA06-003 PT sensor.
*/
/**************************************************************************/
class drvSpa06_003 : public drvBase {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an SPA06-003 sensor.
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
  drvSpa06_003(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
               const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an SPA06-003 sensor.
  */
  /*******************************************************************************/
  ~drvSpa06_003() { delete _spa06_003; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the SPA06-003 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    _spa06_003 = new Adafruit_SPA06_003();
    // attempt to initialize SPA06-003
    if (!_spa06_003->begin(_address, _i2c))
      return false;

    _spa06_003_temp = _spa06_003->getTemperatureSensor();
    _spa06_003_pressure = _spa06_003->getPressureSensor();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Configures the SPA06-003 sensor with default settings.
      @returns  True if configured successfully, False otherwise.
  */
  /*******************************************************************************/
  bool configureDefaults() override {
    // Retain the library's power-up defaults for oversampling and rate.
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the temperature oversampling setting to the driver.
      @param    temp_oversampling
                The temperature oversampling index from the broker
                (0=1x, 1=2x, 2=4x, 3=8x, 4=16x, 5=32x, 6=64x, 7=128x).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setTempOversampling(const ws_config_Value &temp_oversampling) override {
    if (temp_oversampling.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    spa06_003_oversample_t oversampling;
    switch (temp_oversampling.value.int_value) {
    case 0:
      oversampling = SPA06_003_OVERSAMPLE_1;
      break;
    case 1:
      oversampling = SPA06_003_OVERSAMPLE_2;
      break;
    case 2:
      oversampling = SPA06_003_OVERSAMPLE_4;
      break;
    case 3:
      oversampling = SPA06_003_OVERSAMPLE_8;
      break;
    case 4:
      oversampling = SPA06_003_OVERSAMPLE_16;
      break;
    case 5:
      oversampling = SPA06_003_OVERSAMPLE_32;
      break;
    case 6:
      oversampling = SPA06_003_OVERSAMPLE_64;
      break;
    case 7:
      oversampling = SPA06_003_OVERSAMPLE_128;
      break;
    default:
      return false;
    }
    return _spa06_003->setTemperatureOversampling(oversampling);
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the pressure oversampling setting to the driver.
      @param    pressure_oversampling
                The pressure oversampling index from the broker
                (0=1x, 1=2x, 2=4x, 3=8x, 4=16x, 5=32x, 6=64x, 7=128x).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setPressureOversampling(
      const ws_config_Value &pressure_oversampling) override {
    if (pressure_oversampling.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    spa06_003_oversample_t oversampling;
    switch (pressure_oversampling.value.int_value) {
    case 0:
      oversampling = SPA06_003_OVERSAMPLE_1;
      break;
    case 1:
      oversampling = SPA06_003_OVERSAMPLE_2;
      break;
    case 2:
      oversampling = SPA06_003_OVERSAMPLE_4;
      break;
    case 3:
      oversampling = SPA06_003_OVERSAMPLE_8;
      break;
    case 4:
      oversampling = SPA06_003_OVERSAMPLE_16;
      break;
    case 5:
      oversampling = SPA06_003_OVERSAMPLE_32;
      break;
    case 6:
      oversampling = SPA06_003_OVERSAMPLE_64;
      break;
    case 7:
      oversampling = SPA06_003_OVERSAMPLE_128;
      break;
    default:
      return false;
    }
    return _spa06_003->setPressureOversampling(oversampling);
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the output (measurement) data rate setting to both the
                pressure and temperature measurement rates.
      @param    output_data_rate
                The rate index from the broker
                (0=1, 1=2, 2=4, 3=8, 4=16, 5=32, 6=64, 7=128, 8=25/16, 9=25/8,
                10=25/4, 11=25/2, 12=25, 13=50, 14=100, 15=200 measurements/s).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setOutputDataRate(const ws_config_Value &output_data_rate) override {
    if (output_data_rate.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    spa06_003_rate_t r;
    switch (output_data_rate.value.int_value) {
    case 0:
      r = SPA06_003_RATE_1;
      break;
    case 1:
      r = SPA06_003_RATE_2;
      break;
    case 2:
      r = SPA06_003_RATE_4;
      break;
    case 3:
      r = SPA06_003_RATE_8;
      break;
    case 4:
      r = SPA06_003_RATE_16;
      break;
    case 5:
      r = SPA06_003_RATE_32;
      break;
    case 6:
      r = SPA06_003_RATE_64;
      break;
    case 7:
      r = SPA06_003_RATE_128;
      break;
    case 8:
      r = SPA06_003_RATE_25_16;
      break;
    case 9:
      r = SPA06_003_RATE_25_8;
      break;
    case 10:
      r = SPA06_003_RATE_25_4;
      break;
    case 11:
      r = SPA06_003_RATE_25_2;
      break;
    case 12:
      r = SPA06_003_RATE_25;
      break;
    case 13:
      r = SPA06_003_RATE_50;
      break;
    case 14:
      r = SPA06_003_RATE_100;
      break;
    case 15:
      r = SPA06_003_RATE_200;
      break;
    default:
      return false;
    }
    return _spa06_003->setPressureMeasureRate(r) &&
           _spa06_003->setTemperatureMeasureRate(r);
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SPA06-003's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    bool success = false;
    if (_spa06_003_temp == NULL)
      return false;
    success = _spa06_003_temp->getEvent(tempEvent);
    if (tempEvent->temperature > SPA06_003_TEMP_MAX ||
        tempEvent->temperature < SPA06_003_TEMP_MIN) {
      success = false;
    }
    return success;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads a pressure sensor and converts
                the reading into the expected SI unit.
      @param    pressureEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventPressure(sensors_event_t *pressureEvent) {
    bool success = false;
    if (_spa06_003_pressure == NULL || _spa06_003->isPresDataReady() == false) {
      return false;
    }
    success = _spa06_003_pressure->getEvent(pressureEvent);
    if (pressureEvent->pressure < SPA06_003_PRESSURE_MIN ||
        pressureEvent->pressure > SPA06_003_PRESSURE_MAX) {
      success = false;
    }
    return success;
  }

protected:
  Adafruit_SPA06_003 *_spa06_003 = nullptr; ///< SPA06-003 object
  Adafruit_Sensor *_spa06_003_temp =
      nullptr; ///< Ptr to an adafruit_sensor representing the temperature
  Adafruit_Sensor *_spa06_003_pressure =
      nullptr; ///< Ptr to an adafruit_sensor representing the pressure
};

#endif // DRV_SPA06_003_H
