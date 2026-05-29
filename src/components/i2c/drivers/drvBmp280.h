/*!
 * @file drvBmp280.h
 *
 * Device driver for a BMP280 Pressure and Temperature sensor.
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

#ifndef DRV_BMP280_H
#define DRV_BMP280_H

#include "drvBase.h"
#include <Adafruit_BMP280.h>

#define SEALEVELPRESSURE_HPA (1013.25) ///< Default sea level pressure, in hPa

/*!
    @brief  Class that provides a sensor driver for the BMP280 temperature
            and pressure sensor.
*/
class drvBmp280 : public drvBase {

public:
  /*!
      @brief    Constructor for an BMP280 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvBmp280(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*!
      @brief    Destructor for an BMP280 sensor.
  */
  ~drvBmp280() { delete _bmp; }

  /*!
      @brief    Initializes the BMP280 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    _bmp = new Adafruit_BMP280(_i2c);
    // attempt to initialize BMP280
    if (!_bmp->begin(_address))
      return false;

    // attempt to get sensors
    _bmp_temp = _bmp->getTemperatureSensor();
    if (_bmp_temp == NULL)
      return false;
    _bmp_pressure = _bmp->getPressureSensor();
    if (_bmp_pressure == NULL)
      return false;

    return true;
  }

  /*!
      @brief    Configures the BMP280 sensor with default settings.
      @returns  True if configured successfully, False otherwise.
  */
  bool configureDefaults() override {
    _bmp->setSampling(Adafruit_BMP280::MODE_NORMAL,  /* Operating Mode. */
                      Adafruit_BMP280::SAMPLING_X2,  /* Temp. oversampling */
                      Adafruit_BMP280::SAMPLING_X16, /* Pressure oversampling */
                      Adafruit_BMP280::FILTER_X16,   /* Filtering. */
                      Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    return true;
  }

  /*!
      @brief    Applies the measurement mode setting to the driver.
      @param    mode
                The mode index from the broker (0=Sleep, 1=Forced, 2=Normal).
      @returns  True if applied successfully, False otherwise.
  */
  bool setMode(int32_t mode) override {
    switch (mode) {
    case 0:
      _mode = Adafruit_BMP280::MODE_SLEEP;
      break;
    case 1:
      _mode = Adafruit_BMP280::MODE_FORCED;
      break;
    case 2:
      _mode = Adafruit_BMP280::MODE_NORMAL;
      break;
    default:
      return false;
    }
    applySampling();
    return true;
  }

  /*!
      @brief    Applies the temperature oversampling setting to the driver.
      @param    temp_oversampling
                The temperature oversampling index from the broker
                (0=None, 1=1x, 2=2x, 3=4x, 4=8x, 5=16x).
      @returns  True if applied successfully, False otherwise.
  */
  bool setTempOversampling(int32_t temp_oversampling) override {
    switch (temp_oversampling) {
    case 0:
      _temp_sampling = Adafruit_BMP280::SAMPLING_NONE;
      break;
    case 1:
      _temp_sampling = Adafruit_BMP280::SAMPLING_X1;
      break;
    case 2:
      _temp_sampling = Adafruit_BMP280::SAMPLING_X2;
      break;
    case 3:
      _temp_sampling = Adafruit_BMP280::SAMPLING_X4;
      break;
    case 4:
      _temp_sampling = Adafruit_BMP280::SAMPLING_X8;
      break;
    case 5:
      _temp_sampling = Adafruit_BMP280::SAMPLING_X16;
      break;
    default:
      return false;
    }
    applySampling();
    return true;
  }

  /*!
      @brief    Applies the pressure oversampling setting to the driver.
      @param    pressure_oversampling
                The pressure oversampling index from the broker
                (0=None, 1=1x, 2=2x, 3=4x, 4=8x, 5=16x).
      @returns  True if applied successfully, False otherwise.
  */
  bool setPressureOversampling(int32_t pressure_oversampling) override {
    switch (pressure_oversampling) {
    case 0:
      _press_sampling = Adafruit_BMP280::SAMPLING_NONE;
      break;
    case 1:
      _press_sampling = Adafruit_BMP280::SAMPLING_X1;
      break;
    case 2:
      _press_sampling = Adafruit_BMP280::SAMPLING_X2;
      break;
    case 3:
      _press_sampling = Adafruit_BMP280::SAMPLING_X4;
      break;
    case 4:
      _press_sampling = Adafruit_BMP280::SAMPLING_X8;
      break;
    case 5:
      _press_sampling = Adafruit_BMP280::SAMPLING_X16;
      break;
    default:
      return false;
    }
    applySampling();
    return true;
  }

  /*!
      @brief    Applies the filter setting to the driver.
      @param    filter
                The filter index from the broker
                (0=Off, 1=2x, 2=4x, 3=8x, 4=16x).
      @returns  True if applied successfully, False otherwise.
  */
  bool setFilter(int32_t filter) override {
    switch (filter) {
    case 0:
      _filter = Adafruit_BMP280::FILTER_OFF;
      break;
    case 1:
      _filter = Adafruit_BMP280::FILTER_X2;
      break;
    case 2:
      _filter = Adafruit_BMP280::FILTER_X4;
      break;
    case 3:
      _filter = Adafruit_BMP280::FILTER_X8;
      break;
    case 4:
      _filter = Adafruit_BMP280::FILTER_X16;
      break;
    default:
      return false;
    }
    applySampling();
    return true;
  }

  /*!
      @brief    Applies the standby duration setting to the driver.
      @param    standby
                The standby duration index from the broker
                (0=1ms, 1=62.5ms, 2=125ms, 3=250ms, 4=500ms, 5=1000ms,
                6=2000ms, 7=4000ms).
      @returns  True if applied successfully, False otherwise.
  */
  bool setStandby(int32_t standby) override {
    switch (standby) {
    case 0:
      _duration = Adafruit_BMP280::STANDBY_MS_1;
      break;
    case 1:
      _duration = Adafruit_BMP280::STANDBY_MS_63;
      break;
    case 2:
      _duration = Adafruit_BMP280::STANDBY_MS_125;
      break;
    case 3:
      _duration = Adafruit_BMP280::STANDBY_MS_250;
      break;
    case 4:
      _duration = Adafruit_BMP280::STANDBY_MS_500;
      break;
    case 5:
      _duration = Adafruit_BMP280::STANDBY_MS_1000;
      break;
    case 6:
      _duration = Adafruit_BMP280::STANDBY_MS_2000;
      break;
    case 7:
      _duration = Adafruit_BMP280::STANDBY_MS_4000;
      break;
    default:
      return false;
    }
    applySampling();
    return true;
  }

  /*!
      @brief    Gets the BMP280's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    return _bmp_temp->getEvent(tempEvent);
  }

  /*!
      @brief    Reads a pressure sensor and converts
                the reading into the expected SI unit.
      @param    pressureEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  bool getEventPressure(sensors_event_t *pressureEvent) {
    return _bmp_pressure->getEvent(pressureEvent);
  }

  /*!
      @brief    Reads a the BMP280's altitude sensor into an event.
      @param    altitudeEvent
                Pointer to an adafruit sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  bool getEventAltitude(sensors_event_t *altitudeEvent) {
    altitudeEvent->altitude = _bmp->readAltitude(SEALEVELPRESSURE_HPA);
    return true;
  }

protected:
  /*!
      @brief    Re-applies all cached sampling settings to the sensor. The
                BMP280 library only accepts mode, oversampling, filter, and
                standby together in a single setSampling() call, so each
                set* method updates one cached member and then re-issues the
                full call from the cached state.
  */
  void applySampling() {
    _bmp->setSampling(_mode, _temp_sampling, _press_sampling, _filter,
                      _duration);
  }

  Adafruit_BMP280 *_bmp; ///< BMP280  object
  Adafruit_Sensor *_bmp_temp =
      NULL; ///< Ptr to an adafruit_sensor representing the temperature
  Adafruit_Sensor *_bmp_pressure =
      NULL; ///< Ptr to an adafruit_sensor representing the pressure
  Adafruit_Sensor *_bmp_humidity =
      NULL; ///< Ptr to an adafruit_sensor representing the humidity
  Adafruit_BMP280::sensor_mode _mode =
      Adafruit_BMP280::MODE_NORMAL; ///< Operating mode
  Adafruit_BMP280::sensor_sampling _temp_sampling =
      Adafruit_BMP280::SAMPLING_X16; ///< Temperature oversampling
  Adafruit_BMP280::sensor_sampling _press_sampling =
      Adafruit_BMP280::SAMPLING_X16; ///< Pressure oversampling
  Adafruit_BMP280::sensor_filter _filter =
      Adafruit_BMP280::FILTER_OFF; ///< Filter setting
  Adafruit_BMP280::standby_duration _duration =
      Adafruit_BMP280::STANDBY_MS_1; ///< Standby duration
};

#endif // drvBmp280