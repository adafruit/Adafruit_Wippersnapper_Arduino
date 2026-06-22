/*!
 * @file drvBmp5xx.h
 *
 * Device driver for a BMP5XX precision pressure sensor breakout.
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

#ifndef DRV_BMP5XX_H
#define DRV_BMP5XX_H

#include "drvBase.h"
#include <Adafruit_BMP5xx.h>

#define SEALEVELPRESSURE_HPA (1013.25) ///< Default sea level pressure, in hPa

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the BMP5XX temperature
            and pressure sensor.
*/
/**************************************************************************/
class drvBmp5xx : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an BMP5XX sensor.
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
  drvBmp5xx(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    _bmp5xx = nullptr;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an BMP5XX sensor.
  */
  /*******************************************************************************/
  ~drvBmp5xx() {
    if (_bmp5xx) {
      delete _bmp5xx;
      _bmp5xx = nullptr;
    }
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the BMP5XX sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    _bmp5xx = new Adafruit_BMP5xx();
    if (!_bmp5xx->begin(_address, _i2c)) {
      delete _bmp5xx;
      _bmp5xx = nullptr;
      return false;
    }
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Configures the BMP5XX sensor with default settings.
      @returns  True if configured successfully, False otherwise.
  */
  /*******************************************************************************/
  bool configureDefaults() override {
    return _bmp5xx->setTemperatureOversampling(BMP5XX_OVERSAMPLING_8X) &&
           _bmp5xx->setPressureOversampling(BMP5XX_OVERSAMPLING_16X) &&
           _bmp5xx->setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3) &&
           _bmp5xx->setOutputDataRate(BMP5XX_ODR_50_HZ) &&
           _bmp5xx->setPowerMode(BMP5XX_POWERMODE_NORMAL) &&
           _bmp5xx->enablePressure(true);
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
    bmp5xx_oversampling_t oversampling;
    if (!IndexToOversampling(temp_oversampling.value.int_value, oversampling)) {
      return false;
    }
    return _bmp5xx->setTemperatureOversampling(oversampling);
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
    bmp5xx_oversampling_t oversampling;
    if (!IndexToOversampling(pressure_oversampling.value.int_value,
                             oversampling)) {
      return false;
    }
    return _bmp5xx->setPressureOversampling(oversampling);
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the IIR filter coefficient setting to the driver.
      @param    iir_filter
                The IIR filter index from the broker
                (0=Bypass, 1=Coeff 1, 2=Coeff 3, 3=Coeff 7, 4=Coeff 15,
                5=Coeff 31, 6=Coeff 63, 7=Coeff 127).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setIirFilter(const ws_config_Value &iir_filter) override {
    if (iir_filter.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    bmp5xx_iir_filter_t coeff;
    switch (iir_filter.value.int_value) {
    case 0:
      coeff = BMP5XX_IIR_FILTER_BYPASS;
      break;
    case 1:
      coeff = BMP5XX_IIR_FILTER_COEFF_1;
      break;
    case 2:
      coeff = BMP5XX_IIR_FILTER_COEFF_3;
      break;
    case 3:
      coeff = BMP5XX_IIR_FILTER_COEFF_7;
      break;
    case 4:
      coeff = BMP5XX_IIR_FILTER_COEFF_15;
      break;
    case 5:
      coeff = BMP5XX_IIR_FILTER_COEFF_31;
      break;
    case 6:
      coeff = BMP5XX_IIR_FILTER_COEFF_63;
      break;
    case 7:
      coeff = BMP5XX_IIR_FILTER_COEFF_127;
      break;
    default:
      return false;
    }
    return _bmp5xx->setIIRFilterCoeff(coeff);
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the output data rate (ODR) setting to the driver. The
                ODR selects the sensor's sampling frequency, in Hz.
      @param    output_data_rate
                The output data rate index from the broker
                (0=50Hz, 1=45Hz, 2=40Hz, 3=35Hz, 4=30Hz, 5=25Hz, 6=20Hz,
                7=15Hz, 8=10Hz, 9=5Hz, 10=4Hz, 11=3Hz, 12=2Hz, 13=1Hz,
                14=0.5Hz, 15=0.25Hz, 16=0.125Hz).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setOutputDataRate(const ws_config_Value &output_data_rate) override {
    if (output_data_rate.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    static const bmp5xx_odr_t kOdrs[] = {
        BMP5XX_ODR_50_HZ,    BMP5XX_ODR_45_HZ,   BMP5XX_ODR_40_HZ,
        BMP5XX_ODR_35_HZ,    BMP5XX_ODR_30_HZ,   BMP5XX_ODR_25_HZ,
        BMP5XX_ODR_20_HZ,    BMP5XX_ODR_15_HZ,   BMP5XX_ODR_10_HZ,
        BMP5XX_ODR_05_HZ,    BMP5XX_ODR_04_HZ,   BMP5XX_ODR_03_HZ,
        BMP5XX_ODR_02_HZ,    BMP5XX_ODR_01_HZ,   BMP5XX_ODR_0_5_HZ,
        BMP5XX_ODR_0_250_HZ, BMP5XX_ODR_0_125_HZ};
    int32_t val = output_data_rate.value.int_value;
    if (val < 0 || val >= (int32_t)(sizeof(kOdrs) / sizeof(kOdrs[0]))) {
      return false;
    }
    return _bmp5xx->setOutputDataRate(kOdrs[val]);
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the power mode setting to the driver.
      @param    power_mode
                The power mode index from the broker
                (0=Standby, 1=Normal, 2=Forced, 3=Continuous, 4=Deep Standby).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setPowerMode(const ws_config_Value &power_mode) override {
    if (power_mode.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    bmp5xx_powermode_t mode;
    switch (power_mode.value.int_value) {
    case 0:
      mode = BMP5XX_POWERMODE_STANDBY;
      break;
    case 1:
      mode = BMP5XX_POWERMODE_NORMAL;
      break;
    case 2:
      mode = BMP5XX_POWERMODE_FORCED;
      break;
    case 3:
      mode = BMP5XX_POWERMODE_CONTINUOUS;
      break;
    case 4:
      mode = BMP5XX_POWERMODE_DEEP_STANDBY;
      break;
    default:
      return false;
    }
    return _bmp5xx->setPowerMode(mode);
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the BMP5XX's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    if (!_bmp5xx->performReading()) {
      return false;
    }
    tempEvent->temperature = _bmp5xx->temperature;
    return true;
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
    if (!_bmp5xx->performReading()) {
      return false;
    }
    pressureEvent->pressure = _bmp5xx->pressure;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads a the BMP5XX's altitude sensor into an event.
      @param    altitudeEvent
                Pointer to an adafruit sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAltitude(sensors_event_t *altitudeEvent) {
    if (!_bmp5xx->performReading()) {
      return false;
    }
    altitudeEvent->altitude = _bmp5xx->readAltitude(SEALEVELPRESSURE_HPA);
    return true;
  }

protected:
  /*******************************************************************************/
  /*!
      @brief    Maps a broker oversampling index to a BMP5xx oversampling enum.
                Temperature and pressure share the same option list.
      @param    idx
                The oversampling index from the broker
                (0=1x, 1=2x, 2=4x, 3=8x, 4=16x, 5=32x, 6=64x, 7=128x).
      @param    out
                Output oversampling enum, set on success.
      @returns  True if the index was valid, False otherwise.
  */
  /*******************************************************************************/
  static bool IndexToOversampling(int32_t idx, bmp5xx_oversampling_t &out) {
    static const bmp5xx_oversampling_t kOversampling[] = {
        BMP5XX_OVERSAMPLING_1X,  BMP5XX_OVERSAMPLING_2X,
        BMP5XX_OVERSAMPLING_4X,  BMP5XX_OVERSAMPLING_8X,
        BMP5XX_OVERSAMPLING_16X, BMP5XX_OVERSAMPLING_32X,
        BMP5XX_OVERSAMPLING_64X, BMP5XX_OVERSAMPLING_128X};
    if (idx < 0 ||
        idx >= (int32_t)(sizeof(kOversampling) / sizeof(kOversampling[0]))) {
      return false;
    }
    out = kOversampling[idx];
    return true;
  }

  Adafruit_BMP5xx *_bmp5xx; ///< BMP5xx object
};

#endif // DRV_BMP5XX_H
