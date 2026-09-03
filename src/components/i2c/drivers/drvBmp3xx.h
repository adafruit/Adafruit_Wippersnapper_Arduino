/*!
 * @file drvBmp3xx.h
 *
 * Device driver for a BMP3XX precision pressure sensor breakout.
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

#ifndef DRV_BMP3XX_H
#define DRV_BMP3XX_H

#include "drvBase.h"
#include <Adafruit_BMP3XX.h>

#define SEALEVELPRESSURE_HPA (1013.25) ///< Default sea level pressure, in hPa

/*!
    @brief  Class that provides a sensor driver for the BMP3XX temperature
            and pressure sensor.
*/
class drvBmp3xx : public drvBase {
public:
  /*!
      @brief    Constructor for an BMP3XX sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvBmp3xx(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*!
      @brief    Destructor for an BMP3XX sensor.
  */
  ~drvBmp3xx() { delete _bmp3xx; }

  /*!
      @brief    Initializes the BMP3XX sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    _bmp3xx = new Adafruit_BMP3XX();
    // attempt to initialize BMP3XX
    if (!_bmp3xx->begin_I2C(_address, _i2c))
      return false;

    return true;
  }

  /*!
      @brief    Configures the BMP3XX sensor with default settings.
      @returns  True if configured successfully, False otherwise.
  */
  bool configureDefaults() override {
    if (!_bmp3xx->setTemperatureOversampling(BMP3_OVERSAMPLING_8X))
      return false;
    if (!_bmp3xx->setPressureOversampling(BMP3_OVERSAMPLING_4X))
      return false;
    if (!_bmp3xx->setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3))
      return false;
    if (!_bmp3xx->setOutputDataRate(BMP3_ODR_50_HZ))
      return false;
    return true;
  }

  /*!
      @brief    Applies the temperature oversampling setting to the driver.
      @param    temp_oversampling
                The temperature oversampling index from the broker
                (0=None, 1=2x, 2=4x, 3=8x, 4=16x, 5=32x).
      @returns  True if applied successfully, False otherwise.
  */
  bool setTempOversampling(const ws_config_Value &temp_oversampling) override {
    if (temp_oversampling.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    int32_t val = temp_oversampling.value.int_value;
    uint8_t oversampling;
    switch (val) {
    case 0:
      oversampling = BMP3_NO_OVERSAMPLING;
      break;
    case 1:
      oversampling = BMP3_OVERSAMPLING_2X;
      break;
    case 2:
      oversampling = BMP3_OVERSAMPLING_4X;
      break;
    case 3:
      oversampling = BMP3_OVERSAMPLING_8X;
      break;
    case 4:
      oversampling = BMP3_OVERSAMPLING_16X;
      break;
    case 5:
      oversampling = BMP3_OVERSAMPLING_32X;
      break;
    default:
      return false;
    }
    return _bmp3xx->setTemperatureOversampling(oversampling);
  }

  /*!
      @brief    Applies the pressure oversampling setting to the driver.
      @param    pressure_oversampling
                The pressure oversampling index from the broker
                (0=None, 1=2x, 2=4x, 3=8x, 4=16x, 5=32x).
      @returns  True if applied successfully, False otherwise.
  */
  bool setPressureOversampling(
      const ws_config_Value &pressure_oversampling) override {
    if (pressure_oversampling.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    int32_t val = pressure_oversampling.value.int_value;
    uint8_t oversampling;
    switch (val) {
    case 0:
      oversampling = BMP3_NO_OVERSAMPLING;
      break;
    case 1:
      oversampling = BMP3_OVERSAMPLING_2X;
      break;
    case 2:
      oversampling = BMP3_OVERSAMPLING_4X;
      break;
    case 3:
      oversampling = BMP3_OVERSAMPLING_8X;
      break;
    case 4:
      oversampling = BMP3_OVERSAMPLING_16X;
      break;
    case 5:
      oversampling = BMP3_OVERSAMPLING_32X;
      break;
    default:
      return false;
    }
    return _bmp3xx->setPressureOversampling(oversampling);
  }

  /*!
      @brief    Applies the IIR filter coefficient setting to the driver.
      @param    iir_filter
                The IIR filter index from the broker
                (0=Off, 1=Coeff 1, 2=Coeff 3, 3=Coeff 7, 4=Coeff 15,
                5=Coeff 31, 6=Coeff 63, 7=Coeff 127).
      @returns  True if applied successfully, False otherwise.
  */
  bool setIirFilter(const ws_config_Value &iir_filter) override {
    if (iir_filter.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    int32_t val = iir_filter.value.int_value;
    uint8_t coeff;
    switch (val) {
    case 0:
      coeff = BMP3_IIR_FILTER_DISABLE;
      break;
    case 1:
      coeff = BMP3_IIR_FILTER_COEFF_1;
      break;
    case 2:
      coeff = BMP3_IIR_FILTER_COEFF_3;
      break;
    case 3:
      coeff = BMP3_IIR_FILTER_COEFF_7;
      break;
    case 4:
      coeff = BMP3_IIR_FILTER_COEFF_15;
      break;
    case 5:
      coeff = BMP3_IIR_FILTER_COEFF_31;
      break;
    case 6:
      coeff = BMP3_IIR_FILTER_COEFF_63;
      break;
    case 7:
      coeff = BMP3_IIR_FILTER_COEFF_127;
      break;
    default:
      return false;
    }
    return _bmp3xx->setIIRFilterCoeff(coeff);
  }

  /*!
      @brief    Applies the output data rate (ODR) setting to the driver. The
                ODR selects the sensor's sampling frequency, in Hz.
      @param    output_data_rate
                The output data rate index from the broker
                (0=50Hz, 1=25Hz, 2=12.5Hz, 3=6.25Hz, 4=3.1Hz, 5=1.5Hz,
                6=0.78Hz, 7=0.39Hz, 8=0.2Hz).
      @returns  True if applied successfully, False otherwise.
  */
  bool setOutputDataRate(const ws_config_Value &output_data_rate) override {
    if (output_data_rate.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    int32_t val = output_data_rate.value.int_value;
    uint8_t odr;
    switch (val) {
    case 0:
      odr = BMP3_ODR_50_HZ;
      break;
    case 1:
      odr = BMP3_ODR_25_HZ;
      break;
    case 2:
      odr = BMP3_ODR_12_5_HZ;
      break;
    case 3:
      odr = BMP3_ODR_6_25_HZ;
      break;
    case 4:
      odr = BMP3_ODR_3_1_HZ;
      break;
    case 5:
      odr = BMP3_ODR_1_5_HZ;
      break;
    case 6:
      odr = BMP3_ODR_0_78_HZ;
      break;
    case 7:
      odr = BMP3_ODR_0_39_HZ;
      break;
    case 8:
      odr = BMP3_ODR_0_2_HZ;
      break;
    default:
      return false;
    }
    return _bmp3xx->setOutputDataRate(odr);
  }

  /*!
      @brief    Gets the BMP3XX's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    if (!_bmp3xx->performReading())
      return false;
    tempEvent->temperature = _bmp3xx->temperature;
    return true;
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
    if (!_bmp3xx->performReading())
      return false;
    pressureEvent->pressure = _bmp3xx->pressure / 100.0F;
    return true;
  }

  /*!
      @brief    Reads a the BMP3XX's altitude sensor into an event.
      @param    altitudeEvent
                Pointer to an adafruit sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  bool getEventAltitude(sensors_event_t *altitudeEvent) {
    if (!_bmp3xx->performReading())
      return false;
    altitudeEvent->altitude = _bmp3xx->readAltitude(SEALEVELPRESSURE_HPA);
    return true;
  }

protected:
  Adafruit_BMP3XX *_bmp3xx; ///< BMP3XX  object
};
#endif // drvBmp3xx