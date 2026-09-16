/*!
 * @file drvEns160.h
 *
 * Device driver for a ENS160 MOX Gas Sensor.
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

#ifndef DRV_ENS160_H
#define DRV_ENS160_H

#include "drvBase.h"
#include <ScioSense_ENS160.h>

#define SEALEVELPRESSURE_HPA (1013.25) ///< Default sea level pressure, in hPa

/*!
    @brief  Class that provides a sensor driver for the ENS160 temperature
            and humidity sensor.
*/
class drvEns160 : public drvBase {

public:
  /*!
      @brief    Constructor for an ENS160 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvEns160(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*!
      @brief    Destructor for an ENS160 sensor.
  */
  ~drvEns160() { delete _ens160; }

  /*!
      @brief    Initializes the ENS160 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    _ens160 = new ScioSense_ENS160((TwoWire *)_i2c, (uint8_t)_address);

    // attempt to initialize ENS160
    if (!_ens160->begin())
      return false;

    return true;
  }

  /*!
      @brief    Configures the ENS160 sensor with default settings.
      @returns  True if configured successfully, False otherwise.
  */
  bool configureDefaults() override {
    // Set the mode to standard
    return _ens160->setMode(ENS160_OPMODE_STD);
  }

  /*!
      @brief    Applies the operating mode setting to the driver.
      @param    mode
                The mode index from the broker
                (0=Deep Sleep, 1=Idle, 2=Standard, 3=Low Power).
      @returns  True if applied successfully, False otherwise.
  */
  bool setMode(const ws_config_Value &mode) override {
    if (mode.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    int32_t val = mode.value.int_value;
    uint8_t opmode;
    switch (val) {
    case 0:
      opmode = ENS160_OPMODE_DEP_SLEEP;
      break;
    case 1:
      opmode = ENS160_OPMODE_IDLE;
      break;
    case 2:
      opmode = ENS160_OPMODE_STD;
      break;
    case 3:
      opmode = ENS160_OPMODE_LP;
      break;
    default:
      opmode = ENS160_OPMODE_STD;
      break;
    }
    return _ens160->setMode(opmode);
  }

  /*!
      @brief    Checks the ENS160's DEVICE_STATUS register (0x20) directly, as
                the library exposes neither the validity flag nor the new-data
                bit (its available() is the init flag). Ready only when there
                is no error (STATER), the VALIDITY flag is 0 - i.e. not in the
                3 minute warm-up (1), the first-hour initial start-up (2) or
                "no valid output" (3), datasheet 10 / Table 10 - and NEWDAT is
                set. Reading 0x20 does not clear NEWDAT.
      @returns  True if a new, valid measurement is available, False otherwise.
  */
  bool IsSensorReady() override {
    _i2c->beginTransmission((uint8_t)_address);
    _i2c->write((uint8_t)ENS160_REG_DATA_STATUS);
    if (_i2c->endTransmission(false) != 0)
      return false;
    if (_i2c->requestFrom((uint8_t)_address, (uint8_t)1) != 1)
      return false;
    uint8_t status = _i2c->read();
    if (status & 0x40) // STATER: device error
      return false;
    if (((status >> 2) & 0x03) != 0) // VALIDITY: warm-up / start-up / invalid
      return false;
    return (status & ENS160_DATA_STATUS_NEWDAT) != 0;
  }

  /*!
      @brief    Reads the ENS160's measurement (eCO2, TVOC and AQI) in one
                transaction so all metrics reflect the same sample. Called
                only after IsSensorReady() saw NEWDAT, so the non-blocking
                form is used (measure(true) spins until new data).
      @returns  True if the reading succeeded, False otherwise.
  */
  bool ReadSensorData() override { return _ens160->measure(false); }

  /*!
      @brief    Reads the ENS160's eCO2 sensor into an event.
      @param    eco2Event
                Pointer to an adafruit sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  bool getEventECO2(sensors_event_t *eco2Event) {
    if (!AttemptRead())
      return false;
    eco2Event->eCO2 = (float)_ens160->geteCO2();
    return true;
  }

  /*!
      @brief    Reads the ENS160's TVOC sensor into an event.
      @param    tvocEvent
                Pointer to an adafruit sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  bool getEventTVOC(sensors_event_t *tvocEvent) {
    if (!AttemptRead())
      return false;
    tvocEvent->tvoc = (float)_ens160->getTVOC();
    return true;
  }

  /*!
      @brief    Reads the ENS160's AQI value into an event.
      @param    rawEvent
                Pointer to an adafruit sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  bool getEventRaw(sensors_event_t *rawEvent) {
    if (!AttemptRead())
      return false;
    rawEvent->data[0] = (float)_ens160->getAQI();
    return true;
  }

protected:
  ScioSense_ENS160 *_ens160; ///< ENS160 object
};

#endif // drvEns160