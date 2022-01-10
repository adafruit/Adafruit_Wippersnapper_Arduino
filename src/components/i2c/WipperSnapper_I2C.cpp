/*!
 * @file WipperSnapper_I2C.cpp
 *
 * This component initiates I2C operations
 * using the Arduino generic TwoWire driver.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2021 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "WipperSnapper_I2C.h"

/***************************************************************************************************************/
/*!
    @brief    Creates a new WipperSnapper I2C component.
    @param    msgInitRequest
              The I2C initialization request message.
*/
/***************************************************************************************************************/
WipperSnapper_Component_I2C::WipperSnapper_Component_I2C(
    wippersnapper_i2c_v1_I2CBusInitRequest *msgInitRequest) {
  WS_DEBUG_PRINTLN("EXEC: New I2C Port ");
  WS_DEBUG_PRINT("\tPort #: ");
  WS_DEBUG_PRINTLN(msgInitRequest->i2c_port_number);
  WS_DEBUG_PRINT("\tSDA Pin: ");
  WS_DEBUG_PRINTLN(msgInitRequest->i2c_pin_sda);
  WS_DEBUG_PRINT("\tSCL Pin: ");
  WS_DEBUG_PRINTLN(msgInitRequest->i2c_pin_scl);
  WS_DEBUG_PRINT("\tFrequency (Hz): ");
  WS_DEBUG_PRINTLN(msgInitRequest->i2c_frequency);

// Invert Feather ESP32-S2 pin power for I2C
#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) ||                               \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT)
  pinMode(PIN_I2C_POWER_INVERTED, OUTPUT);
  digitalWrite(PIN_I2C_POWER_INVERTED, LOW);
#endif

  // Enable pullups on SCL, SDA
  pinMode(msgInitRequest->i2c_pin_scl, INPUT_PULLUP);
  pinMode(msgInitRequest->i2c_pin_sda, INPUT_PULLUP);
  delay(150);

  // Is SDA or SCL stuck low?
  if ((digitalRead(msgInitRequest->i2c_pin_scl) == 0) ||
      (digitalRead(msgInitRequest->i2c_pin_sda) == 0)) {
    _busStatusResponse =
        wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_ERROR_PULLUPS;
    _isInit = false;
  } else {
    // Reset state of SCL/SDA pins
    pinMode(msgInitRequest->i2c_pin_scl, INPUT);
    pinMode(msgInitRequest->i2c_pin_sda, INPUT);

// Initialize I2C bus
#if defined(ARDUINO_ARCH_ESP32)
    _i2c = new TwoWire(msgInitRequest->i2c_port_number);
    if (!_i2c->begin(msgInitRequest->i2c_pin_sda,
                     msgInitRequest->i2c_pin_scl)) {
      _isInit = false; // if the peripheral was configured incorrectly
    } else {
      _isInit = true; // if the peripheral was configured incorrectly
    }
    _i2c->setClock(msgInitRequest->i2c_frequency);
#elif defined(ARDUINO_ARCH_ESP8266)
    _i2c = new TwoWire();
    _i2c->begin(msgInitRequest->i2c_pin_sda, msgInitRequest->i2c_pin_scl);
    _i2c->setClock(msgInitRequest->i2c_frequency);
    _isInit = true;
#else
    // SAMD
    _i2c = new TwoWire(&PERIPH_WIRE, msgInitRequest->i2c_pin_sda,
                       msgInitRequest->i2c_pin_scl);
    _i2c->begin();
    _isInit = true;
#endif

    // set i2c obj. properties
    _portNum = msgInitRequest->i2c_port_number;
    _busStatusResponse = wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_SUCCESS;
  }
}

/*************************************************************/
/*!
    @brief    Destructor for a WipperSnapper I2C component.
*/
/*************************************************************/
WipperSnapper_Component_I2C::~WipperSnapper_Component_I2C() {
  _portNum = 100; // Invalid = 100
  _isInit = false;
}

/*****************************************************/
/*!
    @brief    Returns if i2c port is initialized.
    @returns  True if initialized, False otherwise.
*/
/*****************************************************/
bool WipperSnapper_Component_I2C::isInitialized() { return _isInit; }

/*****************************************************/
/*!
    @brief    Returns the state of the I2C bus.
    @returns  wippersnapper_i2c_v1_BusResponse.
*/
/*****************************************************/
wippersnapper_i2c_v1_BusResponse WipperSnapper_Component_I2C::getBusStatus() {
  return _busStatusResponse;
}

/************************************************************************/
/*!
    @brief    Scans all I2C addresses on the bus between 0x08 and 0x7F
              inclusive and returns an array of the devices found.
    @returns  wippersnapper_i2c_v1_I2CBusScanResponse
*/
/************************************************************************/
wippersnapper_i2c_v1_I2CBusScanResponse
WipperSnapper_Component_I2C::scanAddresses() {
  uint8_t endTransmissionRC;
  uint16_t address;
  wippersnapper_i2c_v1_I2CBusScanResponse scanResp =
      wippersnapper_i2c_v1_I2CBusScanResponse_init_zero;

#ifndef ARDUINO_ARCH_ESP32
  // Set I2C WDT timeout to catch I2C hangs, SAMD-specific
  WS.enableWDT(I2C_TIMEOUT_MS);
  WS.feedWDT();
#endif

  // Scan all I2C addresses between 0x08 and 0x7F inclusive and return a list of
  // those that respond.
  WS_DEBUG_PRINT("EXEC: I2C Scan");
  for (address = 0x08; address < 0x7F; address++) {
    _i2c->beginTransmission(address);
    endTransmissionRC = _i2c->endTransmission();

#if defined(ARDUINO_ARCH_ESP32)
    // Check endTransmission()'s return code (Arduino-ESP32 ONLY)
    // https://github.com/espressif/arduino-esp32/blob/master/libraries/Wire/src/Wire.cpp
    if (endTransmissionRC == 5) {
      WS_DEBUG_PRINTLN("ESP_ERR_TIMEOUT: I2C Bus Busy");
      scanResp.bus_response =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_ERROR_HANG;
      // NOTE: ESP-IDF appears to handle this "behind the scenes" by
      // resetting/clearing the bus. The user should be prompted to
      // perform a bus scan again.
      break;
    } else if (endTransmissionRC == 7) {
      WS_DEBUG_PRINT("I2C_ESP_ERR: SDA/SCL shorted, requests queued: ");
      WS_DEBUG_PRINTLN(endTransmissionRC);
      break;
    }
#endif

    // Found device!
    if (endTransmissionRC == 0) {
      WS_DEBUG_PRINT("Found I2C Device at 0x");
      WS_DEBUG_PRINTLN(address);
      scanResp.addresses_found[scanResp.addresses_found_count] =
          (uint32_t)address;
      scanResp.addresses_found_count++;
    }
  }

#ifndef ARDUINO_ARCH_ESP32
  // re-enable WipperSnapper SAMD WDT global timeout
  WS.enableWDT(WS_WDT_TIMEOUT);
  WS.feedWDT();
#endif

  WS_DEBUG_PRINT("I2C Devices Found: ")
  WS_DEBUG_PRINTLN(scanResp.addresses_found_count);

  scanResp.bus_response = wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_SUCCESS;

  return scanResp;
}

/*******************************************************************************/
/*!
    @brief    Initializes I2C device driver.
    @param    msgDeviceInitReq
              A decoded I2CDevice initialization request message.
    @returns True if I2C device is initialized and attached, False otherwise.
*/
/*******************************************************************************/
bool WipperSnapper_Component_I2C::initI2CDevice(
    wippersnapper_i2c_v1_I2CDeviceInitRequest *msgDeviceInitReq) {
  WS_DEBUG_PRINTLN("Attempting to initialize an I2C device...");

  uint16_t i2cAddress = (uint16_t)msgDeviceInitReq->i2c_address;
  // Determine which sensor-specific callback to utilize
  if (msgDeviceInitReq->has_aht) {
    // Initialize new AHTX0 sensor
    _ahtx0 = new WipperSnapper_I2C_Driver_AHTX0(this->_i2c, i2cAddress);

    // Did we initialize successfully?
    if (!_ahtx0->getInitialized()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize AHTX0 chip!");
      return false;
    }
    WS_DEBUG_PRINTLN("AHTX0 Initialized Successfully!");

    // Configure AHTX0 sensor
    if (msgDeviceInitReq->aht.enable_temperature) {
      _ahtx0->enableTemperatureSensor();
      _ahtx0->setTemperatureSensorPeriod(
          msgDeviceInitReq->aht.period_temperature);
      WS_DEBUG_PRINTLN("Enabled AHTX0 Temperature Sensor, [Returns every: ");
      WS_DEBUG_PRINT(msgDeviceInitReq->aht.period_temperature);
      WS_DEBUG_PRINTLN("seconds]");
    }
    if (msgDeviceInitReq->aht.enable_humidity) {
      _ahtx0->enableHumiditySensor();
      _ahtx0->setHumiditySensorPeriod(msgDeviceInitReq->aht.period_humidity);
      WS_DEBUG_PRINTLN("Enabled AHTX0 Humidity Sensor, [Returns every: ");
      WS_DEBUG_PRINT(msgDeviceInitReq->aht.period_humidity);
      WS_DEBUG_PRINTLN("seconds]");
    }
    drivers.push_back(_ahtx0);
  } else if (msgDeviceInitReq->has_dps) {
    // Initialize new DPS310 sensor
    _dps310 = new WipperSnapper_I2C_Driver_DPS310(this->_i2c, i2cAddress);

    // Did we initialize successfully?
    if (!_dps310->getInitialized()) {
      WS_DEBUG_PRINTLN("ERROR: DPS310 not initialized successfully!");
      return false;
    }
    WS_DEBUG_PRINTLN("Successfully Initialized DPS310!");

    // Configure DPS310
    if (msgDeviceInitReq->dps.enable_temperature) {
      _dps310->enableTemperatureSensor();
      _dps310->setTemperatureSensorPeriod(
          msgDeviceInitReq->dps.period_temperature);
      WS_DEBUG_PRINTLN("Enabled DPS310 Humidity Sensor, [Returns every: ");
      WS_DEBUG_PRINT(msgDeviceInitReq->dps.period_temperature);
      WS_DEBUG_PRINTLN("seconds]");
    }
    if (msgDeviceInitReq->dps.enable_pressure) {
      _dps310->enablePressureSensor();
      _dps310->setPressureSensorPeriod(msgDeviceInitReq->dps.period_pressure);
      WS_DEBUG_PRINTLN("Enabled DPS310 Pressure Sensor, [Returns every: ");
      WS_DEBUG_PRINT(msgDeviceInitReq->dps.period_pressure);
      WS_DEBUG_PRINTLN("seconds]");
    }
    drivers.push_back(_dps310);
  } else if (msgDeviceInitReq->has_scd30) {
    // Initialize new SCD30 sensor
    _scd30 = new WipperSnapper_I2C_Driver_SCD30(this->_i2c, i2cAddress);

    // Did we initialize successfully?
    if (!_scd30->getInitialized()) {
      WS_DEBUG_PRINTLN("ERROR: SCD30 not initialized successfully!");
      return false;
    }
    WS_DEBUG_PRINTLN("Successfully Initialized SCD30!");

    // Configure SCD4x
    if (msgDeviceInitReq->scd30.enable_temperature) {
      _scd30->enableTemperatureSensor();
      _scd30->setTemperatureSensorPeriod(
          msgDeviceInitReq->scd30.period_temperature);
      WS_DEBUG_PRINTLN("Enabled SCD30 Humidity Sensor, [Returns every: ");
      WS_DEBUG_PRINT(msgDeviceInitReq->scd30.period_temperature);
      WS_DEBUG_PRINTLN("seconds]");
    }
    if (msgDeviceInitReq->scd30.enable_humidity) {
      _scd30->enableHumiditySensor();
      _scd30->setHumiditySensorPeriod(msgDeviceInitReq->scd30.period_humidity);
      WS_DEBUG_PRINTLN("Enabled SCD30 Humidity Sensor, [Returns every: ");
      WS_DEBUG_PRINT(msgDeviceInitReq->scd30.period_humidity);
      WS_DEBUG_PRINTLN("seconds]");
    }
    if (msgDeviceInitReq->scd30.enable_co2) {
      _scd30->enableCO2Sensor();
      _scd30->setCO2SensorPeriod(msgDeviceInitReq->scd30.period_co2);
      WS_DEBUG_PRINTLN("Enabled SCD30 CO2 Sensor, [Returns every: ");
      WS_DEBUG_PRINT(msgDeviceInitReq->scd30.period_co2);
      WS_DEBUG_PRINTLN("seconds]");
    }
    drivers.push_back(_scd30);
  } else if (msgDeviceInitReq->has_scd4x) {
    // Initialize new SCD4x sensor
    _scd4x = new WipperSnapper_I2C_Driver_SCD4X(this->_i2c, i2cAddress);

    // Did we initialize successfully?
    if (!_scd4x->getInitialized()) {
      WS_DEBUG_PRINTLN("ERROR: SCD4x not initialized successfully!");
      return false;
    }
    WS_DEBUG_PRINTLN("Successfully Initialized SCD4x!");

    // Configure SCD4x
    if (msgDeviceInitReq->scd4x.enable_temperature) {
      _scd4x->enableTemperatureSensor();
      _scd4x->setTemperatureSensorPeriod(
          msgDeviceInitReq->scd4x.period_temperature);
      WS_DEBUG_PRINTLN("Enabled scd4x Humidity Sensor, [Returns every: ");
      WS_DEBUG_PRINT(msgDeviceInitReq->scd4x.period_temperature);
      WS_DEBUG_PRINTLN("seconds]");
    }
    if (msgDeviceInitReq->scd4x.enable_humidity) {
      _scd4x->enableHumiditySensor();
      _scd4x->setHumiditySensorPeriod(msgDeviceInitReq->scd4x.period_humidity);
      WS_DEBUG_PRINTLN("Enabled scd4x Humidity Sensor, [Returns every: ");
      WS_DEBUG_PRINT(msgDeviceInitReq->scd4x.period_humidity);
      WS_DEBUG_PRINTLN("seconds]");
    }
    if (msgDeviceInitReq->scd4x.enable_co2) {
      _scd4x->enableCO2Sensor();
      _scd4x->setCO2SensorPeriod(msgDeviceInitReq->scd4x.period_co2);
      WS_DEBUG_PRINTLN("Enabled scd4x CO2 Sensor, [Returns every: ");
      WS_DEBUG_PRINT(msgDeviceInitReq->scd4x.period_co2);
      WS_DEBUG_PRINTLN("seconds]");
    }
    drivers.push_back(_scd4x);
  } else {
    WS_DEBUG_PRINTLN("ERROR: I2C device type not found!")
    return false;
  }
  return true;
}

/*********************************************************************************/
/*!
    @brief    Updates the properties of an I2C device driver.
    @param    msgDeviceUpdateReq
              A decoded I2CDeviceUpdateRequest.
    @returns True if the I2C device is was successfully updated, False
   otherwise.
*/
/*********************************************************************************/
bool WipperSnapper_Component_I2C::updateI2CDeviceProperties(
    wippersnapper_i2c_v1_I2CDeviceUpdateRequest *msgDeviceUpdateReq) {
  bool is_success = false;
  uint16_t deviceAddr = (uint16_t)msgDeviceUpdateReq->i2c_address;
  // Loop thru vector of drivers to find the unique address
  for (int i = 0; i < drivers.size(); i++) {
    if (drivers[i]->getSensorAddress() == deviceAddr) {
      // Check driver type
      if (drivers[i]->driverType == AHTX0) {
        // Update AHTX0 sensor configuration
        if (msgDeviceUpdateReq->aht.enable_temperature == true) {
          drivers[i]->enableTemperatureSensor();
          WS_DEBUG_PRINTLN("ENABLED AHTX0 Temperature Sensor");
        } else {
          drivers[i]->disableTemperatureSensor();
          WS_DEBUG_PRINTLN("DISABLED AHTX0 Temperature Sensor");
        }

        if (msgDeviceUpdateReq->aht.enable_humidity == true) {
          drivers[i]->enableHumiditySensor();
          WS_DEBUG_PRINTLN("ENABLED AHTX0 Humidity Sensor");
        } else {
          drivers[i]->disableHumiditySensor();
          WS_DEBUG_PRINTLN("ENABLED AHTX0 Humidity Sensor");
        }

        // Update AHTX0's  sensor time periods
        if (drivers[i]->getTempSensorPeriod() !=
            msgDeviceUpdateReq->aht.period_temperature) {
          drivers[i]->setTemperatureSensorPeriod(
              msgDeviceUpdateReq->aht.period_temperature);
          WS_DEBUG_PRINTLN(
              "UPDATED AHTX0 Temperature Sensor, [Returns every: ");
          WS_DEBUG_PRINT(msgDeviceUpdateReq->aht.period_temperature);
          WS_DEBUG_PRINTLN("seconds]");
        }
        if (drivers[i]->getHumidSensorPeriod() !=
            msgDeviceUpdateReq->aht.period_humidity) {
          drivers[i]->setHumiditySensorPeriod(
              msgDeviceUpdateReq->aht.period_humidity);
          WS_DEBUG_PRINTLN("UPDATED AHTX0 Humidity Sensor, [Returns every: ");
          WS_DEBUG_PRINT(msgDeviceUpdateReq->aht.period_humidity);
          WS_DEBUG_PRINTLN("seconds]");
        }
        is_success = true;
      } else if (drivers[i]->driverType == DPS310) {
        // Update DPS310 sensor configuration
        if (msgDeviceUpdateReq->dps.enable_temperature == true) {
          drivers[i]->enableTemperatureSensor();
          WS_DEBUG_PRINTLN("ENABLED DPS310 Temperature Sensor");
        } else {
          drivers[i]->disableTemperatureSensor();
          WS_DEBUG_PRINTLN("DISABLED DPS310 Temperature Sensor");
        }

        if (msgDeviceUpdateReq->dps.enable_pressure == true) {
          drivers[i]->enablePressureSensor();
          WS_DEBUG_PRINTLN("ENABLED DPS310 Pressure Sensor");
        } else {
          drivers[i]->disablePressureSensor();
          WS_DEBUG_PRINTLN("DISABLED DPS310 Pressure Sensor");
        }

        // Update DPS310's sensor time periods
        if (drivers[i]->getTempSensorPeriod() !=
            msgDeviceUpdateReq->dps.period_temperature) {
          drivers[i]->setTemperatureSensorPeriod(
              msgDeviceUpdateReq->dps.period_temperature);
          WS_DEBUG_PRINTLN(
              "UPDATED DPS310 Temperature Sensor, [Returns every: ");
          WS_DEBUG_PRINT(msgDeviceUpdateReq->dps.period_temperature);
          WS_DEBUG_PRINTLN("seconds]");
        }
        if (drivers[i]->getPressureSensorPeriod() !=
            msgDeviceUpdateReq->dps.period_pressure) {
          drivers[i]->setPressureSensorPeriod(
              msgDeviceUpdateReq->dps.period_pressure);
          WS_DEBUG_PRINTLN("UPDATED DPS310 Pressure Sensor, [Returns every: ");
          WS_DEBUG_PRINT(msgDeviceUpdateReq->dps.period_pressure);
          WS_DEBUG_PRINTLN("seconds]");
        }
        is_success = true;
        break;
      } else if (drivers[i]->driverType == SCD30) {
        // Update SCD30 sensor configuration
        if (msgDeviceUpdateReq->scd30.enable_temperature == true) {
          drivers[i]->enableTemperatureSensor();
          WS_DEBUG_PRINTLN("ENABLED SCD30 Temperature Sensor");
        } else {
          drivers[i]->disableTemperatureSensor();
          WS_DEBUG_PRINTLN("DISABLED SCD30 Temperature Sensor");
        }

        if (msgDeviceUpdateReq->scd30.enable_humidity == true) {
          drivers[i]->enableHumiditySensor();
          WS_DEBUG_PRINTLN("ENABLED SCD30 Humidity Sensor");
        } else {
          drivers[i]->disableHumiditySensor();
          WS_DEBUG_PRINTLN("DISABLED SCD30 Humidity Sensor");
        }

        // Update SCD30's sensor time periods
        if (drivers[i]->getTempSensorPeriod() !=
            msgDeviceUpdateReq->scd30.period_temperature) {
          drivers[i]->setTemperatureSensorPeriod(
              msgDeviceUpdateReq->scd30.period_temperature);
          WS_DEBUG_PRINTLN(
              "UPDATED SCD30 Temperature Sensor, [Returns every: ");
          WS_DEBUG_PRINT(msgDeviceUpdateReq->scd30.period_temperature);
          WS_DEBUG_PRINTLN("seconds]");
        }

        if (drivers[i]->getHumidSensorPeriod() !=
            msgDeviceUpdateReq->scd30.period_humidity) {
          drivers[i]->setHumiditySensorPeriod(
              msgDeviceUpdateReq->scd30.period_humidity);
          WS_DEBUG_PRINTLN("UPDATED SCD30 Humidity Sensor, [Returns every: ");
          WS_DEBUG_PRINT(msgDeviceUpdateReq->scd30.period_humidity);
          WS_DEBUG_PRINTLN("seconds]");
        }

        if (drivers[i]->getCO2SensorPeriod() !=
            msgDeviceUpdateReq->scd30.period_co2) {
          drivers[i]->setCO2SensorPeriod(msgDeviceUpdateReq->scd30.period_co2);
          WS_DEBUG_PRINTLN("UPDATED SCD30 Pressure Sensor, [Returns every: ");
          WS_DEBUG_PRINT(msgDeviceUpdateReq->scd30.period_co2);
          WS_DEBUG_PRINTLN("seconds]");
        }
        is_success = true;
        break;
      }
    } else if (drivers[i]->driverType == SCD4X) {
      // Update SCD4x sensor configuration
      if (msgDeviceUpdateReq->scd4x.enable_temperature == true) {
        drivers[i]->enableTemperatureSensor();
        WS_DEBUG_PRINTLN("ENABLED scd4x Temperature Sensor");
      } else {
        drivers[i]->disableTemperatureSensor();
        WS_DEBUG_PRINTLN("DISABLED scd4x Temperature Sensor");
      }

      if (msgDeviceUpdateReq->scd4x.enable_humidity == true) {
        drivers[i]->enableHumiditySensor();
        WS_DEBUG_PRINTLN("ENABLED scd4x Humidity Sensor");
      } else {
        drivers[i]->disableHumiditySensor();
        WS_DEBUG_PRINTLN("DISABLED scd4x Humidity Sensor");
      }

      // Update scd4x's sensor time periods
      if (drivers[i]->getTempSensorPeriod() !=
          msgDeviceUpdateReq->scd4x.period_temperature) {
        drivers[i]->setTemperatureSensorPeriod(
            msgDeviceUpdateReq->scd4x.period_temperature);
        WS_DEBUG_PRINTLN("UPDATED scd4x Temperature Sensor, [Returns every: ");
        WS_DEBUG_PRINT(msgDeviceUpdateReq->scd4x.period_temperature);
        WS_DEBUG_PRINTLN("seconds]");
      }

      if (drivers[i]->getHumidSensorPeriod() !=
          msgDeviceUpdateReq->scd4x.period_humidity) {
        drivers[i]->setHumiditySensorPeriod(
            msgDeviceUpdateReq->scd4x.period_humidity);
        WS_DEBUG_PRINTLN("UPDATED scd4x Humidity Sensor, [Returns every: ");
        WS_DEBUG_PRINT(msgDeviceUpdateReq->scd4x.period_humidity);
        WS_DEBUG_PRINTLN("seconds]");
      }

      if (drivers[i]->getCO2SensorPeriod() !=
          msgDeviceUpdateReq->scd4x.period_co2) {
        drivers[i]->setCO2SensorPeriod(msgDeviceUpdateReq->scd4x.period_co2);
        WS_DEBUG_PRINTLN("UPDATED scd4x Pressure Sensor, [Returns every: ");
        WS_DEBUG_PRINT(msgDeviceUpdateReq->scd4x.period_co2);
        WS_DEBUG_PRINTLN("seconds]");
      }
      is_success = true;
      break;
    } else {
      WS_DEBUG_PRINTLN("ERROR: Sensor driver not found!");
    }
  }
  return is_success;
}

/*******************************************************************************/
/*!
    @brief    Deinitializes and deletes an I2C device driver object.
    @param    msgDeviceDeinitReq
              A decoded I2CDeviceDeinitRequest.
    @returns True if I2C device is found and de-initialized, False otherwise.
*/
/*******************************************************************************/
bool WipperSnapper_Component_I2C::deinitI2CDevice(
    wippersnapper_i2c_v1_I2CDeviceDeinitRequest *msgDeviceDeinitReq) {
  bool is_success = false;
  uint16_t deviceAddr = (uint16_t)msgDeviceDeinitReq->i2c_address;
  // Loop thru vector of drivers to find the unique address
  for (int i = 0; i < drivers.size(); i++) {
    if (drivers[i]->getSensorAddress() == deviceAddr) {
      // Check driver type
      if (drivers[i]->driverType == AHTX0) {
        // delete the driver and remove from list so we dont attempt to
        // update() it
        delete _ahtx0;
        drivers.erase(drivers.begin() + i);
        WS_DEBUG_PRINTLN("DEINIT'D AHTX0");
        is_success = true;
      } else if (drivers[i]->driverType == DPS310) {
        delete _dps310;
        drivers.erase(drivers.begin() + i);
        WS_DEBUG_PRINTLN("DEINIT'D DPS310");
        is_success = true;
      } else if (drivers[i]->driverType == SCD30) {
        delete _scd30;
        drivers.erase(drivers.begin() + i);
        WS_DEBUG_PRINTLN("DEINIT'D SCD30");
        is_success = true;
      } else if (drivers[i]->driverType == SCD4X) {
        delete _scd4x;
        drivers.erase(drivers.begin() + i);
        WS_DEBUG_PRINTLN("DEINIT'D SCD4X");
        is_success = true;
      }
    } else {
      WS_DEBUG_PRINTLN("ERROR: Driver type unspecified");
    }
  }
  return is_success;
}

/*******************************************************************************/
/*!
    @brief    Encodes an I2C sensor device's signal message.
    @param    msgi2cResponse
              Pointer to an I2CResponse signal message.
    @param    sensorAddress
              The unique I2C address of the sensor.
    @returns  True if message encoded successfully, False otherwise.
*/
/*******************************************************************************/
bool WipperSnapper_Component_I2C::encodeI2CDeviceEventMsg(
    wippersnapper_signal_v1_I2CResponse *msgi2cResponse,
    uint32_t sensorAddress) {
  msgi2cResponse->payload.resp_i2c_device_event.sensor_address = sensorAddress;
  memset(WS._buffer_outgoing, 0, sizeof(WS._buffer_outgoing));
  pb_ostream_t ostream =
      pb_ostream_from_buffer(WS._buffer_outgoing, sizeof(WS._buffer_outgoing));
  if (!pb_encode(&ostream, wippersnapper_signal_v1_I2CResponse_fields,
                 msgi2cResponse)) {
    WS_DEBUG_PRINTLN(
        "ERROR: Unable to encode I2C device event response message!");
    return false;
  }
  return true;
}

/*******************************************************************************/
/*!
    @brief    Publishes an I2C sensor device's signal message.
    @param    msgi2cResponse
              Pointer to an I2CResponse signal message.
    @returns  True if message published to the broker successfully,
                False otherwise.
*/
/*******************************************************************************/
bool WipperSnapper_Component_I2C::publishI2CDeviceEventMsg(
    wippersnapper_signal_v1_I2CResponse *msgi2cResponse) {
  size_t msgSz;
  pb_get_encoded_size(&msgSz, wippersnapper_signal_v1_I2CResponse_fields,
                      msgi2cResponse);
  WS_DEBUG_PRINT("PUBLISHING -> I2C Device Sensor Event Message...");
  if (!WS._mqtt->publish(WS._topic_signal_i2c_device, WS._buffer_outgoing,
                         msgSz, 1)) {
    return false;
  };
  WS_DEBUG_PRINTLN("PUBLISHED!");
  return true;
}

/*******************************************************************************/
/*!
    @brief    Fills a sensor_event message with the sensor's value and type.
    @param    msgi2cResponse
              A pointer to the signal's I2CResponse message.
    @param    value
              The value read by the sensor.
    @param    sensorType
              The SI unit represented by the sensor's value.
    @param    precision
              The amount of decimal points to round to.
*/
/*******************************************************************************/
void WipperSnapper_Component_I2C::fillEventMessage(
    wippersnapper_signal_v1_I2CResponse *msgi2cResponse, float value,
    wippersnapper_i2c_v1_SensorType sensorType, uint8_t precision = 2) {
  // fill sensor value
  msgi2cResponse->payload.resp_i2c_device_event
      .sensor_event[msgi2cResponse->payload.resp_i2c_device_event
                        .sensor_event_count]
      .value = value;
  // fill sensor type
  msgi2cResponse->payload.resp_i2c_device_event
      .sensor_event[msgi2cResponse->payload.resp_i2c_device_event
                        .sensor_event_count]
      .type = sensorType;
  msgi2cResponse->payload.resp_i2c_device_event.sensor_event_count++;
}

/*******************************************************************************/
/*!
    @brief    Queries all I2C device drivers for new values. Fills and sends an
              I2CSensorEvent with the sensor event data.
*/
/*******************************************************************************/
void WipperSnapper_Component_I2C::update() {
  // New I2CResponse message
  wippersnapper_signal_v1_I2CResponse msgi2cResponse =
      wippersnapper_signal_v1_I2CResponse_init_zero;
  // Set I2CDeviceEvent tag
  msgi2cResponse.which_payload =
      wippersnapper_signal_v1_I2CResponse_resp_i2c_device_event_tag;

  long curTime;
  for (int i = 0; i < drivers.size(); i++) {
    // Check driver type

    if (drivers[i]->driverType == AHTX0) {
      // reset sensor # counter
      msgi2cResponse.payload.resp_i2c_device_event.sensor_event_count = 0;

      // temp
      curTime = millis();
      if (curTime - drivers[i]->getTempSensorPeriodPrv() >
              drivers[i]->getTempSensorPeriod() &&
          drivers[i]->getTempSensorPeriod() != 0L) {
        // poll
        sensors_event_t temp;
        WS_DEBUG_PRINTLN("Polling AHTX0 Temperature Sensor...");
        if (!drivers[i]->getTemp(&temp)) {
          WS_DEBUG_PRINTLN("ERROR: Unable to obtain AHTX0 temperature value.");
          break;
        }
        WS_DEBUG_PRINT("\tTemperature: ");
        WS_DEBUG_PRINT(temp.temperature);
        WS_DEBUG_PRINTLN(" degrees C");

        // pack data into msg
        fillEventMessage(
            &msgi2cResponse, temp.temperature,
            wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE);
        drivers[i]->setTemperatureSensorPeriodPrv(curTime);
      }

      // humid
      curTime = millis();
      if (curTime - drivers[i]->getHumidSensorPeriodPrv() >
              drivers[i]->getHumidSensorPeriod() &&
          drivers[i]->getHumidSensorPeriod() != 0L) {
        // poll
        WS_DEBUG_PRINTLN("Polling AHTX0 Humidity Sensor...");
        sensors_event_t humid;
        if (!drivers[i]->getHumid(&humid)) {
          WS_DEBUG_PRINTLN("ERROR: Unable to obtain AHTX0 humidity value.");
          break;
        }
        WS_DEBUG_PRINT("\tHumidity: ");
        WS_DEBUG_PRINT(humid.relative_humidity);
        WS_DEBUG_PRINTLN(" % rH");

        // pack data into msg
        // TODO: Truncate within this call? (precision=)
        fillEventMessage(
            &msgi2cResponse, humid.relative_humidity,
            wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_RELATIVE_HUMIDITY);
        drivers[i]->setHumidSensorPeriodPrv(curTime);
      }
    } else if (drivers[i]->driverType == DPS310) {
      // reset sensor # counter
      msgi2cResponse.payload.resp_i2c_device_event.sensor_event_count = 0;
      // Check if we're polling the temperature sensor
      // Nothing here is aht-specific though...
      if (millis() - drivers[i]->getTempSensorPeriodPrv() >
              drivers[i]->getTempSensorPeriod() &&
          drivers[i]->getTempSensorPeriod() > 0L) {
        // poll
        sensors_event_t tempEvent;
        WS_DEBUG_PRINTLN("Polling DPS310 Temperature Sensor...");
        if (!drivers[i]->getTemp(&tempEvent)) {
          WS_DEBUG_PRINTLN("ERROR: Unable to obtain DPS310 temperature value.");
          break;
        }
        WS_DEBUG_PRINT("\tTemperature: ");
        WS_DEBUG_PRINT(tempEvent.temperature);
        WS_DEBUG_PRINTLN(" degrees C");
        // pack data into msg
        fillEventMessage(
            &msgi2cResponse, tempEvent.temperature,
            wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE);
      }

      // Check if we're polling the humidity sensor
      if (millis() - drivers[i]->getPressureSensorPeriodPrv() >
              drivers[i]->getPressureSensorPeriod() &&
          drivers[i]->getPressureSensorPeriod() > 0L) {
        // poll
        WS_DEBUG_PRINTLN("Polling DPS310 Pressure Sensor...");
        sensors_event_t presEvent;
        if (!drivers[i]->getPressure(&presEvent)) {
          WS_DEBUG_PRINTLN("ERROR: Unable to obtain DPS310 pressure value.");
          break;
        }
        WS_DEBUG_PRINT("\tPressure: ");
        WS_DEBUG_PRINT(presEvent.pressure);
        WS_DEBUG_PRINTLN(" hPa");

        // pack data into msg
        fillEventMessage(&msgi2cResponse, presEvent.pressure,
                         wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PRESSURE);
      }
    } else if (drivers[i]->driverType == SCD30) {
      // reset sensor # counter
      msgi2cResponse.payload.resp_i2c_device_event.sensor_event_count = 0;

      // Check if we're polling the temperature sensor
      if (millis() - drivers[i]->getTempSensorPeriodPrv() >
              drivers[i]->getTempSensorPeriod() &&
          drivers[i]->getTempSensorPeriod() > 0L) {
        // poll
        sensors_event_t tempEvent;
        WS_DEBUG_PRINTLN("Polling SCD30 Temperature Sensor...");
        if (!drivers[i]->getTemp(&tempEvent)) {
          WS_DEBUG_PRINTLN("ERROR: Unable to obtain SCD30 temperature value.");
          break;
        }
        WS_DEBUG_PRINT("\tTemperature: ");
        WS_DEBUG_PRINT(tempEvent.temperature);
        WS_DEBUG_PRINTLN(" degrees C");
        // pack data into msg
        fillEventMessage(
            &msgi2cResponse, tempEvent.temperature,
            wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE);
      }

      // Check if we're polling the humidity sensor
      if (millis() - drivers[i]->getHumidSensorPeriodPrv() >
              drivers[i]->getHumidSensorPeriod() &&
          drivers[i]->getHumidSensorPeriod() > 0L) {
        // poll
        WS_DEBUG_PRINTLN("Polling SCD30 Humidity Sensor...");
        sensors_event_t humidEvent;
        if (!drivers[i]->getHumid(&humidEvent)) {
          WS_DEBUG_PRINTLN("ERROR: Unable to obtain SCD30 humidity value.");
          break;
        }
        WS_DEBUG_PRINT("\tHumidity: ");
        WS_DEBUG_PRINT(humidEvent.relative_humidity);
        WS_DEBUG_PRINTLN(" %RH");

        // pack data into msg
        fillEventMessage(
            &msgi2cResponse, humidEvent.relative_humidity,
            wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_RELATIVE_HUMIDITY);
      }

      // Check if we're polling the humidity sensor
      if (millis() - drivers[i]->getHumidSensorPeriodPrv() >
              drivers[i]->getHumidSensorPeriod() &&
          drivers[i]->getHumidSensorPeriod() > 0L) {
        // poll
        WS_DEBUG_PRINTLN("Polling SCD30 Humidity Sensor...");
        sensors_event_t humidEvent;
        if (!drivers[i]->getHumid(&humidEvent)) {
          WS_DEBUG_PRINTLN("ERROR: Unable to obtain SCD30 humidity value.");
          break;
        }
        WS_DEBUG_PRINT("\tHumidity: ");
        WS_DEBUG_PRINT(humidEvent.relative_humidity);
        WS_DEBUG_PRINTLN(" %RH");

        // pack data into msg
        fillEventMessage(
            &msgi2cResponse, humidEvent.relative_humidity,
            wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_RELATIVE_HUMIDITY);
      }

      // Check if we're polling the gas sensor
      if (millis() - drivers[i]->getCO2SensorPeriodPrv() >
              drivers[i]->getCO2SensorPeriod() &&
          drivers[i]->getCO2SensorPeriod() > 0L) {
        // poll
        WS_DEBUG_PRINTLN("Polling SCD30 C02 Sensor...");
        float CO2;
        if (!drivers[i]->getCO2(&CO2)) {
          WS_DEBUG_PRINTLN("ERROR: Unable to obtain SCD30 CO2 value.");
          break;
        }
        WS_DEBUG_PRINT("\tCO2: ");
        WS_DEBUG_PRINT(CO2);

        // pack data into msg
        fillEventMessage(&msgi2cResponse, CO2,
                         wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_CO2);
      }
    } else if (drivers[i]->driverType == SCD4X) {
      // reset sensor # counter
      msgi2cResponse.payload.resp_i2c_device_event.sensor_event_count = 0;

      // Check if we're polling the temperature sensor
      if (millis() - drivers[i]->getTempSensorPeriodPrv() >
              drivers[i]->getTempSensorPeriod() &&
          drivers[i]->getTempSensorPeriod() > 0L) {
        // poll
        float tempEvent = 0.0f;
        WS_DEBUG_PRINTLN("Polling SCD4x Temperature Sensor...");
        if (!drivers[i]->getTemp(&tempEvent)) {
          WS_DEBUG_PRINTLN("ERROR: Unable to obtain SCD4X temperature value.");
          break;
        }
        WS_DEBUG_PRINT("\tTemperature: ");
        WS_DEBUG_PRINT(tempEvent);
        WS_DEBUG_PRINTLN(" degrees C");
        // pack data into msg
        fillEventMessage(
            &msgi2cResponse, tempEvent,
            wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE);
      }

      // Check if we're polling the humidity sensor
      if (millis() - drivers[i]->getHumidSensorPeriodPrv() >
              drivers[i]->getHumidSensorPeriod() &&
          drivers[i]->getHumidSensorPeriod() > 0L) {
        // poll
        WS_DEBUG_PRINTLN("Polling SCD40 Humidity Sensor...");
        float humidEvent = 0.0f;
        if (!drivers[i]->getHumid(&humidEvent)) {
          WS_DEBUG_PRINTLN("ERROR: Unable to obtain SCD40 humidity value.");
          break;
        }
        WS_DEBUG_PRINT("\tHumidity: ");
        WS_DEBUG_PRINT(humidEvent);
        WS_DEBUG_PRINTLN(" %RH");

        // pack data into msg
        fillEventMessage(
            &msgi2cResponse, humidEvent,
            wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_RELATIVE_HUMIDITY);
      }

      // Check if we're polling the humidity sensor
      if (millis() - drivers[i]->getHumidSensorPeriodPrv() >
              drivers[i]->getHumidSensorPeriod() &&
          drivers[i]->getHumidSensorPeriod() > 0L) {
        // poll
        WS_DEBUG_PRINTLN("Polling SCD30 Humidity Sensor...");
        sensors_event_t humidEvent;
        if (!drivers[i]->getHumid(&humidEvent)) {
          WS_DEBUG_PRINTLN("ERROR: Unable to obtain SCD30 humidity value.");
          break;
        }
        WS_DEBUG_PRINT("\tHumidity: ");
        WS_DEBUG_PRINT(humidEvent.relative_humidity);
        WS_DEBUG_PRINTLN(" %RH");

        // pack data into msg
        fillEventMessage(
            &msgi2cResponse, humidEvent.relative_humidity,
            wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_RELATIVE_HUMIDITY);
      }

      // Check if we're polling the gas sensor
      if (millis() - drivers[i]->getCO2SensorPeriodPrv() >
              drivers[i]->getCO2SensorPeriod() &&
          drivers[i]->getCO2SensorPeriod() > 0L) {
        // poll
        WS_DEBUG_PRINTLN("Polling SCD30 C02 Sensor...");
        float CO2;
        if (!drivers[i]->getCO2(&CO2)) {
          WS_DEBUG_PRINTLN("ERROR: Unable to obtain SCD30 CO2 value.");
          break;
        }
        WS_DEBUG_PRINT("\tCO2: ");
        WS_DEBUG_PRINT(CO2);

        // pack data into msg
        fillEventMessage(&msgi2cResponse, CO2,
                         wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_CO2);
      }
    }
    // Did we write into the device event?
    if (msgi2cResponse.payload.resp_i2c_device_event.sensor_event_count > 0) {
      // Encode device event message
      if (!encodeI2CDeviceEventMsg(&msgi2cResponse,
                                   (uint32_t)drivers[i]->getSensorAddress())) {
        WS_DEBUG_PRINTLN("ERROR: Failed to encode sensor event");
        break;
      }
      // Publish device event message
      if (!publishI2CDeviceEventMsg(&msgi2cResponse)) {
        WS_DEBUG_PRINTLN("ERROR: Failed to publish sensor event");
        break;
      }
    }
  } // loop
}