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

  // validate if SDA & SCL has pullup
  if (digitalRead(msgInitRequest->i2c_pin_sda) == LOW) {
    pinMode(msgInitRequest->i2c_pin_sda, INPUT_PULLUP);
  }
  if (digitalRead(msgInitRequest->i2c_pin_scl) == LOW) {
    pinMode(msgInitRequest->i2c_pin_scl, INPUT_PULLUP);
  }

// Initialize TwoWire interface
#if defined(ARDUINO_ARCH_ESP32)
  // ESP32, ESP32-S2
  _i2c = new TwoWire(msgInitRequest->i2c_port_number);
  _i2c->begin(msgInitRequest->i2c_pin_sda, msgInitRequest->i2c_pin_scl);
#else
  // SAMD
  _i2c = new TwoWire(&PERIPH_WIRE, msgInitRequest->i2c_pin_sda,
                     msgInitRequest->i2c_pin_scl);
  _i2c->begin();
#endif

  _i2c->setClock(msgInitRequest->i2c_frequency);
  // set i2c obj. properties
  _portNum = msgInitRequest->i2c_port_number;
  _isInit = true;
  yield();
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

/************************************************************************/
/*!
    @brief    Scans all I2C addresses on the bus between 0x08 and 0x7F
              inclusive and returns an array of the devices found.
    @returns  wippersnapper_i2c_v1_I2CBusScanResponse
*/
/************************************************************************/
wippersnapper_i2c_v1_I2CBusScanResponse
WipperSnapper_Component_I2C::scanAddresses() {
  WS_DEBUG_PRINT("EXEC: I2C Scan");

  // Create response
  wippersnapper_i2c_v1_I2CBusScanResponse scanResp =
      wippersnapper_i2c_v1_I2CBusScanResponse_init_zero;

  // Scan all I2C addresses between 0x08 and 0x7F inclusive and return a list of
  // those that respond.
  for (uint16_t addr = 0x08; addr < 0x7F; addr++) {
    _i2c->beginTransmission(addr);
    // Address ACKed
    if (_i2c->endTransmission() == 0) {
      WS_DEBUG_PRINT("Found I2C Device on: ");
      WS_DEBUG_PRINTLN(addr);
      scanResp.addresses_found[scanResp.addresses_found_count] = (uint32_t)addr;
      scanResp.addresses_found_count++;
    }
  }

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
  } else if (msgDeviceInitReq->has_dps310) {
    // Initialize new DPS310 sensor
    _dps310 = new WipperSnapper_I2C_Driver_DPS310(this->_i2c, i2cAddress);

    // Did we initialize successfully?
    if (!_dps310->getInitialized()) {
        WS_DEBUG_PRINTLN("ERROR: DPS310 not initialized successfully!");
        return false;
    }
    WS_DEBUG_PRINTLN("Successfully Initialized DPS310!");

    // Configure DPS310
    if (msgDeviceInitReq->dps310.enable_temperature) {
        _dps310->enableTemperatureSensor();
        _dps310->setTemperatureSensorPeriod(msgDeviceInitReq->dps310.period_temperature);
        WS_DEBUG_PRINTLN("Enabled DPS310 Humidity Sensor, [Returns every: ");
        WS_DEBUG_PRINT(msgDeviceInitReq->dps310.period_temperature);
        WS_DEBUG_PRINTLN("seconds]");
    }
    if (msgDeviceInitReq->dps310.enable_pressure) {
        _dps310->enablePressureSensor();
        _dps310->setPressureSensorPeriod(msgDeviceInitReq->dps310.period_pressure);
        WS_DEBUG_PRINTLN("Enabled DPS310 Pressure Sensor, [Returns every: ");
        WS_DEBUG_PRINT(msgDeviceInitReq->dps310.period_pressure);
        WS_DEBUG_PRINTLN("seconds]");
    }
    drivers.push_back(_dps310);
  }
  else {
    WS_DEBUG_PRINTLN("ERROR: Sensor not found")
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
bool WipperSnapper_Component_I2C::updateI2CDevice(
    wippersnapper_i2c_v1_I2CDeviceUpdateRequest *msgDeviceUpdateReq) {
  bool is_success = false;
  uint16_t deviceAddr = (uint16_t)msgDeviceUpdateReq->i2c_address;
  // Loop thru vector of drivers to find the unique address
  for (int i = 0; i < drivers.size(); i++) {
    if (drivers[i]->getSensorAddress() == deviceAddr) {
      // Check driver type
      if (drivers[i]->getDriverType() == AHTX0) {
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
      } else if (drivers[i]->getDriverType() == DPS310) {
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
      }
      else {
        WS_DEBUG_PRINTLN("ERROR: Sensor driver not found!");
      }
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
  uint16_t deviceAddr = (uint16_t)msgDeviceDeinitReq->i2c_address;
  // Loop thru vector of drivers to find the unique address
  for (int i = 0; i < drivers.size(); i++) {
    if (drivers[i]->getSensorAddress() == deviceAddr) {
      // Check driver type
      if (drivers[i]->getDriverType() == AHTX0) {
        // delete the driver and remove from list so we dont attempt to
        // update() it
        delete _ahtx0;
        drivers.erase(drivers.begin() + i);
        WS_DEBUG_PRINTLN("AHTX0 Deleted");
        return true;
      }
    } else {
      WS_DEBUG_PRINTLN("ERROR: Driver type unspecified");
    }
  }
  // Driver was not erased or not found
  return false;
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
                 &msgi2cResponse)) {
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
      wippersnapper_signal_v1_I2CResponse_resp_i2c_device_update_tag;

  long curTime;
  for (int i = 0; i < drivers.size(); i++) {
    // Check driver type
    if (drivers[i]->getDriverType() == AHTX0) {
      // reset sensor # counter
      msgi2cResponse.payload.resp_i2c_device_event.sensor_event_count = 0;
      // Check if we're polling the temperature sensor
      // Nothing here is aht-specific though...
      if (millis() - drivers[i]->getTempSensorPeriodPrv() >
              drivers[i]->getTempSensorPeriod() &&
          drivers[i]->getTempSensorPeriod() > -1L) {
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
        // Pack event payload
        // TODO: Abstract this
        msgi2cResponse.payload.resp_i2c_device_event
            .sensor_event[msgi2cResponse.payload.resp_i2c_device_event
                              .sensor_event_count]
            .value = temp.temperature;
        msgi2cResponse.payload.resp_i2c_device_event
            .sensor_event[msgi2cResponse.payload.resp_i2c_device_event
                              .sensor_event_count]
            .type =
            wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE;
        msgi2cResponse.payload.resp_i2c_device_event.sensor_event_count++;
      }

      // Check if we're polling the humidity sensor
      if (millis() - drivers[i]->getHumidSensorPeriodPrv() >
              drivers[i]->getHumidSensorPeriod() &&
          drivers[i]->getHumidSensorPeriod() > -1L) {
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
        // Pack event payload
        // TODO: Abstract this
        msgi2cResponse.payload.resp_i2c_device_event
            .sensor_event[msgi2cResponse.payload.resp_i2c_device_event
                              .sensor_event_count]
            .value = humid.relative_humidity;
        msgi2cResponse.payload.resp_i2c_device_event
            .sensor_event[msgi2cResponse.payload.resp_i2c_device_event
                              .sensor_event_count]
            .type =
            wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_RELATIVE_HUMIDITY;
        msgi2cResponse.payload.resp_i2c_device_event.sensor_event_count++;
      }
      // Did we write into the device event?
      if (msgi2cResponse.payload.resp_i2c_device_event.sensor_event_count > 0) {
        // Encode device event message
        if (!encodeI2CDeviceEventMsg(
                &msgi2cResponse, (uint32_t)drivers[i]->getSensorAddress())) {
          WS_DEBUG_PRINTLN("ERROR: Failed to encode sensor event");
          break;
        }
        // Publish device event message
        if (!publishI2CDeviceEventMsg(&msgi2cResponse)) {
          WS_DEBUG_PRINTLN("ERROR: Failed to publish sensor event");
          break;
        }
      }
    }
  }
}