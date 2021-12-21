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
#ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32S2
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
    // ESP32, ESP32-S2
    _i2c = new TwoWire(msgInitRequest->i2c_port_number);
    if (!_i2c->begin(msgInitRequest->i2c_pin_sda,
                     msgInitRequest->i2c_pin_scl)) {
      _isInit = false; // if the peripheral was configured incorrectly
    } else {
      _isInit = true; // if the peripheral was configured incorrectly
    }
    _i2c->setClock(msgInitRequest->i2c_frequency);
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

  if (scanResp.addresses_found_count == 0) {
    WS_DEBUG_PRINTLN("No I2C devices found");
    scanResp.bus_response =
        wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_ERROR_WIRING;
  } else {
    WS_DEBUG_PRINT("I2C Devices Found: ")
    WS_DEBUG_PRINTLN(scanResp.addresses_found_count);
    scanResp.bus_response =
        wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_SUCCESS;
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
  WS_DEBUG_PRINTLN("Attempting to initialize an I2C device...");

  uint16_t i2cAddress = (uint16_t)msgDeviceInitReq->i2c_device_address;

  // AHT20
  if (strcmp("aht20", msgDeviceInitReq->i2c_device_name) == 0) {
    // Initialize AHTX0
    _ahtx0 = new WipperSnapper_I2C_Driver_AHTX0(this->_i2c, i2cAddress);
    if (!_ahtx0->getInitialized()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize AHTX0 chip!");
      return false;
    }
    // add to vec. of driver objects
    drivers.push_back(_ahtx0);
    WS_DEBUG_PRINTLN("AHTX0 Initialized Successfully!");
    // Configure sensor properties
    // TODO: This will become a generic function, I think we'll pass
    // sensorObject into it.
    for (int i = 0; i < msgDeviceInitReq->i2c_device_properties_count; i++) {
      // Generic as well?
      switch (msgDeviceInitReq->i2c_device_properties[i].sensor_type) {
      case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE:
        _ahtx0->enableSensorAmbientTemperature();
        _ahtx0->setSensorAmbientTemperaturePeriod(
            msgDeviceInitReq->i2c_device_properties[i].sensor_period);
        // TODO: This should be moved into the driver?
        WS_DEBUG_PRINTLN("Enabled AHTX0 Temperature Sensor, [Returns every: ");
        WS_DEBUG_PRINT(
            msgDeviceInitReq->i2c_device_properties[i].sensor_period);
        WS_DEBUG_PRINTLN("seconds]");
        break;
      case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_RELATIVE_HUMIDITY:
        _ahtx0->enableSensorRelativeHumidity();
        _ahtx0->setSensorRelativeHumidityPeriod(
            msgDeviceInitReq->i2c_device_properties[i].sensor_period);
        // TODO: This should be moved into the driver?
        WS_DEBUG_PRINTLN("Enabled AHTX0 Temperature Sensor, [Returns every: ");
        WS_DEBUG_PRINT(
            msgDeviceInitReq->i2c_device_properties[i].sensor_period);
        WS_DEBUG_PRINTLN("seconds]");
        break;
      default:
        WS_DEBUG_PRINTLN("ERROR: Unable to determine sensor_type!");
        return false;
      }
    }
    // TODO: Check next type!
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
  bool is_success = true;

  uint16_t i2cAddress = (uint16_t)msgDeviceUpdateReq->i2c_address;

  // Loop thru vector of drivers to find the unique address
  for (int i = 0; i < drivers.size(); i++) {
    if (drivers[i]->getSensorAddress() == i2cAddress) {
      // Update the properties of each driver
      for (int j = 0; j < msgDeviceUpdateReq->i2c_device_properties_count;
           j++) {
        // test
        switch (msgDeviceUpdateReq->i2c_device_properties[j].sensor_type) {
        case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE:
          drivers[i]->updateSensorAmbientTemperature(
              msgDeviceUpdateReq->i2c_device_properties[j].sensor_period);
          break;
        case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_RELATIVE_HUMIDITY:
          drivers[i]->updateSensorRelativeHumidity(
              msgDeviceUpdateReq->i2c_device_properties[j].sensor_period);
          break;
        case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PRESSURE:
          drivers[i]->updateSensorPressure(
              msgDeviceUpdateReq->i2c_device_properties[j].sensor_period);
          break;
        // TODO: implement more types
        default:
          WS_DEBUG_PRINTLN("ERROR: Unable to determine sensor_type!");
          return false;
        }
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
  bool is_success = false;
  uint16_t deviceAddr = (uint16_t)msgDeviceDeinitReq->i2c_address;
  // Loop thru vector of drivers to find the unique address
  for (int i = 0; i < drivers.size(); i++) {
    if (drivers[i]->getSensorAddress() == deviceAddr) {
      drivers.erase(drivers.begin() + i);
      WS_DEBUG_PRINTLN("Driver de-initialized!");
      is_success = true;
    } else {
      WS_DEBUG_PRINTLN("ERROR: Driver type not found");
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

  // Create response message
  wippersnapper_signal_v1_I2CResponse msgi2cResponse =
      wippersnapper_signal_v1_I2CResponse_init_zero;
  msgi2cResponse.which_payload =
      wippersnapper_signal_v1_I2CResponse_resp_i2c_device_event_tag;

  long curTime;
  std::vector<WipperSnapper_I2C_Driver *>::iterator iter, end;
  for (iter = drivers.begin(), end = drivers.end(); iter != end; ++iter) {
    // Number of events which occured for this driver
    msgi2cResponse.payload.resp_i2c_device_event.sensor_event_count = 0;

    // AMBIENT_TEMPERATURE sensor
    curTime = millis();
    if ((*iter)->getSensorAmbientTemperaturePeriod() != 0L &&
        curTime - (*iter)->getSensorAmbientTemperaturePeriodPrv() >
            (*iter)->getSensorAmbientTemperaturePeriod()) {
      sensors_event_t event;
      if ((*iter)->getSensorAmbientTemperature(&event)) {
        WS_DEBUG_PRINT("\tTemperature: ");
        WS_DEBUG_PRINT(event.temperature);
        WS_DEBUG_PRINTLN(" degrees C");

        // pack event data into msg
        fillEventMessage(
            &msgi2cResponse, event.temperature,
            wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE);

        (*iter)->setSensorAmbientTemperaturePeriodPrv(curTime);
      }
    }

    // RELATIVE_HUMIDITY sensor
    curTime = millis();
    if ((*iter)->getSensorRelativeHumidityPeriod() != 0L &&
        curTime - (*iter)->getSensorRelativeHumidityPeriodPrv() >
            (*iter)->getSensorRelativeHumidityPeriod()) {
      sensors_event_t event;
      if ((*iter)->getSensorAmbientTemperature(&event)) {
        WS_DEBUG_PRINT("\tHumidity: ");
        WS_DEBUG_PRINT(event.temperature);
        WS_DEBUG_PRINTLN("%RH");

        // pack event data into msg
        fillEventMessage(
            &msgi2cResponse, event.relative_humidity,
            wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_RELATIVE_HUMIDITY);

        (*iter)->setSensorRelativeHumidityPeriodPrv(curTime);
      }
    }

    // Did this driver obtain data from sensors?
    if (msgi2cResponse.payload.resp_i2c_device_event.sensor_event_count == 0)
      continue;

    // Encode I2CDeviceEvent message
    if (!encodeI2CDeviceEventMsg(&msgi2cResponse,
                                 (*iter)->getSensorAddress())) {
      WS_DEBUG_PRINTLN("ERROR: Failed to encode I2CDeviceEvent!");
      continue;
    }

    // Publish I2CDeviceEvent
    if (!publishI2CDeviceEventMsg(&msgi2cResponse)) {
      WS_DEBUG_PRINTLN("ERROR: Failed to publish sensor event");
      continue;
    }
  }
}