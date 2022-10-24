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
 * Copyright (c) Brent Rubell 2021-2022 for Adafruit Industries.
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

#if defined(PIN_I2C_POWER)
  // turn on the I2C power by setting pin to opposite of 'rest state'
  pinMode(PIN_I2C_POWER, INPUT);
  delay(1);
  bool polarity = digitalRead(PIN_I2C_POWER);
  pinMode(PIN_I2C_POWER, OUTPUT);
  digitalWrite(PIN_I2C_POWER, !polarity);
#elif defined(TFT_I2C_POWER)
  // ADAFRUIT_FEATHER_ESP32S2_TFT
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
#elif defined(NEOPIXEL_I2C_POWER)
  // ADAFRUIT_FEATHER_ESP32_V2
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
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
    if (!_i2c->begin((int)msgInitRequest->i2c_pin_sda,
                     (int)msgInitRequest->i2c_pin_scl)) {
      _isInit = false; // if the peripheral was configured incorrectly
    } else {
      _isInit = true; // if the peripheral was configured incorrectly
    }
    _i2c->setClock(50000);
#elif defined(ARDUINO_ARCH_ESP8266)
    _i2c = new TwoWire();
    _i2c->begin(msgInitRequest->i2c_pin_sda, msgInitRequest->i2c_pin_scl);
    _i2c->setClock(50000);
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
  WS_DEBUG_PRINTLN("EXEC: I2C Scan");
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
  WS_DEBUG_PRINT("Attempting to initialize I2C device: ");
  WS_DEBUG_PRINTLN(msgDeviceInitReq->i2c_device_name);

  uint16_t i2cAddress = (uint16_t)msgDeviceInitReq->i2c_device_address;
  if (strcmp("aht20", msgDeviceInitReq->i2c_device_name) == 0) {
    _ahtx0 = new WipperSnapper_I2C_Driver_AHTX0(this->_i2c, i2cAddress);
    if (!_ahtx0->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize AHTX0 chip!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _ahtx0->configureDriver(msgDeviceInitReq);
    drivers.push_back(_ahtx0);
  } else if (strcmp("bme280", msgDeviceInitReq->i2c_device_name) == 0) {
    _bme280 = new WipperSnapper_I2C_Driver_BME280(this->_i2c, i2cAddress);
    if (!_bme280->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize BME280!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _bme280->configureDriver(msgDeviceInitReq);
    drivers.push_back(_bme280);
    WS_DEBUG_PRINTLN("BME280 Initialized Successfully!");
  } else if (strcmp("bme680", msgDeviceInitReq->i2c_device_name) == 0) {
    _bme680 = new WipperSnapper_I2C_Driver_BME680(this->_i2c, i2cAddress);
    if (!_bme680->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize BME680!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _bme680->configureDriver(msgDeviceInitReq);
    drivers.push_back(_bme680);
    WS_DEBUG_PRINTLN("BME680 Initialized Successfully!");
  } else if (strcmp("dps310", msgDeviceInitReq->i2c_device_name) == 0) {
    _dps310 = new WipperSnapper_I2C_Driver_DPS310(this->_i2c, i2cAddress);
    if (!_dps310->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize DPS310!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _dps310->configureDriver(msgDeviceInitReq);
    drivers.push_back(_dps310);
    WS_DEBUG_PRINTLN("DPS310 Initialized Successfully!");
  } else if (strcmp("scd30", msgDeviceInitReq->i2c_device_name) == 0) {
    _scd30 = new WipperSnapper_I2C_Driver_SCD30(this->_i2c, i2cAddress);
    if (!_scd30->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize SCD30!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _scd30->configureDriver(msgDeviceInitReq);
    drivers.push_back(_scd30);
    WS_DEBUG_PRINTLN("SCD30 Initialized Successfully!");
  } else if (strcmp("si7021", msgDeviceInitReq->i2c_device_name) == 0) {
    _si7021 = new WipperSnapper_I2C_Driver_SI7021(this->_i2c, i2cAddress);
    if (!_si7021->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize SI7021!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _si7021->configureDriver(msgDeviceInitReq);
    drivers.push_back(_si7021);
    WS_DEBUG_PRINTLN("SI7021 Initialized Successfully!");
  } else if (strcmp("mcp9808", msgDeviceInitReq->i2c_device_name) == 0) {
    _mcp9808 = new WipperSnapper_I2C_Driver_MCP9808(this->_i2c, i2cAddress);
    if (!_mcp9808->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize MCP9808!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _mcp9808->configureDriver(msgDeviceInitReq);
    drivers.push_back(_mcp9808);
    WS_DEBUG_PRINTLN("MCP9808 Initialized Successfully!");
  } else if (strcmp("mcp9601", msgDeviceInitReq->i2c_device_name) == 0) {
    _mcp9601 = new WipperSnapper_I2C_Driver_MCP9601(this->_i2c, i2cAddress);
    if (!_mcp9601->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize MCP9601!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _mcp9601->configureDriver(msgDeviceInitReq);
    drivers.push_back(_mcp9601);
    WS_DEBUG_PRINTLN("MCP9601 Initialized Successfully!");
  } else if (strcmp("tsl2591", msgDeviceInitReq->i2c_device_name) == 0) {
    _tsl2591 = new WipperSnapper_I2C_Driver_TSL2591(this->_i2c, i2cAddress);
    if (!_tsl2591->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize TSL2591!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _tsl2591->configureDriver(msgDeviceInitReq);
    drivers.push_back(_tsl2591);
    WS_DEBUG_PRINTLN("TSL2591 Initialized Successfully!");
  } else if (strcmp("veml7700", msgDeviceInitReq->i2c_device_name) == 0) {
    _veml7700 = new WipperSnapper_I2C_Driver_VEML7700(this->_i2c, i2cAddress);
    if (!_veml7700->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize VEML7700!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _veml7700->configureDriver(msgDeviceInitReq);
    drivers.push_back(_veml7700);
    WS_DEBUG_PRINTLN("VEML7700 Initialized Successfully!");
  } else if (strcmp("scd40", msgDeviceInitReq->i2c_device_name) == 0) {
    _scd40 = new WipperSnapper_I2C_Driver_SCD40(this->_i2c, i2cAddress);
    if (!_scd40->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize SCD40!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _scd40->configureDriver(msgDeviceInitReq);
    drivers.push_back(_scd40);
    WS_DEBUG_PRINTLN("SCD40 Initialized Successfully!");
  } else if (strcmp("sht40", msgDeviceInitReq->i2c_device_name) == 0) {
    _sht4x = new WipperSnapper_I2C_Driver_SHT4X(this->_i2c, i2cAddress);
    if (!_sht4x->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize sht4x!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _sht4x->configureDriver(msgDeviceInitReq);
    drivers.push_back(_sht4x);
    WS_DEBUG_PRINTLN("SHT4X Initialized Successfully!");
  } else if (strcmp("sht3x", msgDeviceInitReq->i2c_device_name) == 0) {
    _sht3x = new WipperSnapper_I2C_Driver_SHT3X(this->_i2c, i2cAddress);
    if (!_sht3x->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize sht3x!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _sht3x->configureDriver(msgDeviceInitReq);
    drivers.push_back(_sht3x);
    WS_DEBUG_PRINTLN("SHT3X Initialized Successfully!");
  } else if (strcmp("shtc3", msgDeviceInitReq->i2c_device_name) == 0) {
    _shtc3 = new WipperSnapper_I2C_Driver_SHTC3(this->_i2c, i2cAddress);
    if (!_shtc3->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize SHTC3!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _shtc3->configureDriver(msgDeviceInitReq);
    drivers.push_back(_shtc3);
    WS_DEBUG_PRINTLN("SHTC3 Initialized Successfully!");
  } else if (strcmp("pmsa003i", msgDeviceInitReq->i2c_device_name) == 0) {
    _pm25 = new WipperSnapper_I2C_Driver_PM25(this->_i2c, i2cAddress);
    if (!_pm25->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize PM2.5 AQI Sensor!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _pm25->configureDriver(msgDeviceInitReq);
    drivers.push_back(_pm25);
    WS_DEBUG_PRINTLN("PM2.5 AQI Sensor Initialized Successfully!");
  } else if (strcmp("lc709203f", msgDeviceInitReq->i2c_device_name) == 0) {
    _lc = new WipperSnapper_I2C_Driver_LC709203F(this->_i2c, i2cAddress);
    if (!_lc->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize LC709203F Sensor!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _lc->configureDriver(msgDeviceInitReq);
    drivers.push_back(_lc);
    WS_DEBUG_PRINTLN("LC709203F Sensor Initialized Successfully!");
  } else if (strcmp("stemma_soil", msgDeviceInitReq->i2c_device_name) == 0) {
    _ss =
        new WipperSnapper_I2C_Driver_STEMMA_Soil_Sensor(this->_i2c, i2cAddress);
    if (!_ss->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize STEMMA Soil Sensor!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _ss->configureDriver(msgDeviceInitReq);
    drivers.push_back(_ss);
    WS_DEBUG_PRINTLN("STEMMA Soil Sensor Initialized Successfully!");
  } else {
    WS_DEBUG_PRINTLN("ERROR: I2C device type not found!")
    _busStatusResponse =
        wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_UNSUPPORTED_SENSOR;
    return false;
  }
  _busStatusResponse = wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_SUCCESS;
  return true;
}

/*********************************************************************************/
/*!
    @brief    Updates the properties of an I2C device driver.
    @param    msgDeviceUpdateReq
              A decoded I2CDeviceUpdateRequest.
*/
/*********************************************************************************/
void WipperSnapper_Component_I2C::updateI2CDeviceProperties(
    wippersnapper_i2c_v1_I2CDeviceUpdateRequest *msgDeviceUpdateReq) {
  uint16_t i2cAddress = (uint16_t)msgDeviceUpdateReq->i2c_device_address;

  // Loop thru vector of drivers to find the unique address
  for (int i = 0; i < drivers.size(); i++) {
    if (drivers[i]->getI2CAddress() == i2cAddress) {
      // Update the properties of each driver
      for (int j = 0; j < msgDeviceUpdateReq->i2c_device_properties_count;
           j++) {
        drivers[i]->setSensorPeriod(
            msgDeviceUpdateReq->i2c_device_properties[j].sensor_period,
            msgDeviceUpdateReq->i2c_device_properties[j].sensor_type);
      }
    }
  }

  // set response OK
  _busStatusResponse = wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_SUCCESS;
}

/*******************************************************************************/
/*!
    @brief    Deinitializes and deletes an I2C device driver object.
    @param    msgDeviceDeinitReq
              A decoded I2CDeviceDeinitRequest.
*/
/*******************************************************************************/
void WipperSnapper_Component_I2C::deinitI2CDevice(
    wippersnapper_i2c_v1_I2CDeviceDeinitRequest *msgDeviceDeinitReq) {
  uint16_t deviceAddr = (uint16_t)msgDeviceDeinitReq->i2c_device_address;
  std::vector<WipperSnapper_I2C_Driver *>::iterator iter, end;

  for (iter = drivers.begin(), end = drivers.end(); iter != end; ++iter) {
    if ((*iter)->getI2CAddress() == deviceAddr) {
      // Delete the object that iter points to
      // delete *iter;
      *iter = nullptr;
// ESP-IDF, Erase–remove iter ptr from driver vector
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
      *iter = nullptr;
      drivers.erase(remove(drivers.begin(), drivers.end(), nullptr),
                    drivers.end());
#else
      // Arduino can not erase-remove, erase only
      drivers.erase(iter);
#endif
      WS_DEBUG_PRINTLN("I2C Device De-initialized!");
    }
  }
  _busStatusResponse = wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_SUCCESS;
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
bool WipperSnapper_Component_I2C::encodePublishI2CDeviceEventMsg(
    wippersnapper_signal_v1_I2CResponse *msgi2cResponse,
    uint32_t sensorAddress) {
  // Encode I2CResponse msg
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

  // Publish I2CResponse msg
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
*/
/*******************************************************************************/
void WipperSnapper_Component_I2C::fillEventMessage(
    wippersnapper_signal_v1_I2CResponse *msgi2cResponse, float value,
    wippersnapper_i2c_v1_SensorType sensorType) {
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

    // Event struct
    sensors_event_t event;

    // AMBIENT_TEMPERATURE sensor (°C)
    curTime = millis();
    if ((*iter)->getSensorAmbientTempPeriod() != 0L &&
        curTime - (*iter)->getSensorAmbientTempPeriodPrv() >
            (*iter)->getSensorAmbientTempPeriod()) {
      if ((*iter)->getEventAmbientTemp(&event)) {
        WS_DEBUG_PRINT("Sensor 0x");
        WS_DEBUG_PRINTHEX((*iter)->getI2CAddress());
        WS_DEBUG_PRINTLN("");
        WS_DEBUG_PRINT("\tTemperature: ");
        WS_DEBUG_PRINT(event.temperature);
        WS_DEBUG_PRINTLN(" degrees C");

        // pack event data into msg
        fillEventMessage(
            &msgi2cResponse, event.temperature,
            wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE);

        (*iter)->setSensorAmbientTempPeriodPrv(curTime);
      } else {
        WS_DEBUG_PRINTLN(
            "ERROR: Failed to get ambient temperature sensor reading!");
      }
    }

    // Ambient Temperature sensor (°F)
    curTime = millis();
    if ((*iter)->getSensorAmbientTempFPeriod() != 0L &&
        curTime - (*iter)->getSensorAmbientTempFPeriodPrv() >
            (*iter)->getSensorAmbientTempFPeriod()) {
      if ((*iter)->getEventAmbientTempF(&event)) {
        WS_DEBUG_PRINT("Sensor 0x");
        WS_DEBUG_PRINTHEX((*iter)->getI2CAddress());
        WS_DEBUG_PRINTLN("");
        WS_DEBUG_PRINT("\tAmbient Temp.: ");
        WS_DEBUG_PRINT(event.temperature);
        WS_DEBUG_PRINTLN("°F");

        (*iter)->setSensorAmbientTempFPeriodPrv(curTime);

        fillEventMessage(
            &msgi2cResponse, event.temperature,
            wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE_FAHRENHEIT);
      } else {
        WS_DEBUG_PRINTLN(
            "ERROR: Failed to obtain ambient temp. (°F)) sensor reading!");
      }
    }

    // OBJECT_TEMPERATURE sensor (°C)
    curTime = millis();
    if ((*iter)->getSensorObjectTempPeriod() != 0L &&
        curTime - (*iter)->getSensorObjectTempPeriodPrv() >
            (*iter)->getSensorObjectTempPeriod()) {
      if ((*iter)->getEventObjectTemp(&event)) {
        WS_DEBUG_PRINT("Sensor 0x");
        WS_DEBUG_PRINTHEX((*iter)->getI2CAddress());
        WS_DEBUG_PRINTLN("");
        WS_DEBUG_PRINT("\tTemperature: ");
        WS_DEBUG_PRINT(event.temperature);
        WS_DEBUG_PRINTLN("°C");

        // pack event data into msg
        fillEventMessage(
            &msgi2cResponse, event.temperature,
            wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_OBJECT_TEMPERATURE);

        (*iter)->setSensorObjectTempPeriodPrv(curTime);
      } else {
        WS_DEBUG_PRINTLN(
            "ERROR: Failed to get object temperature sensor (°C) reading!");
      }
    }

    // OBJECT_TEMPERATURE sensor (°F)
    curTime = millis();
    if ((*iter)->getSensorObjectTempFPeriod() != 0L &&
        curTime - (*iter)->getSensorObjectTempFPeriodPrv() >
            (*iter)->getSensorObjectTempFPeriod()) {
      if ((*iter)->getEventObjectTempF(&event)) {
        WS_DEBUG_PRINT("Sensor 0x");
        WS_DEBUG_PRINTHEX((*iter)->getI2CAddress());
        WS_DEBUG_PRINTLN("");
        WS_DEBUG_PRINT("\tTemperature: ");
        WS_DEBUG_PRINT(event.temperature);
        WS_DEBUG_PRINTLN("°F");

        // pack event data into msg
        fillEventMessage(
            &msgi2cResponse, event.temperature,
            wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_OBJECT_TEMPERATURE_FAHRENHEIT);

        (*iter)->setSensorObjectTempFPeriodPrv(curTime);
      } else {
        WS_DEBUG_PRINTLN(
            "ERROR: Failed to get object temperature sensor (°F) reading!");
      }
    }

    // RELATIVE_HUMIDITY sensor
    curTime = millis();
    if ((*iter)->getSensorRelativeHumidityPeriod() != 0L &&
        curTime - (*iter)->getSensorRelativeHumidityPeriodPrv() >
            (*iter)->getSensorRelativeHumidityPeriod()) {
      if ((*iter)->getEventRelativeHumidity(&event)) {
        WS_DEBUG_PRINT("Sensor 0x");
        WS_DEBUG_PRINTHEX((*iter)->getI2CAddress());
        WS_DEBUG_PRINTLN("");
        WS_DEBUG_PRINT("\tHumidity: ");
        WS_DEBUG_PRINT(event.relative_humidity);
        WS_DEBUG_PRINTLN("%RH");

        // pack event data into msg
        fillEventMessage(
            &msgi2cResponse, event.relative_humidity,
            wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_RELATIVE_HUMIDITY);

        (*iter)->setSensorRelativeHumidityPeriodPrv(curTime);
      } else {
        WS_DEBUG_PRINTLN("ERROR: Failed to get humidity sensor reading!");
      }
    }

    // PRESSURE sensor
    curTime = millis();
    if ((*iter)->getSensorPressurePeriod() != 0L &&
        curTime - (*iter)->getSensorPressurePeriodPrv() >
            (*iter)->getSensorPressurePeriod()) {
      if ((*iter)->getEventPressure(&event)) {
        WS_DEBUG_PRINT("Sensor 0x");
        WS_DEBUG_PRINTHEX((*iter)->getI2CAddress());
        WS_DEBUG_PRINTLN("");
        WS_DEBUG_PRINT("\tPressure: ");
        WS_DEBUG_PRINT(event.pressure);
        WS_DEBUG_PRINTLN(" hPa");

        // pack event data into msg
        fillEventMessage(&msgi2cResponse, event.pressure,
                         wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PRESSURE);

        (*iter)->setSensorPressurePeriodPrv(curTime);
      } else {
        WS_DEBUG_PRINTLN("ERROR: Failed to get Pressure sensor reading!");
      }
    }

    // CO2 sensor
    curTime = millis();
    if ((*iter)->getSensorCO2Period() != 0L &&
        curTime - (*iter)->getSensorCO2PeriodPrv() >
            (*iter)->getSensorCO2Period()) {
      if ((*iter)->getEventCO2(&event)) {
        WS_DEBUG_PRINT("Sensor 0x");
        WS_DEBUG_PRINTHEX((*iter)->getI2CAddress());
        WS_DEBUG_PRINTLN("");
        WS_DEBUG_PRINT("\tCO2: ");
        WS_DEBUG_PRINT(event.data[0]);
        WS_DEBUG_PRINTLN(" ppm");

        fillEventMessage(&msgi2cResponse, event.data[0],
                         wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_CO2);
        (*iter)->setSensorCO2PeriodPrv(curTime);
      } else {
        WS_DEBUG_PRINTLN("ERROR: Failed to obtain CO2 sensor reading!");
      }
    }

    // Altitude sensor
    curTime = millis();
    if ((*iter)->getSensorAltitudePeriod() != 0L &&
        curTime - (*iter)->getSensorAltitudePeriodPrv() >
            (*iter)->getSensorAltitudePeriod()) {
      if ((*iter)->getEventAltitude(&event)) {
        WS_DEBUG_PRINT("Sensor 0x");
        WS_DEBUG_PRINTHEX((*iter)->getI2CAddress());
        WS_DEBUG_PRINTLN("");
        WS_DEBUG_PRINT("\tAltitude: ");
        WS_DEBUG_PRINT(event.data[0]);
        WS_DEBUG_PRINTLN(" m");

        // pack event data into msg
        fillEventMessage(&msgi2cResponse, event.data[0],
                         wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_ALTITUDE);

        (*iter)->setSensorAltitudePeriodPrv(curTime);
      } else {
        WS_DEBUG_PRINTLN("ERROR: Failed to get altitude sensor reading!");
      }
    }

    // Light sensor
    curTime = millis();
    if ((*iter)->getSensorLightPeriod() != 0L &&
        curTime - (*iter)->getSensorLightPeriodPrv() >
            (*iter)->getSensorLightPeriod()) {
      if ((*iter)->getEventLight(&event)) {
        WS_DEBUG_PRINT("Sensor 0x");
        WS_DEBUG_PRINTHEX((*iter)->getI2CAddress());
        WS_DEBUG_PRINTLN("");
        WS_DEBUG_PRINT("\tLight: ");
        WS_DEBUG_PRINT(event.light);
        WS_DEBUG_PRINTLN(" lux");

        // pack event data into msg
        fillEventMessage(&msgi2cResponse, event.light,
                         wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_LIGHT);

        (*iter)->setSensorLightPeriodPrv(curTime);
      } else {
        WS_DEBUG_PRINTLN("ERROR: Failed to get light sensor reading!");
      }
    }

    // PM10_STD sensor
    curTime = millis();
    if ((*iter)->getSensorPM10_STDPeriod() != 0L &&
        curTime - (*iter)->getSensorPM10_STDPeriodPrv() >
            (*iter)->getSensorPM10_STDPeriod()) {
      if ((*iter)->getEventPM10_STD(&event)) {
        WS_DEBUG_PRINT("Sensor 0x");
        WS_DEBUG_PRINTHEX((*iter)->getI2CAddress());
        WS_DEBUG_PRINTLN("");
        WS_DEBUG_PRINT("\tPM1.0: ");
        WS_DEBUG_PRINT(event.data[0]);
        WS_DEBUG_PRINTLN(" ppm");

        // pack event data into msg
        fillEventMessage(&msgi2cResponse, event.data[0],
                         wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM10_STD);
      } else {
        WS_DEBUG_PRINTLN("ERROR: Failed to get PM1.0 sensor reading!");
      }
      // try again in curTime seconds
      (*iter)->setSensorPM10_STDPeriodPrv(curTime);
    }

    // PM25_STD sensor
    curTime = millis();
    if ((*iter)->getSensorPM25_STDPeriod() != 0L &&
        curTime - (*iter)->getSensorPM25_STDPeriodPrv() >
            (*iter)->getSensorPM25_STDPeriod()) {
      if ((*iter)->getEventPM25_STD(&event)) {
        WS_DEBUG_PRINT("Sensor 0x");
        WS_DEBUG_PRINTHEX((*iter)->getI2CAddress());
        WS_DEBUG_PRINTLN("");
        WS_DEBUG_PRINT("\tPM2.5: ");
        WS_DEBUG_PRINT(event.data[0]);
        WS_DEBUG_PRINTLN(" ppm");

        // pack event data into msg
        fillEventMessage(&msgi2cResponse, event.data[0],
                         wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM25_STD);
      } else {
        WS_DEBUG_PRINTLN("ERROR: Failed to get PM2.5 sensor reading!");
      }
      // try again in curTime seconds
      (*iter)->setSensorPM25_STDPeriodPrv(curTime);
    }

    // PM100_STD sensor
    curTime = millis();
    if ((*iter)->getSensorPM100_STDPeriod() != 0L &&
        curTime - (*iter)->getSensorPM100_STDPeriodPrv() >
            (*iter)->getSensorPM100_STDPeriod()) {
      if ((*iter)->getEventPM100_STD(&event)) {
        WS_DEBUG_PRINT("Sensor 0x");
        WS_DEBUG_PRINTHEX((*iter)->getI2CAddress());
        WS_DEBUG_PRINTLN("");
        WS_DEBUG_PRINT("\tPM100: ");
        WS_DEBUG_PRINT(event.data[0]);
        WS_DEBUG_PRINTLN(" ppm");

        // pack event data into msg
        fillEventMessage(&msgi2cResponse, event.data[0],
                         wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM100_STD);
      } else {
        WS_DEBUG_PRINTLN("ERROR: Failed to get PM10.0 sensor reading!");
      }
      (*iter)->setSensorPM100_STDPeriodPrv(
          curTime); // try again in curTime seconds
    }

    // Voltage sensor
    curTime = millis();
    if ((*iter)->getSensorVoltagePeriod() != 0L &&
        curTime - (*iter)->getSensorVoltagePeriodPrv() >
            (*iter)->getSensorVoltagePeriod()) {
      if ((*iter)->getEventVoltage(&event)) {
        WS_DEBUG_PRINT("Sensor 0x");
        WS_DEBUG_PRINTHEX((*iter)->getI2CAddress());
        WS_DEBUG_PRINTLN("");
        WS_DEBUG_PRINT("\tVoltage: ");
        WS_DEBUG_PRINT(event.voltage);
        WS_DEBUG_PRINTLN(" v");

        // pack event data into msg
        fillEventMessage(&msgi2cResponse, event.voltage,
                         wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_VOLTAGE);
      } else {
        WS_DEBUG_PRINTLN("ERROR: Failed to get voltage sensor reading!");
      }
      // try again in curTime seconds
      (*iter)->setSensorVoltagePeriodPrv(curTime);
    }

    // Unitless % sensor
    curTime = millis();
    if ((*iter)->getSensorUnitlessPercentPeriod() != 0L &&
        curTime - (*iter)->getSensorUnitlessPercentPeriodPrv() >
            (*iter)->getSensorUnitlessPercentPeriod()) {
      if ((*iter)->getEventUnitlessPercent(&event)) {
        WS_DEBUG_PRINT("Sensor 0x");
        WS_DEBUG_PRINTHEX((*iter)->getI2CAddress());
        WS_DEBUG_PRINTLN("");
        WS_DEBUG_PRINT("\tRead: ");
        WS_DEBUG_PRINT(event.data[0]);
        WS_DEBUG_PRINTLN(" %");

        // pack event data into msg
        fillEventMessage(
            &msgi2cResponse, event.voltage,
            wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_UNITLESS_PERCENT);
      } else {
        WS_DEBUG_PRINTLN(
            "ERROR: Failed to get unitless percentage sensor reading!");
      }
      // try again in curTime seconds
      (*iter)->setSensorUnitlessPercentPeriodPrv(curTime);
    }

    // Raw sensor
    curTime = millis();
    if ((*iter)->getSensorRawPeriod() != 0L &&
        curTime - (*iter)->getSensorRawPeriodPrv() >
            (*iter)->getSensorRawPeriod()) {
      if ((*iter)->getEventRaw(&event)) {
        WS_DEBUG_PRINT("Sensor 0x");
        WS_DEBUG_PRINTHEX((*iter)->getI2CAddress());
        WS_DEBUG_PRINTLN("");
        WS_DEBUG_PRINT("\tRaw: ");
        WS_DEBUG_PRINTLN(event.data[0]);

        fillEventMessage(&msgi2cResponse, event.data[0],
                         wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_RAW);
      } else {
        WS_DEBUG_PRINTLN("ERROR: Failed to obtain Raw sensor reading!");
      }
      (*iter)->setSensorRawPeriodPrv(curTime);
    }

    // Gas sensor
    curTime = millis();
    if ((*iter)->getSensorGasResistancePeriod() != 0L &&
        curTime - (*iter)->getSensorGasResistancePeriodPrv() >
            (*iter)->getSensorGasResistancePeriod()) {
      if ((*iter)->getEventGasResistance(&event)) {
        WS_DEBUG_PRINT("Sensor 0x");
        WS_DEBUG_PRINTHEX((*iter)->getI2CAddress());
        WS_DEBUG_PRINTLN("");
        WS_DEBUG_PRINT("\tGas Resistance: ");
        WS_DEBUG_PRINT(event.data[0]);
        WS_DEBUG_PRINT(" ohms");

        fillEventMessage(
            &msgi2cResponse, event.data[0],
            wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_GAS_RESISTANCE);
      } else {
        WS_DEBUG_PRINTLN(
            "ERROR: Failed to obtain gas resistance sensor reading!");
      }
      (*iter)->setSensorGasResistancePeriodPrv(curTime);
    }

    // Did this driver obtain data from sensors?
    if (msgi2cResponse.payload.resp_i2c_device_event.sensor_event_count == 0)
      continue;

    // Encode and publish I2CDeviceEvent message
    if (!encodePublishI2CDeviceEventMsg(&msgi2cResponse,
                                        (*iter)->getI2CAddress())) {
      WS_DEBUG_PRINTLN("ERROR: Failed to encode and publish I2CDeviceEvent!");
      continue;
    }
  }
}
