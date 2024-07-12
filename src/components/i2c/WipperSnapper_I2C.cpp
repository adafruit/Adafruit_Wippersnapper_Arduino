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

#ifdef ARDUINO_ARCH_RP2040
// Wire uses GPIO4 (SDA) and GPIO5 (SCL) automatically.
#define WIRE Wire
#endif

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
#elif defined(ARDUINO_ARCH_RP2040)
    _i2c = &WIRE;
    _i2c->begin();
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
  if ((strcmp("aht20", msgDeviceInitReq->i2c_device_name) == 0) ||
      (strcmp("am2301b", msgDeviceInitReq->i2c_device_name) == 0) ||
      (strcmp("am2315c", msgDeviceInitReq->i2c_device_name) == 0) ||
      (strcmp("dht20", msgDeviceInitReq->i2c_device_name) == 0)) {
    _ahtx0 = new WipperSnapper_I2C_Driver_AHTX0(this->_i2c, i2cAddress);
    if (!_ahtx0->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize AHTX0 chip!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _ahtx0->configureDriver(msgDeviceInitReq);
    drivers.push_back(_ahtx0);
  } else if (strcmp("bh1750", msgDeviceInitReq->i2c_device_name) == 0) {
    _bh1750 = new WipperSnapper_I2C_Driver_BH1750(this->_i2c, i2cAddress);
    if (!_bh1750->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize BH1750 chip!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _bh1750->configureDriver(msgDeviceInitReq);
    drivers.push_back(_bh1750);
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
  } else if (strcmp("bmp280", msgDeviceInitReq->i2c_device_name) == 0) {
    _bmp280 = new WipperSnapper_I2C_Driver_BMP280(this->_i2c, i2cAddress);
    if (!_bmp280->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize BMP280!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _bmp280->configureDriver(msgDeviceInitReq);
    drivers.push_back(_bmp280);
    WS_DEBUG_PRINTLN("BMP280 Initialized Successfully!");
  } else if ((strcmp("bmp388", msgDeviceInitReq->i2c_device_name) == 0) ||
             (strcmp("bmp390", msgDeviceInitReq->i2c_device_name) == 0)) {
    _bmp3xx = new WipperSnapper_I2C_Driver_BMP3XX(this->_i2c, i2cAddress);
    if (!_bmp3xx->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize BMP388!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _bmp3xx->configureDriver(msgDeviceInitReq);
    drivers.push_back(_bmp3xx);
    WS_DEBUG_PRINTLN("BMP388 Initialized Successfully!");
  } else if ((strcmp("bme680", msgDeviceInitReq->i2c_device_name) == 0) ||
             (strcmp("bme688", msgDeviceInitReq->i2c_device_name) == 0)) {
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
  } else if (strcmp("ens160", msgDeviceInitReq->i2c_device_name) == 0) {
    _ens160 = new WipperSnapper_I2C_Driver_ENS160(this->_i2c, i2cAddress);
    if (!_ens160->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize ENS160!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _ens160->configureDriver(msgDeviceInitReq);
    drivers.push_back(_ens160);
    WS_DEBUG_PRINTLN("ENS160 Initialized Successfully!");
  } else if (strcmp("hts221", msgDeviceInitReq->i2c_device_name) == 0) {
    _hts221 = new WipperSnapper_I2C_Driver_HTS221(this->_i2c, i2cAddress);
    if (!_hts221->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize HTS221!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _hts221->configureDriver(msgDeviceInitReq);
    drivers.push_back(_hts221);
    WS_DEBUG_PRINTLN("HTS221 Initialized Successfully!");
  } else if (strcmp("htu21d", msgDeviceInitReq->i2c_device_name) == 0) {
    _htu21d = new WipperSnapper_I2C_Driver_HTU21D(this->_i2c, i2cAddress);
    if (!_htu21d->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize HTU21D!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _htu21d->configureDriver(msgDeviceInitReq);
    drivers.push_back(_htu21d);
    WS_DEBUG_PRINTLN("HTU21D Initialized Successfully!");
  } else if (strcmp("htu31d", msgDeviceInitReq->i2c_device_name) == 0) {
    _htu31d = new WipperSnapper_I2C_Driver_HTU31D(this->_i2c, i2cAddress);
    if (!_htu31d->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize HTU31D!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _htu31d->configureDriver(msgDeviceInitReq);
    drivers.push_back(_htu31d);
    WS_DEBUG_PRINTLN("HTU31D Initialized Successfully!");
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
  } else if (strcmp("ina219", msgDeviceInitReq->i2c_device_name) == 0) {
    _ina219 = new WipperSnapper_I2C_Driver_INA219(this->_i2c, i2cAddress);
    if (!_ina219->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize INA219");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _ina219->configureDriver(msgDeviceInitReq);
    drivers.push_back(_ina219);
    WS_DEBUG_PRINTLN("INA219 Initialized Successfully!");
  } else if (strcmp("ltr390", msgDeviceInitReq->i2c_device_name) == 0) {
    _ltr390 = new WipperSnapper_I2C_Driver_LTR390(this->_i2c, i2cAddress);
    if (!_ltr390->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize LTR390");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _ltr390->configureDriver(msgDeviceInitReq);
    drivers.push_back(_ltr390);
    WS_DEBUG_PRINTLN("LTR390 Initialized Successfully!");
  } else if ((strcmp("ltr329", msgDeviceInitReq->i2c_device_name) == 0) ||
             (strcmp("ltr303", msgDeviceInitReq->i2c_device_name) == 0)) {
    _ltr329 =
        new WipperSnapper_I2C_Driver_LTR329_LTR303(this->_i2c, i2cAddress);
    if (!_ltr329->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize LTR329/303");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _ltr329->configureDriver(msgDeviceInitReq);
    drivers.push_back(_ltr329);
    WS_DEBUG_PRINTLN("LTR329/303 Initialized Successfully!");
  } else if (strcmp("nau7802", msgDeviceInitReq->i2c_device_name) == 0) {
    _nau7802 = new WipperSnapper_I2C_Driver_NAU7802(this->_i2c, i2cAddress);
    if (!_nau7802->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize NAU7802");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _nau7802->configureDriver(msgDeviceInitReq);
    drivers.push_back(_nau7802);
    WS_DEBUG_PRINTLN("NAU7802 Initialized Successfully!");
  } else if (strcmp("sgp30", msgDeviceInitReq->i2c_device_name) == 0) {
    _sgp30 = new WipperSnapper_I2C_Driver_SGP30(this->_i2c, i2cAddress);
    if (!_sgp30->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize SGP30!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _sgp30->configureDriver(msgDeviceInitReq);
    drivers.push_back(_sgp30);
    WS_DEBUG_PRINTLN("SGP30 Initialized Successfully!");
  } else if (strcmp("sgp40", msgDeviceInitReq->i2c_device_name) == 0) {
    _sgp40 = new WipperSnapper_I2C_Driver_SGP40(this->_i2c, i2cAddress);
    if (!_sgp40->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize SGP40!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _sgp40->configureDriver(msgDeviceInitReq);
    drivers.push_back(_sgp40);
    WS_DEBUG_PRINTLN("SGP40 Initialized Successfully!");
  } else if ((strcmp("sht20", msgDeviceInitReq->i2c_device_name) == 0) ||
             (strcmp("si7021", msgDeviceInitReq->i2c_device_name) == 0)) {
    _si7021 = new WipperSnapper_I2C_Driver_SI7021(this->_i2c, i2cAddress);
    if (!_si7021->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize SI7021/SHT20!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _si7021->configureDriver(msgDeviceInitReq);
    drivers.push_back(_si7021);
    WS_DEBUG_PRINTLN("SI7021/SHT20 Initialized Successfully!");
  } else if (strcmp("mcp3421", msgDeviceInitReq->i2c_device_name) == 0) {
    _mcp3421 = new WipperSnapper_I2C_Driver_MCP3421(this->_i2c, i2cAddress);
    if (!_mcp3421->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize MCP3421!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _mcp3421->configureDriver(msgDeviceInitReq);
    drivers.push_back(_mcp3421);
    WS_DEBUG_PRINTLN("MCP3421 Initialized Successfully!");
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
  } else if (strcmp("mpl115a2", msgDeviceInitReq->i2c_device_name) == 0) {
    _mpl115a2 = new WipperSnapper_I2C_Driver_MPL115A2(this->_i2c, i2cAddress);
    if (!_mpl115a2->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize MPL115A2!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _mpl115a2->configureDriver(msgDeviceInitReq);
    drivers.push_back(_mpl115a2);
    WS_DEBUG_PRINTLN("MPL115A2 Initialized Successfully!");
  } else if (strcmp("mprls", msgDeviceInitReq->i2c_device_name) == 0) {
    _mprls = new WipperSnapper_I2C_Driver_MPRLS(this->_i2c, i2cAddress);
    if (!_mprls->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize MPRLS!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _mprls->configureDriver(msgDeviceInitReq);
    drivers.push_back(_mprls);
    WS_DEBUG_PRINTLN("MPRLS Initialized Successfully!");
  } else if (strcmp("ms8607", msgDeviceInitReq->i2c_device_name) == 0) {
    _ms8607 = new WipperSnapper_I2C_Driver_MS8607(this->_i2c, i2cAddress);
    if (!_ms8607->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize MS8607!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _ms8607->configureDriver(msgDeviceInitReq);
    drivers.push_back(_ms8607);
    WS_DEBUG_PRINTLN("MS8607 Initialized Successfully!");
  } else if (strcmp("tmp117", msgDeviceInitReq->i2c_device_name) == 0) {
    _tmp117 = new WipperSnapper_I2C_Driver_TMP117(this->_i2c, i2cAddress);
    if (!_tmp117->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize TMP117!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _tmp117->configureDriver(msgDeviceInitReq);
    drivers.push_back(_tmp117);
    WS_DEBUG_PRINTLN("TMP117 Initialized Successfully!");
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
  } else if (strcmp("vcnl4020", msgDeviceInitReq->i2c_device_name) == 0) {
    _vcnl4020 = new WipperSnapper_I2C_Driver_VCNL4020(this->_i2c, i2cAddress);
    if (!_vcnl4020->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize VCNL4020!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _vcnl4020->configureDriver(msgDeviceInitReq);
    drivers.push_back(_vcnl4020);
    WS_DEBUG_PRINTLN("VCNL4020 Initialized Successfully!");
  } else if (strcmp("vcnl4040", msgDeviceInitReq->i2c_device_name) == 0) {
    _vcnl4040 = new WipperSnapper_I2C_Driver_VCNL4040(this->_i2c, i2cAddress);
    if (!_vcnl4040->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize VCNL4040!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _vcnl4040->configureDriver(msgDeviceInitReq);
    drivers.push_back(_vcnl4040);
    WS_DEBUG_PRINTLN("VCNL4040 Initialized Successfully!");
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
    _scd40 = new WipperSnapper_I2C_Driver_SCD4X(this->_i2c, i2cAddress);
    if (!_scd40->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize SCD4x!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _scd40->configureDriver(msgDeviceInitReq);
    drivers.push_back(_scd40);
    WS_DEBUG_PRINTLN("SCD4x Initialized Successfully!");
  } else if ((strcmp("sen5x", msgDeviceInitReq->i2c_device_name) == 0) ||
             (strcmp("sen55", msgDeviceInitReq->i2c_device_name) == 0) ||
             (strcmp("sen54", msgDeviceInitReq->i2c_device_name) == 0) ||
             (strcmp("sen50", msgDeviceInitReq->i2c_device_name) == 0)) {
    _sen5x = new WipperSnapper_I2C_Driver_SEN5X(this->_i2c, i2cAddress);
    if (!_sen5x->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize SEN5X!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _sen5x->configureDriver(msgDeviceInitReq);
    drivers.push_back(_sen5x);
    WS_DEBUG_PRINTLN("SEN5X Initialized Successfully!");
  } else if ((strcmp("sht40", msgDeviceInitReq->i2c_device_name) == 0) ||
             (strcmp("sht41", msgDeviceInitReq->i2c_device_name) == 0) ||
             (strcmp("sht45", msgDeviceInitReq->i2c_device_name) == 0)) {
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
  } else if ((strcmp("sht3x", msgDeviceInitReq->i2c_device_name) == 0) ||
             (strcmp("sht30_shell", msgDeviceInitReq->i2c_device_name) == 0) ||
             (strcmp("sht30_mesh", msgDeviceInitReq->i2c_device_name) == 0)) {
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
  } else if ((strcmp("pct2075", msgDeviceInitReq->i2c_device_name) == 0) ||
             (strcmp("tc74a0", msgDeviceInitReq->i2c_device_name) == 0)) {
    _pct2075 = new WipperSnapper_I2C_Driver_PCT2075(this->_i2c, i2cAddress);
    if (!_pct2075->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize PCT2075 Temp Sensor!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _pct2075->configureDriver(msgDeviceInitReq);
    drivers.push_back(_pct2075);
    WS_DEBUG_PRINTLN("PCT2075 Temp Sensor Initialized Successfully!");
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
  } else if (strcmp("lps22hb", msgDeviceInitReq->i2c_device_name) == 0) {
    _lps22hb = new WipperSnapper_I2C_Driver_LPS22HB(this->_i2c, i2cAddress);
    if (!_lps22hb->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize LPS22HB Sensor!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _lps22hb->configureDriver(msgDeviceInitReq);
    drivers.push_back(_lps22hb);
    WS_DEBUG_PRINTLN("LPS22HB Sensor Initialized Successfully!");
  } else if (strcmp("lps25hb", msgDeviceInitReq->i2c_device_name) == 0) {
    _lps25hb = new WipperSnapper_I2C_Driver_LPS25HB(this->_i2c, i2cAddress);
    if (!_lps25hb->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize LPS25HB Sensor!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _lps25hb->configureDriver(msgDeviceInitReq);
    drivers.push_back(_lps25hb);
    WS_DEBUG_PRINTLN("LPS25HB Sensor Initialized Successfully!");
  } else if ((strcmp("lps33hw", msgDeviceInitReq->i2c_device_name) == 0) ||
             (strcmp("lps35hw", msgDeviceInitReq->i2c_device_name)) == 0) {
    _lps3xhw = new WipperSnapper_I2C_Driver_LPS3XHW(this->_i2c, i2cAddress);
    if (!_lps3xhw->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize LPS3XHW Sensor!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _lps3xhw->configureDriver(msgDeviceInitReq);
    drivers.push_back(_lps3xhw);
    WS_DEBUG_PRINTLN("LPS3XHW Sensor Initialized Successfully!");
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
  } else if (strcmp("vl53l0x", msgDeviceInitReq->i2c_device_name) == 0) {
    _vl53l0x = new WipperSnapper_I2C_Driver_VL53L0X(this->_i2c, i2cAddress);
    if (!_vl53l0x->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize VL53L0X!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _vl53l0x->configureDriver(msgDeviceInitReq);
    drivers.push_back(_vl53l0x);
    WS_DEBUG_PRINTLN("VL53L0X Initialized Successfully!");
  } else if (strcmp("vl53l1x", msgDeviceInitReq->i2c_device_name) == 0) {
    _vl53l1x = new WipperSnapper_I2C_Driver_VL53L1X(this->_i2c, i2cAddress);
    if (!_vl53l1x->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize VL53L1X!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _vl53l1x->configureDriver(msgDeviceInitReq);
    drivers.push_back(_vl53l1x);
    WS_DEBUG_PRINTLN("VL53L1X Initialized Successfully!");
  } else if (strcmp("vl53l4cd", msgDeviceInitReq->i2c_device_name) == 0) {
    _vl53l4cd = new WipperSnapper_I2C_Driver_VL53L4CD(this->_i2c, i2cAddress);
    if (!_vl53l4cd->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize VL53L4CD!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _vl53l4cd->configureDriver(msgDeviceInitReq);
    drivers.push_back(_vl53l4cd);
    WS_DEBUG_PRINTLN("VL53L4CD Initialized Successfully!");
  } else if (strcmp("vl6180x", msgDeviceInitReq->i2c_device_name) == 0) {
    _vl6180x = new WipperSnapper_I2C_Driver_VL6180X(this->_i2c, i2cAddress);
    if (!_vl6180x->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize VL6180X!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _vl6180x->configureDriver(msgDeviceInitReq);
    drivers.push_back(_vl6180x);
    WS_DEBUG_PRINTLN("VL6180X Initialized Successfully!");
  } else if (strcmp("max17048", msgDeviceInitReq->i2c_device_name) == 0) {
    _max17048 = new WipperSnapper_I2C_Driver_MAX17048(this->_i2c, i2cAddress);
    if (!_max17048->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize MAX17048/MAX17049!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _max17048->configureDriver(msgDeviceInitReq);
    drivers.push_back(_max17048);
    WS_DEBUG_PRINTLN("MAX17048/MAX17049 Initialized Successfully!");
  } else if (strcmp("adt7410", msgDeviceInitReq->i2c_device_name) == 0) {
    _adt7410 = new WipperSnapper_I2C_Driver_ADT7410(this->_i2c, i2cAddress);
    if (!_adt7410->begin()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize ADT7410!");
      _busStatusResponse =
          wippersnapper_i2c_v1_BusResponse_BUS_RESPONSE_DEVICE_INIT_FAIL;
      return false;
    }
    _adt7410->configureDriver(msgDeviceInitReq);
    drivers.push_back(_adt7410);
    WS_DEBUG_PRINTLN("ADT7410 Initialized Successfully!");
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
  for (size_t i = 0; i < drivers.size(); i++) {
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
    @brief    Displays a sensor event message on the TFT
    @param    msgi2cResponse
              A pointer to an I2CResponse message.
    @param    sensorAddress
              The unique I2C address of the sensor.
*/
/*******************************************************************************/
void WipperSnapper_Component_I2C::displayDeviceEventMessage(
    wippersnapper_signal_v1_I2CResponse *msgi2cResponse,
    uint32_t sensorAddress) {

  pb_size_t numEvents =
      msgi2cResponse->payload.resp_i2c_device_event.sensor_event_count;

  char buffer[100];
  for (int i = 0; i < numEvents; i++) {
    float value =
        msgi2cResponse->payload.resp_i2c_device_event.sensor_event[i].value;

    switch (
        msgi2cResponse->payload.resp_i2c_device_event.sensor_event[i].type) {
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE:
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_OBJECT_TEMPERATURE:
      snprintf(buffer, 100, "[I2C: %#x] Read: %0.3f *C\n",
               (unsigned int)sensorAddress, value);
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE_FAHRENHEIT:
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_OBJECT_TEMPERATURE_FAHRENHEIT:
      snprintf(buffer, 100, "[I2C: %#x] Read: %0.3f *F\n",
               (unsigned int)sensorAddress, value);
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_RELATIVE_HUMIDITY:
      snprintf(buffer, 100, "[I2C: %#x] Read: %0.3f %% rh\n",
               (unsigned int)sensorAddress, value);
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PRESSURE:
      snprintf(buffer, 100, "[I2C: %#x] Read: %0.3f hPA\n",
               (unsigned int)sensorAddress, value);
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_ALTITUDE:
      snprintf(buffer, 100, "[I2C: %x] Read: %0.3f m\n",
               (unsigned int)sensorAddress, value);
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_LIGHT:
      snprintf(buffer, 100, "[I2C: %x] Read: %0.3f lux\n",
               (unsigned int)sensorAddress, value);
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM10_STD:
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM25_STD:
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM100_STD:
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_CO2:
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_ECO2:
      snprintf(buffer, 100, "[I2C: %x] Read: %0.3f ppm\n",
               (unsigned int)sensorAddress, value);
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_TVOC:
      snprintf(buffer, 100, "[I2C: %x] Read: %0.3f ppb\n",
               (unsigned int)sensorAddress, value);
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_UNITLESS_PERCENT:
      snprintf(buffer, 100, "[I2C: %x] Read: %0.3f%%\n",
               (unsigned int)sensorAddress, value);
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_VOLTAGE:
      snprintf(buffer, 100, "[I2C: %x] Read: %0.3f V\n",
               (unsigned int)sensorAddress, value);
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_CURRENT:
      snprintf(buffer, 100, "[I2C: %x] Read: %0.3f mA\n",
               (unsigned int)sensorAddress, value);
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_RAW:
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PROXIMITY:
      snprintf(buffer, 100, "[I2C: %x] Read: %0.3f\n",
               (unsigned int)sensorAddress, value);
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_GAS_RESISTANCE:
      snprintf(buffer, 100, "[I2C: %x] Read: %0.3f Ohms\n",
               (unsigned int)sensorAddress, value);
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_NOX_INDEX:
      snprintf(buffer, 100, "[I2C: %x] Read: %0.3f NOX\n",
               (unsigned int)sensorAddress, value);
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_VOC_INDEX:
      snprintf(buffer, 100, "[I2C: %x] Read: %0.3f VOC\n",
               (unsigned int)sensorAddress, value);
      break;
    default:
      break;
    }
#ifdef USE_DISPLAY
    WS._ui_helper->add_text_to_terminal(buffer);
#endif
  }
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
  bool sensorsReturningFalse = true;
  int retries = 3;

  while (sensorsReturningFalse && retries > 0) {
    sensorsReturningFalse = false;
    retries--;

    std::vector<WipperSnapper_I2C_Driver *>::iterator iter, end;
    for (iter = drivers.begin(), end = drivers.end(); iter != end; ++iter) {
      // Number of events which occured for this driver
      msgi2cResponse.payload.resp_i2c_device_event.sensor_event_count = 0;

      // Event struct
      sensors_event_t event;

        // AMBIENT_TEMPERATURE sensor (°C)
      sensorEventRead(iter, curTime, &msgi2cResponse,
                      &WipperSnapper_I2C_Driver::getEventAmbientTemp,
                      &WipperSnapper_I2C_Driver::getSensorAmbientTempPeriod,
                      &WipperSnapper_I2C_Driver::getSensorAmbientTempPeriodPrv,
                      &WipperSnapper_I2C_Driver::setSensorAmbientTempPeriodPrv,
                      wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE,
                      "Ambient Temperature", " degrees C", event,
                      &sensors_event_t::temperature, sensorsReturningFalse, retries);

      // Ambient Temperature sensor (°F)
      sensorEventRead(iter, curTime, &msgi2cResponse,
                      &WipperSnapper_I2C_Driver::getEventAmbientTempF,
                      &WipperSnapper_I2C_Driver::getSensorAmbientTempFPeriod,
                      &WipperSnapper_I2C_Driver::getSensorAmbientTempFPeriodPrv,
                      &WipperSnapper_I2C_Driver::setSensorAmbientTempFPeriodPrv,
                      wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE_FAHRENHEIT,
                      "Ambient Temperature", "°F", event,
                      &sensors_event_t::temperature, sensorsReturningFalse, retries);

      // OBJECT_TEMPERATURE sensor (°C)
      sensorEventRead(iter, curTime, &msgi2cResponse,
                      &WipperSnapper_I2C_Driver::getEventObjectTemp,
                      &WipperSnapper_I2C_Driver::getSensorObjectTempPeriod,
                      &WipperSnapper_I2C_Driver::getSensorObjectTempPeriodPrv,
                      &WipperSnapper_I2C_Driver::setSensorObjectTempPeriodPrv,
                      wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_OBJECT_TEMPERATURE,
                      "Object Temperature", " degrees C", event,
                      &sensors_event_t::temperature, sensorsReturningFalse, retries);

      // OBJECT_TEMPERATURE sensor (°F)
      sensorEventRead(iter, curTime, &msgi2cResponse,
                      &WipperSnapper_I2C_Driver::getEventObjectTempF,
                      &WipperSnapper_I2C_Driver::getSensorObjectTempFPeriod,
                      &WipperSnapper_I2C_Driver::getSensorObjectTempFPeriodPrv,
                      &WipperSnapper_I2C_Driver::setSensorObjectTempFPeriodPrv,
                      wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_OBJECT_TEMPERATURE_FAHRENHEIT,
                      "Object Temperature", "°F", event,
                      &sensors_event_t::temperature, sensorsReturningFalse, retries);

      // RELATIVE_HUMIDITY sensor
      sensorEventRead(iter, curTime, &msgi2cResponse,
                      &WipperSnapper_I2C_Driver::getEventRelativeHumidity,
                      &WipperSnapper_I2C_Driver::getSensorRelativeHumidityPeriod,
                      &WipperSnapper_I2C_Driver::getSensorRelativeHumidityPeriodPrv,
                      &WipperSnapper_I2C_Driver::setSensorRelativeHumidityPeriodPrv,
                      wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_RELATIVE_HUMIDITY,
                      "Humidity", " %RH", event, &sensors_event_t::relative_humidity, sensorsReturningFalse, retries);

      // PRESSURE sensor
      sensorEventRead(iter, curTime, &msgi2cResponse,
                      &WipperSnapper_I2C_Driver::getEventPressure,
                      &WipperSnapper_I2C_Driver::getSensorPressurePeriod,
                      &WipperSnapper_I2C_Driver::getSensorPressurePeriodPrv,
                      &WipperSnapper_I2C_Driver::setSensorPressurePeriodPrv,
                      wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PRESSURE,
                      "Pressure", " hPa", event, &sensors_event_t::pressure, sensorsReturningFalse, retries);

      // CO2 sensor
      sensorEventRead(iter, curTime, &msgi2cResponse,
                      &WipperSnapper_I2C_Driver::getEventCO2,
                      &WipperSnapper_I2C_Driver::getSensorCO2Period,
                      &WipperSnapper_I2C_Driver::getSensorCO2PeriodPrv,
                      &WipperSnapper_I2C_Driver::setSensorCO2PeriodPrv,
                      wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_CO2,
                      "CO2", " ppm", event, &sensors_event_t::CO2, sensorsReturningFalse, retries);

      // eCO2 sensor
      sensorEventRead(iter, curTime, &msgi2cResponse,
                      &WipperSnapper_I2C_Driver::getEventECO2,
                      &WipperSnapper_I2C_Driver::getSensorECO2Period,
                      &WipperSnapper_I2C_Driver::getSensorECO2PeriodPrv,
                      &WipperSnapper_I2C_Driver::setSensorECO2PeriodPrv,
                      wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_ECO2,
                      "eCO2", " ppm", event, &sensors_event_t::eCO2, sensorsReturningFalse, retries);

      // TVOC sensor
      sensorEventRead(iter, curTime, &msgi2cResponse,
                      &WipperSnapper_I2C_Driver::getEventTVOC,
                      &WipperSnapper_I2C_Driver::getSensorTVOCPeriod,
                      &WipperSnapper_I2C_Driver::getSensorTVOCPeriodPrv,
                      &WipperSnapper_I2C_Driver::setSensorTVOCPeriodPrv,
                      wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_TVOC,
                      "TVOC", " ppb", event, &sensors_event_t::tvoc, sensorsReturningFalse, retries);

      // Altitude sensor
      sensorEventRead(iter, curTime, &msgi2cResponse,
                      &WipperSnapper_I2C_Driver::getEventAltitude,
                      &WipperSnapper_I2C_Driver::getSensorAltitudePeriod,
                      &WipperSnapper_I2C_Driver::getSensorAltitudePeriodPrv,
                      &WipperSnapper_I2C_Driver::setSensorAltitudePeriodPrv,
                      wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_ALTITUDE,
                      "Altitude", " m", event, &sensors_event_t::altitude, sensorsReturningFalse, retries);

      // Light sensor
      sensorEventRead(iter, curTime, &msgi2cResponse,
                      &WipperSnapper_I2C_Driver::getEventLight,
                      &WipperSnapper_I2C_Driver::getSensorLightPeriod,
                      &WipperSnapper_I2C_Driver::getSensorLightPeriodPrv,
                      &WipperSnapper_I2C_Driver::setSensorLightPeriodPrv,
                      wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_LIGHT,
                      "Light", " lux", event, &sensors_event_t::light, sensorsReturningFalse, retries);

      // PM10_STD sensor
      sensorEventRead(iter, curTime, &msgi2cResponse,
                      &WipperSnapper_I2C_Driver::getEventPM10_STD,
                      &WipperSnapper_I2C_Driver::getSensorPM10_STDPeriod,
                      &WipperSnapper_I2C_Driver::getSensorPM10_STDPeriodPrv,
                      &WipperSnapper_I2C_Driver::setSensorPM10_STDPeriodPrv,
                      wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM10_STD,
                      "PM1.0", " ppm", event, &sensors_event_t::pm10_std, sensorsReturningFalse, retries);

      // PM25_STD sensor
      sensorEventRead(iter, curTime, &msgi2cResponse,
                      &WipperSnapper_I2C_Driver::getEventPM25_STD,
                      &WipperSnapper_I2C_Driver::getSensorPM25_STDPeriod,
                      &WipperSnapper_I2C_Driver::getSensorPM25_STDPeriodPrv,
                      &WipperSnapper_I2C_Driver::setSensorPM25_STDPeriodPrv,
                      wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM25_STD,
                      "PM2.5", " ppm", event, &sensors_event_t::pm25_std, sensorsReturningFalse, retries);

      // PM100_STD sensor
      sensorEventRead(iter, curTime, &msgi2cResponse,
                      &WipperSnapper_I2C_Driver::getEventPM100_STD,
                      &WipperSnapper_I2C_Driver::getSensorPM100_STDPeriod,
                      &WipperSnapper_I2C_Driver::getSensorPM100_STDPeriodPrv,
                      &WipperSnapper_I2C_Driver::setSensorPM100_STDPeriodPrv,
                      wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM100_STD,
                      "PM10.0", " ppm", event, &sensors_event_t::pm100_std, sensorsReturningFalse, retries);

      // Voltage sensor
      sensorEventRead(iter, curTime, &msgi2cResponse,
                      &WipperSnapper_I2C_Driver::getEventVoltage,
                      &WipperSnapper_I2C_Driver::getSensorVoltagePeriod,
                      &WipperSnapper_I2C_Driver::getSensorVoltagePeriodPrv,
                      &WipperSnapper_I2C_Driver::setSensorVoltagePeriodPrv,
                      wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_VOLTAGE,
                      "Voltage", " V", event, &sensors_event_t::voltage, sensorsReturningFalse, retries);

      // Current sensor
      sensorEventRead(iter, curTime, &msgi2cResponse,
                      &WipperSnapper_I2C_Driver::getEventCurrent,
                      &WipperSnapper_I2C_Driver::getSensorCurrentPeriod,
                      &WipperSnapper_I2C_Driver::getSensorCurrentPeriodPrv,
                      &WipperSnapper_I2C_Driver::setSensorCurrentPeriodPrv,
                      wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_CURRENT,
                      "Current", " mA", event, &sensors_event_t::current, sensorsReturningFalse, retries);

      // Unitless % sensor
      sensorEventRead(iter, curTime, &msgi2cResponse,
                      &WipperSnapper_I2C_Driver::getEventUnitlessPercent,
                      &WipperSnapper_I2C_Driver::getSensorUnitlessPercentPeriod,
                      &WipperSnapper_I2C_Driver::getSensorUnitlessPercentPeriodPrv,
                      &WipperSnapper_I2C_Driver::setSensorUnitlessPercentPeriodPrv,
                      wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_UNITLESS_PERCENT,
                      "Unitless Percent", " %", event, &sensors_event_t::unitless_percent, sensorsReturningFalse, retries);

      // Raw sensor
      sensorEventRead(iter, curTime, &msgi2cResponse,
                      &WipperSnapper_I2C_Driver::getEventRaw,
                      &WipperSnapper_I2C_Driver::getSensorRawPeriod,
                      &WipperSnapper_I2C_Driver::getSensorRawPeriodPrv,
                      &WipperSnapper_I2C_Driver::setSensorRawPeriodPrv,
                      wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_RAW,
                      "Raw", "", event, &sensors_event_t::data, sensorsReturningFalse, retries);

      // Gas sensor
      sensorEventRead(iter, curTime, &msgi2cResponse,
                      &WipperSnapper_I2C_Driver::getEventGasResistance,
                      &WipperSnapper_I2C_Driver::getSensorGasResistancePeriod,
                      &WipperSnapper_I2C_Driver::getSensorGasResistancePeriodPrv,
                      &WipperSnapper_I2C_Driver::setSensorGasResistancePeriodPrv,
                      wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_GAS_RESISTANCE,
                      "Gas Resistance", " Ohms", event, &sensors_event_t::gas_resistance, sensorsReturningFalse, retries);

      // NOx-index sensor
      sensorEventRead(iter, curTime, &msgi2cResponse,
                      &WipperSnapper_I2C_Driver::getEventNOxIndex,
                      &WipperSnapper_I2C_Driver::getSensorNOxIndexPeriod,
                      &WipperSnapper_I2C_Driver::getSensorNOxIndexPeriodPrv,
                      &WipperSnapper_I2C_Driver::setSensorNOxIndexPeriodPrv,
                      wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_NOX_INDEX,
                      "NOx Index", "", event, &sensors_event_t::nox_index, sensorsReturningFalse, retries);

      // VOC-index sensor
      sensorEventRead(iter, curTime, &msgi2cResponse,
                      &WipperSnapper_I2C_Driver::getEventVOCIndex,
                      &WipperSnapper_I2C_Driver::getSensorVOCIndexPeriod,
                      &WipperSnapper_I2C_Driver::getSensorVOCIndexPeriodPrv,
                      &WipperSnapper_I2C_Driver::setSensorVOCIndexPeriodPrv,
                      wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_VOC_INDEX,
                      "VOC Index", "", event, &sensors_event_t::voc_index, sensorsReturningFalse, retries);

      // Proximity sensor
      sensorEventRead(iter, curTime, &msgi2cResponse,
                      &WipperSnapper_I2C_Driver::getEventProximity,
                      &WipperSnapper_I2C_Driver::sensorProximityPeriod,
                      &WipperSnapper_I2C_Driver::SensorProximityPeriodPrv,
                      &WipperSnapper_I2C_Driver::setSensorProximityPeriodPrv,
                      wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PROXIMITY,
                      "Proximity", "", event, &sensors_event_t::distance, sensorsReturningFalse, retries);


      // Did this driver obtain data from sensors?
      if (msgi2cResponse.payload.resp_i2c_device_event.sensor_event_count ==
          0) {
        continue;
      }

      displayDeviceEventMessage(&msgi2cResponse, (*iter)->getI2CAddress());

      // Encode and publish I2CDeviceEvent message
      if (!encodePublishI2CDeviceEventMsg(&msgi2cResponse,
                                          (*iter)->getI2CAddress())) {
        WS_DEBUG_PRINTLN("ERROR: Failed to encode and publish I2CDeviceEvent!");
        continue;
      }
    } // end of retry loop
  }
}

void WipperSnapper_Component_I2C::sensorEventRead(
    std::vector<WipperSnapper_I2C_Driver *>::iterator &iter,
    unsigned long curTime,
    wippersnapper_signal_v1_I2CResponse *msgi2cResponse,
    bool (WipperSnapper_I2C_Driver::*getEventFunc)(sensors_event_t*),
    long (WipperSnapper_I2C_Driver::*getPeriodFunc)(),
    long (WipperSnapper_I2C_Driver::*getPeriodPrvFunc)(),
    void (WipperSnapper_I2C_Driver::*setPeriodPrvFunc)(long),
    wippersnapper_i2c_v1_SensorType sensorType,
    const char* sensorName, const char* unit, sensors_event_t event,
    auto sensors_event_t::*valueMember, bool &sensorsReturningFalse, int &retries) {
  // sensorName is a prefix and error message, units is a suffix when logging value
  curTime = millis();
  if (((*iter)->*getPeriodFunc)() != 0L &&
      curTime - ((*iter)->*getPeriodPrvFunc)() > ((*iter)->*getPeriodFunc)()) {
    if (((*iter)->*getEventFunc)(&event)) {
      WS_DEBUG_PRINT("Sensor 0x");
      WS_DEBUG_PRINTHEX((*iter)->getI2CAddress());
      WS_DEBUG_PRINTLN("");
      WS_DEBUG_PRINT("\t");
      WS_DEBUG_PRINT(sensorName);
      WS_DEBUG_PRINT(": ");
      WS_DEBUG_PRINT(event.*valueMember);
      WS_DEBUG_PRINTLN(unit);

      // pack event data into msg
      if (sensorType == wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_RAW) {
        fillEventMessage(msgi2cResponse, event.*valueMember[0], sensorType);
      } else {
        fillEventMessage(msgi2cResponse, event.*valueMember, sensorType);
      }

      ((*iter)->*setPeriodPrvFunc)(curTime);
    } else {
      WS_DEBUG_PRINT("ERROR: Failed to get ");
      WS_DEBUG_PRINT(sensorName);
      WS_DEBUG_PRINTLN(" reading!");
      sensorsReturningFalse = true;
      if (retries == 1) {
        ((*iter)->*setPeriodPrvFunc)(curTime);
      }
    }
  }
}
