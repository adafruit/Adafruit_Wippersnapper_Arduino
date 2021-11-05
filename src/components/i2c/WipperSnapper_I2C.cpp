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
  if (msgDeviceInitReq->has_aht_init) {
    // Initialize new AHTX0 sensor
    _ahtx0 = new WipperSnapper_I2C_Driver_AHTX0(this->_i2c, i2cAddress);

    // Did we initialize successfully?
    if (!_ahtx0->getInitialized()) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize AHTX0 chip!");
      return false;
    }
    WS_DEBUG_PRINTLN("AHTX0 Initialized Successfully!");

    // Configure AHTX0 sensor
    if (msgDeviceInitReq->aht_init.enable_temperature) {
      _ahtx0->enableTemperatureSensor();
      _ahtx0->setTemperatureSensorPeriod(
          msgDeviceInitReq->aht_init.period_temperature);
      WS_DEBUG_PRINTLN("Enabled AHTX0 Temperature Sensor, [Returns every: ");
      WS_DEBUG_PRINT(msgDeviceInitReq->aht_init.period_temperature);
      WS_DEBUG_PRINTLN("seconds]");
    }
    if (msgDeviceInitReq->aht_init.enable_humidity) {
      _ahtx0->enableHumiditySensor();
      _ahtx0->setHumiditySensorPeriod(
          msgDeviceInitReq->aht_init.period_humidity);
      WS_DEBUG_PRINTLN("Enabled AHTX0 Humidity Sensor, [Returns every: ");
      WS_DEBUG_PRINT(msgDeviceInitReq->aht_init.period_humidity);
      WS_DEBUG_PRINTLN("seconds]");
    }
    drivers.push_back(_ahtx0);
  } else {
    WS_DEBUG_PRINTLN("ERROR: Sensor not found")
  }
  WS_DEBUG_PRINTLN("Successfully initialized AHTX0 sensor!");
  return true;
}

/*******************************************************************************/
/*!
    @brief    Updates the properties of an I2C device driver.
    @param    msgDeviceUpdateReq
              A decoded I2CDeviceUpdateRequest.
    @returns True if I2C device is was successfully updated, False otherwise.
*/
/*******************************************************************************/
bool WipperSnapper_Component_I2C::updateI2CDevice(
    wippersnapper_i2c_v1_I2CDeviceDeinitRequest *msgDeviceUpdateReq) {
  // TODO!! //
}

/*******************************************************************************/
/*!
    @brief    Deinitializes an I2C device driver.
    @param    msgDeviceDeinitReq
              A decoded I2CDeviceDeinitRequest.
    @returns True if I2C device is found and de-initialized, False otherwise.
*/
/*******************************************************************************/
bool WipperSnapper_Component_I2C::deinitI2CDevice(
    wippersnapper_i2c_v1_I2CDeviceDeinitRequest *msgDeviceDeinitReq) {
  uint16_t deviceAddr = (uint16_t)msgDeviceDeinitReq->i2c_address;
  // Loop thru vector of drivers
  for (int i = 0; i < drivers.size(); i++) {
    if (drivers[i]->getSensorAddress() == deviceAddr) {
      // Check driver type
      if (drivers[i]->getDriverType() == AHTX0) {
        // Should we delete the driver entirely, or just update?
        if ((msgDeviceDeinitReq->aht.disable_temperature &&
             drivers[i]->getHumidSensorPeriod() == -1L) ||
            (msgDeviceDeinitReq->aht.disable_humidity &&
             drivers[i]->getTempSensorPeriod() == -1L) ||
            (msgDeviceDeinitReq->aht.disable_temperature &&
             msgDeviceDeinitReq->aht.disable_humidity)) {
          // delete the driver and remove from list so we dont attempt to
          // update() it
          delete _ahtx0;
          drivers.erase(drivers.begin() + i);
          return true;
          WS_DEBUG_PRINTLN("AHTX0 Deleted");
        }

        // Disable the device's temperature sensor
        else if (msgDeviceDeinitReq->aht.disable_temperature) {
          drivers[i]->disableTemperatureSensor();
          return true;
          WS_DEBUG_PRINTLN("AHTX0 Temperature Sensor Disabled");
        }

        // Disable the device's humidity sensor
        else if (msgDeviceDeinitReq->aht.disable_humidity) {
          drivers[i]->disableHumiditySensor();
          WS_DEBUG_PRINTLN("AHTX0 Humidity Sensor Disabled");
          return true;
        }
      } else {
        WS_DEBUG_PRINTLN("ERROR: Driver type unspecified");
      }
    }
  }
  // Driver was not erased or not found
  return false;
}

/*******************************************************************************/
/*!
    @brief    Queries all I2C device drivers for new values. Fills and sends an
              I2CSensorEvent with the sensor event data.
*/
/*******************************************************************************/
void WipperSnapper_Component_I2C::update() {
  // TODO
}