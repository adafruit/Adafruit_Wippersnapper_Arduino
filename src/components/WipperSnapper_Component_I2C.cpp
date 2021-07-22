/*!
 * @file Wippersnapper_Component_I2C.cpp
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

#include "WipperSnapper_Component_I2C.h"

WipperSnapper_Component_I2C::WipperSnapper_Component_I2C(wippersnapper_i2c_v1_I2CInitRequest *msgInitRequest) {
  WS_DEBUG_PRINTLN("NEW WipperSnapper_Component_I2C");
  // initialize using portNum
  _i2c = new TwoWire(msgInitRequest->i2c_port_number);
  // validate if SDA & SCL has pullup
  if (digitalRead(msgInitRequest->i2c_pin_sda) == LOW) {
    pinMode(msgInitRequest->i2c_pin_sda, INPUT_PULLUP);
  }
  if (digitalRead(msgInitRequest->i2c_pin_scl) == LOW) {
    pinMode(msgInitRequest->i2c_pin_scl, INPUT_PULLUP);
  }
  // set up i2c
  _i2c->begin(msgInitRequest->i2c_pin_sda, msgInitRequest->i2c_pin_scl);
  _i2c->setClock(msgInitRequest->i2c_frequency);
  WS_DEBUG_PRINTLN("Init. new I2C Bus: ");
  WS_DEBUG_PRINT("Port#: ");WS_DEBUG_PRINTLN(msgInitRequest->i2c_port_number);
  WS_DEBUG_PRINT("SDA: ");WS_DEBUG_PRINTLN(msgInitRequest->i2c_pin_sda);
  WS_DEBUG_PRINT("SCL: ");WS_DEBUG_PRINTLN(msgInitRequest->i2c_pin_scl);
  WS_DEBUG_PRINT("Frequency (Hz): ");WS_DEBUG_PRINTLN(msgInitRequest->i2c_frequency);
  WS_DEBUG_PRINTLN("Init. new I2C Bus: ");
  // set i2c obj. properties
  _portNum = msgInitRequest->i2c_port_number;
  _isInit = true;
  yield();
}

WipperSnapper_Component_I2C::~WipperSnapper_Component_I2C() {
  _portNum = 100; // Invalid = 100
  _isInit = false;
}

bool WipperSnapper_Component_I2C::scanAddresses(wippersnapper_i2c_v1_I2CScanRequest msgScanReq) {
  // decode stream into i2c request
  bool is_detected = false;
  uint16_t scanAddr;
  WS_DEBUG_PRINT("EXEC: I2C Scan, Port ("); WS_DEBUG_PRINT(_portNum); WS_DEBUG_PRINTLN(")");
  for (int i = 0; i < msgScanReq.address_count; i++) {
    scanAddr = msgScanReq.address[i];
    WS_DEBUG_PRINT("Scanning address "); WS_DEBUG_PRINTLN(scanAddr);
    _i2c->beginTransmission(scanAddr);
    if (_i2c->endTransmission() == 0) {
        // found it!
        WS_DEBUG_PRINTLN("I2C device detected!");
        is_detected = true;
        break;
    } else {
        WS_DEBUG_PRINTLN("I2C device not detected!");
    }
  }
  // TODO
  // encode response and publish to broker
  return is_detected;
}
