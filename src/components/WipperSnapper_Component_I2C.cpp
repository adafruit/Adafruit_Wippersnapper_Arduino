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

bool WipperSnapper_Component_I2C::scanForAddress(uint32_t address) {
  bool is_detected = false;
  WS_DEBUG_PRINT("EXEC: I2C Scan, Port ("); WS_DEBUG_PRINT(_portNum); WS_DEBUG_PRINTLN(")");
  // do i2c scan
  // A basic scanner, see if it ACK's
  // TODO: Remove address, use request protobuf struct. typedef
  _wire->beginTransmission(address);
  if (_wire->endTransmission() == 0) {
    WS_DEBUG_PRINTLN("I2C device detected!");
    is_detected = true;
  } else {
    WS_DEBUG_PRINTLN("I2C device not detected!");
  }
  // TODO
  // encode response and publish to broker
  return is_success;
}
