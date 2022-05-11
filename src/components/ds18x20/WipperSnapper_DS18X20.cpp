/*!
 * @file WipperSnapper_DS18X20.cpp
 *
 * This component implements 1-wire communication
 * for the DS18X20-line of Maxim Temperature ICs.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2022 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "WipperSnapper_DS18X20.h"

/*************************************************************/
/*!
    @brief    Creates a new WipperSnapper Ds18x20 component.
    @param    msgDs18x20InitReq
              The Ds18x20 initialization request message.
*/
/*************************************************************/
WipperSnapper_DS18X20::WipperSnapper_DS18X20(
    wippersnapper_ds18x20_v1_Ds18x20InitRequest *msgDs18x20InitReq) {
  // Set sensor pin
  _sensorPin = msgDs18x20InitReq->onewire_pin;

  // Initialize OneWire instance
  _wire = new OneWire();

  // Initialize DallasTemperature instance
  _ds = new DallasTemperature(_wire);

  // Set sensor resolution
  _resolution = msgDs18x20InitReq->sensor_resolution;
  // Set sensor period, in milliseconds
  _sensorPeriod = (long)msgDs18x20InitReq->sensor_period * 1000;
}

/*************************************************************/
/*!
    @brief    Destructor for a WipperSnapper DS18X20 component.
*/
/*************************************************************/
WipperSnapper_DS18X20::~WipperSnapper_DS18X20() {
  delete _ds;
  delete _wire;
}

/*************************************************************/
/*!
    @brief    Attempts to initialize a DS18X20 sensor.
    @returns  True if a sensor address was found and the
              sensor address is supported by the
              arduino-temperature-control library.
*/
/*************************************************************/
bool WipperSnapper_DS18X20::begin() {
  // Attempt to get address from DS sensor at index 0
  if (!_ds->getAddress(_sensorAddress, 0))
    return false;

  // Check if address is within the family of sensors the
  // Arduino-Temperature-Control-Library supports
  if (!_ds->validFamily(_sensorAddress))
    return false;

  // Attempt to set DS sensor's resolution
  _ds->setResolution(_sensorAddress, _resolution);

  return true;
}

/*************************************************************/
/*!
    @brief    Gets the pin used by the OneWire bus.
    @returns  The OneWire pin.
*/
/*************************************************************/
int32_t WipperSnapper_DS18X20::getPin() { return _sensorPin; }

/*************************************************************/
/*!
    @brief    Gets the address of the 0th sensor idx on the
              1-wire bus.
    @returns  The sensor's address.
*/
/*************************************************************/
uint8_t *WipperSnapper_DS18X20::getAddress() {
  _ds->getAddress(_sensorAddress, 0);
  return _sensorAddress;
}

/*************************************************************/
/*!
    @brief    Gets the resolution of the 0th sensor on the
              1-wire bus.
    @returns  The sensor's resolution
*/
/*************************************************************/
uint8_t WipperSnapper_DS18X20::getResolution() {
  return _ds->getResolution(_sensorAddress);
}

/*************************************************************/
/*!
    @brief    Obtains a temperature...
*/
/*************************************************************/
void WipperSnapper_DS18X20::update() {

  // Check if sensor period time has not yet elapsed
  long curTime = millis();
  if (!(curTime - _sensorPeriodPrv > _sensorPeriod))
    return;

  // Request temperature from 1-wire bus
  _ds->requestTemperatures();
  // Get temperature from sensor at idx 0 on 1-wire bus
  float tempC = _ds->getTempC(_sensorAddress);

  // Sensor is disconnected from 1-wire bus
  if (tempC == DEVICE_DISCONNECTED_C) {
    WS_DEBUG_PRINTLN("ERROR OBTAINING TEMPERATURE FROM DS18X20");
    return;
  }

  // Create a new sensor_event
  wippersnapper_signal_v1_Ds18x20Response msgDS18Resp =
      wippersnapper_signal_v1_Ds18x20Response_init_zero;
  msgDS18Resp.which_payload =
      wippersnapper_signal_v1_Ds18x20Response_resp_ds18x20_event_tag;
  msgDS18Resp.payload.resp_ds18x20_event.has_sensor_event = true;
  msgDS18Resp.payload.resp_ds18x20_event.onewire_pin = _sensorPin;
  msgDS18Resp.payload.resp_ds18x20_event.sensor_event.type =
      wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_OBJECT_TEMPERATURE;
  msgDS18Resp.payload.resp_ds18x20_event.sensor_event.value = tempC;

  // Encode sensor_event
  memset(WS._buffer_outgoing, 0, sizeof(WS._buffer_outgoing));
  pb_ostream_t ostream =
      pb_ostream_from_buffer(WS._buffer_outgoing, sizeof(WS._buffer_outgoing));
  if (!pb_encode(&ostream, wippersnapper_signal_v1_Ds18x20Response_fields,
                 &msgDS18Resp)) {
    WS_DEBUG_PRINTLN(
        "ERROR: Unable to encode DS18 device event response message!");
    return;
  }

  // Publish sensor_event
  size_t msgSz;
  pb_get_encoded_size(&msgSz, wippersnapper_signal_v1_Ds18x20Response_fields,
                      &msgDS18Resp);
  WS_DEBUG_PRINT("PUBLISHING -> DS Event...");
  if (!WS._mqtt->publish(WS._topic_signal_ds18_device, WS._buffer_outgoing,
                         msgSz, 1)) {
    WS_DEBUG_PRINTLN("ERROR: Unable to publish!");
    return;
  };
  WS_DEBUG_PRINTLN("PUBLISHED!");
  _sensorPeriodPrv = millis(); // set period
}