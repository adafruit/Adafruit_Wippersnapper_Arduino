/*!
 * @file ws_ds18x20.cpp
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

#include "ws_ds18x20.h"

/*************************************************************/
/*!
    @brief    Creates a new WipperSnapper Ds18x20 component.
*/
/*************************************************************/
ws_ds18x20::ws_ds18x20() {}

/*************************************************************/
/*!
    @brief    Destructor for a WipperSnapper DS18X20 component.
*/
/*************************************************************/
ws_ds18x20::~ws_ds18x20() {}

bool ws_ds18x20::addDS18x20(
    wippersnapper_ds18x20_v1_Ds18x20InitRequest *msgDs18x20InitReq) {
  bool is_success = false;
  // unpack into usable variables
  char *oneWirePin = msgDs18x20InitReq->onewire_pin + 1;
  int pin = atoi(oneWirePin);

  // init. new ds18x20 object
  ds18x20Obj *newObj = new ds18x20Obj();
  newObj->oneWire = new OneWire(atoi(oneWirePin));
  newObj->dallasTempObj = new DallasTemperature(newObj->oneWire);
  newObj->dallasTempObj->begin();
  // attempt to obtain sensor address
  if (newObj->dallasTempObj->getAddress(newObj->dallasTempAddr, 0)) {
    // attempt to set sensor resolution
    newObj->dallasTempObj->setResolution(msgDs18x20InitReq->sensor_resolution);
    // add the new ds18x20 driver to vec.
    ds18xDrivers.push_back(newObj);
    is_success = true;
  } else {
    WS_DEBUG_PRINTLN("Failed to obtain DSx sensor address");
  }

  // fill and publish the initialization response back to the broker
  size_t msgSz; // message's encoded size

  wippersnapper_signal_v1_Ds18x20Response msgInitResp =
      wippersnapper_signal_v1_Ds18x20Response_init_zero;
  msgInitResp.which_payload =
      wippersnapper_signal_v1_Ds18x20Response_resp_ds18x20_init_tag;
  msgInitResp.payload.resp_ds18x20_init.is_initialized = is_success;
  strcpy(msgInitResp.payload.resp_ds18x20_init.onewire_pin,
         msgDs18x20InitReq->onewire_pin);

  // Encode and publish response back to broker
  memset(WS._buffer_outgoing, 0, sizeof(WS._buffer_outgoing));
  pb_ostream_t ostream =
      pb_ostream_from_buffer(WS._buffer_outgoing, sizeof(WS._buffer_outgoing));
  if (!pb_encode(&ostream, wippersnapper_signal_v1_Ds18x20Response_fields,
                 &msgInitResp)) {
    WS_DEBUG_PRINTLN("ERROR: Unable to encode msg_init response message!");
    return false;
  }
  pb_get_encoded_size(&msgSz, wippersnapper_signal_v1_Ds18x20Response_fields,
                      &msgInitResp);
  WS_DEBUG_PRINT("-> DS18x Init Response...");
  WS._mqtt->publish(WS._topic_signal_ds18_device, WS._buffer_outgoing, msgSz,
                    1);
  WS_DEBUG_PRINTLN("Published!");

  return is_success;
}

void ws_ds18x20::deleteDS18x20(
    wippersnapper_ds18x20_v1_Ds18x20DeInitRequest *msgDS18x20DeinitReq) {
  // TODO!
}

/*************************************************************/
/*!
    @brief    Obtains a temperature...
*/
/*************************************************************/
void ws_ds18x20::update() {
  /*
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
        pb_ostream_from_buffer(WS._buffer_outgoing,
    sizeof(WS._buffer_outgoing)); if (!pb_encode(&ostream,
    wippersnapper_signal_v1_Ds18x20Response_fields, &msgDS18Resp)) {
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
    _sensorPeriodPrv = millis(); // set period */
}