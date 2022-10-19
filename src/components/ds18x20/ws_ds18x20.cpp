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
ws_ds18x20::~ws_ds18x20() {
  // delete DallasTemp sensors and release onewire buses
  for (int idx = 0; idx < _ds18xDrivers.size(); idx++) {
    delete _ds18xDrivers[idx]->dallasTempObj;
    delete _ds18xDrivers[idx]->oneWire;
  }
  // remove all elements
  _ds18xDrivers.clear();
}

/********************************************************************/
/*!
    @brief    Initializes a DS18x20 sensor using a
              configuration sent by the broker and adds it to a
              vector of ds18x20 sensor drivers.
    @param    msgDs18x20InitReq
              Message containing configuration data for a
              ds18x20 sensor.
    @returns  True if initialized successfully, False otherwise.
*/
/********************************************************************/
bool ws_ds18x20::addDS18x20(
    wippersnapper_ds18x20_v1_Ds18x20InitRequest *msgDs18x20InitReq) {
  bool is_success = false;

  // init. new ds18x20 object
  ds18x20Obj *newObj = new ds18x20Obj();
  char *oneWirePin = msgDs18x20InitReq->onewire_pin + 1;
  newObj->oneWire = new OneWire(atoi(oneWirePin));
  newObj->dallasTempObj = new DallasTemperature(newObj->oneWire);
  newObj->dallasTempObj->begin();
  // attempt to obtain sensor address
  if (newObj->dallasTempObj->getAddress(newObj->dallasTempAddr, 0)) {
    // attempt to set sensor resolution
    newObj->dallasTempObj->setResolution(msgDs18x20InitReq->sensor_resolution);
    // copy the device's sensor properties
    newObj->sensorPropertiesCount =
        msgDs18x20InitReq->i2c_device_properties_count;
    // TODO: Make sure this works, it's a new idea and untested :)
    for (int i = 0; i < newObj->sensorPropertiesCount; i++) {
      newObj->sensorProperties[i].sensor_type =
          msgDs18x20InitReq->i2c_device_properties[i].sensor_type;
      newObj->sensorProperties[i].sensor_period =
          (long)msgDs18x20InitReq->i2c_device_properties[i].sensor_period *
          1000;
    }
    // set pin
    strcpy(newObj->onewire_pin, msgDs18x20InitReq->onewire_pin);
    // add the new ds18x20 driver to vec.
    _ds18xDrivers.push_back(newObj);
    is_success = true;
  } else {
    WS_DEBUG_PRINTLN("Failed to find DSx sensor on specified pin.");
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

  WS_DEBUG_PRINT("Created OneWireBus on GPIO ");
  WS_DEBUG_PRINT(msgDs18x20InitReq->onewire_pin);
  WS_DEBUG_PRINTLN(" with DS18x20 attached!");

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

/********************************************************************/
/*!
    @brief    De-initializes a DS18x20 sensor and releases its
              pin and resources.
    @param    msgDS18x20DeinitReq
              Message containing configuration data for a
              ds18x20 sensor.
*/
/********************************************************************/
void ws_ds18x20::deleteDS18x20(
    wippersnapper_ds18x20_v1_Ds18x20DeInitRequest *msgDS18x20DeinitReq) {
  // Loop thru vector of drivers to find the unique address
  for (int idx = 0; idx < _ds18xDrivers.size(); idx++) {
    if (strcmp(_ds18xDrivers[idx]->onewire_pin,
               msgDS18x20DeinitReq->onewire_pin) == 0) {
      WS_DEBUG_PRINT("Deleting OneWire instance on pin ");
      WS_DEBUG_PRINTLN(msgDS18x20DeinitReq->onewire_pin);
      delete _ds18xDrivers[idx]
          ->dallasTempObj; // delete dallas temp instance on pin
      delete _ds18xDrivers[idx]
          ->oneWire; // delete OneWire instance on pin and release pin for reuse
      _ds18xDrivers.erase(_ds18xDrivers.begin() +
                          idx); // erase vector and re-allocate
    }
  }
}

/*************************************************************/
/*!
    @brief    Iterates through each ds18x20 sensor and
              reports data (if period expired) to Adafruit IO.
*/
/*************************************************************/
void ws_ds18x20::update() {
  // return immediately if no drivers have been initialized
  if (_ds18xDrivers.size() == 0)
    return;

  long curTime; // used for holding the millis() value
  std::vector<ds18x20Obj *>::iterator iter, end;
  for (iter = _ds18xDrivers.begin(), end = _ds18xDrivers.end(); iter != end;
       ++iter) {

    // Create an empty DS18x20 event signal message and configure
    wippersnapper_signal_v1_Ds18x20Response msgDS18x20Response =
        wippersnapper_signal_v1_Ds18x20Response_init_zero;
    msgDS18x20Response.which_payload =
        wippersnapper_signal_v1_Ds18x20Response_resp_ds18x20_event_tag;

    // take the current time for the driver (*iter)
    curTime = millis();
    // Poll each sensor type, if period has elapsed
    for (int i = 0; i < (*iter)->sensorPropertiesCount; i++) {
      // has sensor_period elapsed?
      if (curTime - (*iter)->sensorPeriodPrv >
          (*iter)->sensorProperties[i].sensor_period) {
        // issue global temperature request to all DS sensors
        WS_DEBUG_PRINTLN("Requesting temperature..");
        (*iter)->dallasTempObj->requestTemperatures();
        // poll the DS sensor driver
        float tempC = (*iter)->dallasTempObj->getTempC((*iter)->dallasTempAddr);
        if (tempC == DEVICE_DISCONNECTED_C) {
          WS_DEBUG_PRINTLN("ERROR: Could not read temperature data, is the "
                           "sensor disconnected?");
          break;
        }

        // check and pack based on sensorType
        if ((*iter)->sensorProperties[i].sensor_type ==
            wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE) {

          WS_DEBUG_PRINT("(OneWireBus GPIO: ");
          WS_DEBUG_PRINT((*iter)->onewire_pin);
          WS_DEBUG_PRINT(") DS18x20 Value: ");
          WS_DEBUG_PRINT(tempC);
          WS_DEBUG_PRINTLN("*C")

          msgDS18x20Response.payload.resp_ds18x20_event.sensor_event[i].type =
              wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE;
          msgDS18x20Response.payload.resp_ds18x20_event.sensor_event[i].value =
              tempC;

          msgDS18x20Response.payload.resp_ds18x20_event.sensor_event_count++;
        }

        if ((*iter)->sensorProperties[i].sensor_type ==
            wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE_FAHRENHEIT) {
          msgDS18x20Response.payload.resp_ds18x20_event.sensor_event[i].type =
              wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE_FAHRENHEIT;
          msgDS18x20Response.payload.resp_ds18x20_event.sensor_event[i].value =
              (*iter)->dallasTempObj->toFahrenheit(tempC);
          WS_DEBUG_PRINT("(OneWireBus GPIO: ");
          WS_DEBUG_PRINT((*iter)->onewire_pin);
          WS_DEBUG_PRINT(") DS18x20 Value: ");
          WS_DEBUG_PRINT(
              msgDS18x20Response.payload.resp_ds18x20_event.sensor_event[i]
                  .value);
          WS_DEBUG_PRINTLN("*F")

          msgDS18x20Response.payload.resp_ds18x20_event.sensor_event_count++;
        }

        // did we obtain the expected amount of sensor events for the
        // `resp_ds18x20_event` message?
        if (msgDS18x20Response.payload.resp_ds18x20_event.sensor_event_count ==
            (*iter)->sensorPropertiesCount) {

          // prep sensor event data for sending to IO
          // use onewire_pin as the "address"
          strcpy(msgDS18x20Response.payload.resp_ds18x20_event.onewire_pin,
                 (*iter)->onewire_pin);
          // prep and encode buffer
          memset(WS._buffer_outgoing, 0, sizeof(WS._buffer_outgoing));
          pb_ostream_t ostream = pb_ostream_from_buffer(
              WS._buffer_outgoing, sizeof(WS._buffer_outgoing));
          if (!pb_encode(&ostream,
                         wippersnapper_signal_v1_Ds18x20Response_fields,
                         &msgDS18x20Response)) {
            WS_DEBUG_PRINTLN(
                "ERROR: Unable to encode DS18x20 event responsemessage!");
            return;
          }

          WS_DEBUG_PRINTLN(
              "DEBUG: msgDS18x20Response sensor_event message contents:");
          for (int i = 0;
               i <
               msgDS18x20Response.payload.resp_ds18x20_event.sensor_event_count;
               i++) {
            WS_DEBUG_PRINT("sensor_event[#]: ");
            WS_DEBUG_PRINTLN(i);
            WS_DEBUG_PRINT("\tOneWire Bus: ");
            WS_DEBUG_PRINTLN(
                msgDS18x20Response.payload.resp_ds18x20_event.onewire_pin);
            WS_DEBUG_PRINT("\tsensor_event type: ");
            WS_DEBUG_PRINTLN(
                msgDS18x20Response.payload.resp_ds18x20_event.sensor_event[i]
                    .type);
            WS_DEBUG_PRINT("\tsensor_event value: ");
            WS_DEBUG_PRINTLN(
                msgDS18x20Response.payload.resp_ds18x20_event.sensor_event[i]
                    .value);
          }

          // Publish I2CResponse msg
          size_t msgSz;
          pb_get_encoded_size(&msgSz,
                              wippersnapper_signal_v1_Ds18x20Response_fields,
                              &msgDS18x20Response);
          WS_DEBUG_PRINT("PUBLISHING -> msgDS18x20Response Event Message...");
          if (!WS._mqtt->publish(WS._topic_signal_ds18_device,
                                 WS._buffer_outgoing, msgSz, 1)) {
            return;
          };
          WS_DEBUG_PRINTLN("PUBLISHED!");

          (*iter)->sensorPeriodPrv = curTime; // set prv period
        }
      }
    }
  }
}