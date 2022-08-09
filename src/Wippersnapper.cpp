/*!
 * @file Wippersnapper.cpp
 *
 * @mainpage Adafruit Wippersnapper Wrapper
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's Wippersnapper wrapper for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit IO+ Wippersnapper IoT platform.
 *
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 * This library depends on <a
 * href="https://github.com/adafruit/Adafruit_Sensor"> Adafruit_Sensor</a> being
 * present on your system. Please make sure you have installed the latest
 * version before using this library.
 *
 * @section author Author
 *
 * Copyright (c) Brent Rubell 2020-2021 for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "Wippersnapper.h"

Wippersnapper WS;

Wippersnapper::Wippersnapper() {
  _mqtt = 0; // MQTT Client object

  // IO creds
  _username = 0;
  _key = 0;

  // Reserved MQTT Topics
  _topic_description = 0;
  _topic_description_status = 0;
  _topic_signal_device = 0;
  _topic_signal_brkr = 0;
  _err_topic = 0;
  _throttle_topic = 0;
  _err_sub = 0;
  _throttle_sub = 0;
};

/**************************************************************************/
/*!
    @brief    Wippersnapper destructor
*/
/**************************************************************************/
Wippersnapper::~Wippersnapper() {
  // free topics
  free(_topic_description);
  free(_topic_signal_device);
  free(_topic_signal_brkr);
  free(_err_sub);
  free(_throttle_sub);
}

/**************************************************************************/
/*!
    @brief    Provisions a WipperSnapper device with its network
              configuration and Adafruit IO credentials.
*/
/**************************************************************************/
void Wippersnapper::provision() {
  // Get MAC address from network interface
  getMacAddr();

  // init. LED for status signaling
  statusLEDInit();
#ifdef USE_TINYUSB
  _fileSystem = new Wippersnapper_FS();
  _fileSystem->parseSecrets();
#elif defined(USE_LITTLEFS)
  _littleFS = new WipperSnapper_LittleFS();
  _littleFS->parseSecrets();
#else
  set_user_key(); // non-fs-backed, sets global credentials within network iface
#endif

  set_ssid_pass();
}

/**************************************************************************/
/*!
    @brief    Disconnects from Adafruit IO+ Wippersnapper.
*/
/**************************************************************************/
void Wippersnapper::disconnect() { _disconnect(); }

// Concrete class definition for abstract classes

/****************************************************************************/
/*!
    @brief    Connects to wireless network.
*/
/****************************************************************************/
void Wippersnapper::_connect() {
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
}

/****************************************************************************/
/*!
    @brief    Disconnect Wippersnapper MQTT session and network.
*/
/****************************************************************************/
void Wippersnapper::_disconnect() {
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
}

/****************************************************************************/
/*!
    @brief    Sets the network interface's unique identifer, typically the
              MAC address.
*/
/****************************************************************************/
void Wippersnapper::getMacAddr() {
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
}

/****************************************************************************/
/*!
    @brief    Sets up the MQTT client session.
    @param    clientID
              A unique client identifier string.
*/
/****************************************************************************/
void Wippersnapper::setupMQTTClient(const char * /*clientID*/) {
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
}

/****************************************************************************/
/*!
    @brief    Returns the network's connection status
    @returns  Network status as ws_status_t.
*/
/****************************************************************************/
ws_status_t Wippersnapper::networkStatus() {
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
  return WS_IDLE;
}

/****************************************************************************/
/*!
    @brief    Sets the device's wireless network credentials.
    @param    ssid
              Your wireless network's SSID
    @param    ssidPassword
              Your wireless network's password.
*/
/****************************************************************************/
void Wippersnapper::set_ssid_pass(const char * /*ssid*/,
                                  const char * /*ssidPassword*/) {
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
}

/****************************************************************************/
/*!
    @brief    Sets the device's wireless network credentials from the
              secrets.json configuration file.
*/
/****************************************************************************/
void Wippersnapper::set_ssid_pass() {
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
}

/****************************************************************************/
/*!
    @brief    Configures the device's Adafruit IO credentials. This method
              should be used only if filesystem-backed provisioning is
              not avaliable.
*/
/****************************************************************************/
void Wippersnapper::set_user_key() {
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
}

// Decoders //
/****************************************************************************/
/*!
    @brief    Configures a pin according to a
                wippersnapper_pin_v1_ConfigurePinRequest message.
    @param    pinMsg
              Pointer to a wippersnapper_pin_v1_ConfigurePinRequest message.
    @returns  True if pin configured successfully, False otherwise.
*/
/****************************************************************************/
bool Wippersnapper::configurePinRequest(
    wippersnapper_pin_v1_ConfigurePinRequest *pinMsg) {
  WS_DEBUG_PRINTLN("configurePinRequest");

  bool is_success = true;
  char *pinName = pinMsg->pin_name + 1;
  int pin = atoi(pinName);

  // Decode pin mode
  if (pinMsg->mode == wippersnapper_pin_v1_Mode_MODE_DIGITAL) {
    if (pinMsg->request_type ==
        wippersnapper_pin_v1_ConfigurePinRequest_RequestType_REQUEST_TYPE_CREATE) {
      // Initialize GPIO pin
      WS._digitalGPIO->initDigitalPin(pinMsg->direction, pin, pinMsg->period,
                                      pinMsg->pull);
    } else if (
        pinMsg->request_type ==
        wippersnapper_pin_v1_ConfigurePinRequest_RequestType_REQUEST_TYPE_DELETE) {
      // Delete digital GPIO pin
      WS._digitalGPIO->deinitDigitalPin(pinMsg->direction, pin);
    } else {
      WS_DEBUG_PRINTLN("ERROR: Could not decode digital pin request type");
    }
  }

  else if (pinMsg->mode == wippersnapper_pin_v1_Mode_MODE_ANALOG) {
    if (pinMsg->request_type ==
        wippersnapper_pin_v1_ConfigurePinRequest_RequestType_REQUEST_TYPE_CREATE) {
      // Initialize analog io pin
      if (pinMsg->direction ==
          wippersnapper_pin_v1_ConfigurePinRequest_Direction_DIRECTION_INPUT) {
        WS._analogIO->initAnalogInputPin(pin, pinMsg->period, pinMsg->pull,
                                         pinMsg->analog_read_mode);
      } else if (
          pinMsg->direction ==
          wippersnapper_pin_v1_ConfigurePinRequest_Direction_DIRECTION_OUTPUT) {
        WS._analogIO->initAnalogOutputPin(pin);
      } else {
        WS_DEBUG_PRINTLN("ERROR: Unable to decode analog pin direction.")
        is_success = false;
      }
    } else if (
        pinMsg->request_type ==
        wippersnapper_pin_v1_ConfigurePinRequest_RequestType_REQUEST_TYPE_DELETE) {
      // Delete analog io pin
      WS._analogIO->deinitAnalogPin(pinMsg->direction, pin);
    } else {
      WS_DEBUG_PRINTLN("ERROR: Could not decode digital pin request type");
    }
  } else {
    WS_DEBUG_PRINTLN("ERROR: Could not decode pin mode")
    is_success = false;
  }

  return is_success;
}

/*****************************************************************************/
/*!
    @brief  Decodes a repeated ConfigurePinRequests messages.
    @param  stream
            Input stream to read from.
    @param  field
            Message descriptor, usually autogenerated.
    @param  arg
            Stores any information the decoding callback may need.
    @returns  True if pin configuration decoded successfully, False otherwise.
*/
/*****************************************************************************/
bool cbDecodePinConfigMsg(pb_istream_t *stream, const pb_field_t *field,
                          void **arg) {
  bool is_success = true;
  WS_DEBUG_PRINTLN("cbDecodePinConfigMsg");

  // pb_decode the stream into a pinReqMessage
  wippersnapper_pin_v1_ConfigurePinRequest pinReqMsg =
      wippersnapper_pin_v1_ConfigurePinRequest_init_zero;
  if (!pb_decode(stream, wippersnapper_pin_v1_ConfigurePinRequest_fields,
                 &pinReqMsg)) {
    WS_DEBUG_PRINTLN("ERROR: Could not decode CreateSignalRequest")
    is_success = false;
  }

  // Pass ConfigurePinRequest message
  if (!WS.configurePinRequest(&pinReqMsg)) {
    WS_DEBUG_PRINTLN("Unable to configure pin");
    is_success = false;
  }

  return is_success;
}

/**************************************************************************/
/*!
    @brief  Decodes repeated PinEvents messages.
    @param  stream
            Input stream to read from.
    @param  field
            Message descriptor, usually autogenerated.
    @param  arg
            Stores any information the decoding callback may need.
    @returns True if successfully decoded, False otherwise.
*/
/**************************************************************************/
bool cbDecodePinEventMsg(pb_istream_t *stream, const pb_field_t *field,
                         void **arg) {
  bool is_success = true;
  WS_DEBUG_PRINTLN("cbDecodePinEventMsg");

  // Decode stream into a PinEvent
  wippersnapper_pin_v1_PinEvent pinEventMsg =
      wippersnapper_pin_v1_PinEvent_init_zero;
  if (!pb_decode(stream, wippersnapper_pin_v1_PinEvent_fields, &pinEventMsg)) {
    WS_DEBUG_PRINTLN("ERROR: Could not decode PinEvents")
    is_success = false;
  }

  char *pinName = pinEventMsg.pin_name + 1;
  if (pinEventMsg.pin_name[0] == 'D') { // digital pin event
    WS._digitalGPIO->digitalWriteSvc(atoi(pinName),
                                     atoi(pinEventMsg.pin_value));
  } else if (pinEventMsg.pin_name[0] == 'A') { // analog pin event
    // TODO
    WS_DEBUG_PRINTLN("ERROR: Analog PinEvent unimplemented!");
  } else {
    WS_DEBUG_PRINTLN("ERROR: Unable to decode pin event name.");
    is_success = false;
  }

  return is_success;
}

/**************************************************************************/
/*!
    @brief      Sets payload callbacks inside the signal message's
                submessage.
    @param  stream
            Input stream to read from.
    @param  field
            Message descriptor, usually autogenerated.
    @param  arg
            Stores any information the decoding callback may need.
    @returns True if successfully decoded, false otherwise.
*/
/**************************************************************************/
bool cbSignalMsg(pb_istream_t *stream, const pb_field_t *field, void **arg) {
  bool is_success = true;
  WS_DEBUG_PRINTLN("cbSignalMsg");

  pb_size_t arr_sz = field->array_size;
  WS_DEBUG_PRINT("Sub-messages found: ");
  WS_DEBUG_PRINTLN(arr_sz);

  if (field->tag ==
      wippersnapper_signal_v1_CreateSignalRequest_pin_configs_tag) {
    WS_DEBUG_PRINTLN("Signal Msg Tag: Pin Configuration");
    // array to store the decoded CreateSignalRequests data
    wippersnapper_pin_v1_ConfigurePinRequests msg =
        wippersnapper_pin_v1_ConfigurePinRequests_init_zero;
    // set up callback
    msg.list.funcs.decode = cbDecodePinConfigMsg;
    msg.list.arg = field->pData;
    // decode each ConfigurePinRequest sub-message
    if (!pb_decode(stream, wippersnapper_pin_v1_ConfigurePinRequests_fields,
                   &msg)) {
      WS_DEBUG_PRINTLN("ERROR: Could not decode CreateSignalRequest")
      is_success = false;
      WS.pinCfgCompleted = false;
    }
    // If this is the initial configuration
    if (!WS.pinCfgCompleted) {
      WS_DEBUG_PRINTLN("Initial Pin Configuration Complete!");
      WS.pinCfgCompleted = true;
    }
  } else if (field->tag ==
             wippersnapper_signal_v1_CreateSignalRequest_pin_events_tag) {
    WS_DEBUG_PRINTLN("Signal Msg Tag: Pin Event");
    // array to store the decoded PinEvents data
    wippersnapper_pin_v1_PinEvents msg =
        wippersnapper_pin_v1_PinEvents_init_zero;
    // set up callback
    msg.list.funcs.decode = cbDecodePinEventMsg;
    msg.list.arg = field->pData;
    // decode each PinEvents sub-message
    if (!pb_decode(stream, wippersnapper_pin_v1_PinEvents_fields, &msg)) {
      WS_DEBUG_PRINTLN("ERROR: Could not decode CreateSign2alRequest")
      is_success = false;
    }
  } else {
    WS_DEBUG_PRINTLN("ERROR: Unexpected signal msg tag.");
  }

  // once this is returned, pb_dec_submessage()
  // decodes the submessage contents.
  return is_success;
}

/**************************************************************************/
/*!
    @brief    Decodes a signal buffer protobuf message.
        NOTE: Should be executed in-order after a new _buffer is recieved.
    @param    encodedSignalMsg
              Encoded signal message.
    @return   true if successfully decoded signal message, false otherwise.
*/
/**************************************************************************/
bool Wippersnapper::decodeSignalMsg(
    wippersnapper_signal_v1_CreateSignalRequest *encodedSignalMsg) {
  bool is_success = true;
  WS_DEBUG_PRINTLN("decodeSignalMsg");

  /* Set up the payload callback, which will set up the callbacks for
  each oneof payload field once the field tag is known */
  encodedSignalMsg->cb_payload.funcs.decode = cbSignalMsg;

  // decode the CreateSignalRequest, calls cbSignalMessage and assoc. callbacks
  pb_istream_t stream = pb_istream_from_buffer(WS._buffer, WS.bufSize);
  if (!pb_decode(&stream, wippersnapper_signal_v1_CreateSignalRequest_fields,
                 encodedSignalMsg)) {
    WS_DEBUG_PRINTLN(
        "ERROR (decodeSignalMsg):, Could not decode CreateSignalRequest")
    is_success = false;
  }
  return is_success;
}

/**************************************************************************/
/*!
    @brief    Called when signal topic receives a new message. Fills
                shared buffer with data from payload.
    @param    data
                Data from MQTT broker.
    @param    len
                Length of data received from MQTT broker.
*/
/**************************************************************************/
void cbSignalTopic(char *data, uint16_t len) {
  WS_DEBUG_PRINTLN("cbSignalTopic: New Msg on Signal Topic");
  WS_DEBUG_PRINT(len);
  WS_DEBUG_PRINTLN(" bytes.");
  // zero-out current buffer
  memset(WS._buffer, 0, sizeof(WS._buffer));
  // copy data to buffer
  memcpy(WS._buffer, data, len);
  WS.bufSize = len;

  // Empty struct for storing the signal message
  WS._incomingSignalMsg = wippersnapper_signal_v1_CreateSignalRequest_init_zero;

  // Attempt to decode a signal message
  if (!WS.decodeSignalMsg(&WS._incomingSignalMsg)) {
    WS_DEBUG_PRINTLN("ERROR: Failed to decode signal message");
  }
}

/******************************************************************************************/
/*!
    @brief    Publishes an I2C response signal message to the broker.
    @param    msgi2cResponse
              A pointer to an I2C response message typedef.
*/
/******************************************************************************************/
void publishI2CResponse(wippersnapper_signal_v1_I2CResponse *msgi2cResponse) {
  size_t msgSz;
  pb_get_encoded_size(&msgSz, wippersnapper_signal_v1_I2CResponse_fields,
                      msgi2cResponse);
  WS_DEBUG_PRINT("Publishing Message: I2CResponse...");
  WS._mqtt->publish(WS._topic_signal_i2c_device, WS._buffer_outgoing, msgSz, 1);
  WS_DEBUG_PRINTLN("Published!");
}

/******************************************************************************************/
/*!
    @brief    Encodes an wippersnapper_signal_v1_I2CResponse message.
    @param    msgi2cResponse
              A pointer to an wippersnapper_signal_v1_I2CResponse.
    @return   True if encoded successfully, False otherwise.
*/
/******************************************************************************************/
bool encodeI2CResponse(wippersnapper_signal_v1_I2CResponse *msgi2cResponse) {
  memset(WS._buffer_outgoing, 0, sizeof(WS._buffer_outgoing));
  pb_ostream_t ostream =
      pb_ostream_from_buffer(WS._buffer_outgoing, sizeof(WS._buffer_outgoing));
  if (!pb_encode(&ostream, wippersnapper_signal_v1_I2CResponse_fields,
                 msgi2cResponse)) {
    WS_DEBUG_PRINTLN("ERROR: Unable to encode I2C response message!");
    return false;
  }
  return true;
}

/******************************************************************************************/
/*!
    @brief    Initializes an I2C bus component
    @param    msgInitRequest
              A pointer to an i2c bus initialization message.
    @param    i2cPort
              Desired I2C port to initialize.
    @return   True if initialized successfully, False otherwise.
*/
/******************************************************************************************/
bool initializeI2CBus(wippersnapper_i2c_v1_I2CBusInitRequest msgInitRequest,
                      int i2cPort) {
  // TODO: i2cPort is not handled right now, we should add support for multiple
  // i2c ports!
  if (WS._isI2CPort0Init)
    return true;
  // Initialize bus
  WS._i2cPort0 = new WipperSnapper_Component_I2C(&msgInitRequest);
  WS.i2cComponents.push_back(WS._i2cPort0);
  WS._isI2CPort0Init = WS._i2cPort0->isInitialized();
  return WS._isI2CPort0Init;
}

/******************************************************************************************/
/*!
    @brief    Decodes a list of I2C Device Initialization messages.
    @param    stream
              Incoming data stream from buffer.
    @param    field
              Protobuf message's tag type.
    @param    arg
              Optional arguments from pb_decode calling function.
    @returns  True if decoded successfully, False otherwise.
*/
/******************************************************************************************/
bool cbDecodeI2CDeviceInitRequestList(pb_istream_t *stream,
                                      const pb_field_t *field, void **arg) {
  WS_DEBUG_PRINTLN("EXEC: cbDecodeI2CDeviceInitRequestList");
  // Decode stream into individual msgI2CDeviceInitRequest messages
  wippersnapper_i2c_v1_I2CDeviceInitRequest msgI2CDeviceInitRequest =
      wippersnapper_i2c_v1_I2CDeviceInitRequest_init_zero;
  if (!pb_decode(stream, wippersnapper_i2c_v1_I2CDeviceInitRequest_fields,
                 &msgI2CDeviceInitRequest)) {
    WS_DEBUG_PRINTLN("ERROR: Could not decode I2CDeviceInitRequest message.");
    return false;
  }

  // Create response
  wippersnapper_signal_v1_I2CResponse msgi2cResponse =
      wippersnapper_signal_v1_I2CResponse_init_zero;
  msgi2cResponse.which_payload =
      wippersnapper_signal_v1_I2CResponse_resp_i2c_device_init_tag;

  // Check I2C bus
  if (!initializeI2CBus(msgI2CDeviceInitRequest.i2c_bus_init_req, 0)) {
    WS_DEBUG_PRINTLN("ERROR: Failed to initialize I2C Bus");
    msgi2cResponse.payload.resp_i2c_device_init.bus_response =
        WS._i2cPort0->getBusStatus();
    if (!encodeI2CResponse(&msgi2cResponse)) {
      WS_DEBUG_PRINTLN("ERROR: encoding I2C Response!");
      return false;
    }
    publishI2CResponse(&msgi2cResponse);
    return true;
  }

  WS._i2cPort0->initI2CDevice(&msgI2CDeviceInitRequest);

  // Fill device's address and the initialization status
  // TODO: The filling should be done within the method though?
  msgi2cResponse.payload.resp_i2c_device_init.i2c_device_address =
      msgI2CDeviceInitRequest.i2c_device_address;
  msgi2cResponse.payload.resp_i2c_device_init.bus_response =
      WS._i2cPort0->getBusStatus();

  // Encode response
  if (!encodeI2CResponse(&msgi2cResponse)) {
    return false;
  }

  // Publish a response for the I2C device
  publishI2CResponse(&msgi2cResponse);
  return true;
}

/******************************************************************************************/
/*!
    @brief    Decodes an I2C signal request message and executes the
              callback based on the message's tag. If successful,
              publishes an I2C signal response back to the broker.
    @param    stream
              Incoming data stream from buffer.
    @param    field
              Protobuf message's tag type.
    @param    arg
              Optional arguments from decoder calling function.
    @returns  True if decoded successfully, False otherwise.
*/
/******************************************************************************************/
bool cbDecodeSignalRequestI2C(pb_istream_t *stream, const pb_field_t *field,
                              void **arg) {
  bool is_success = true;
  WS_DEBUG_PRINTLN("cbDecodeSignalRequestI2C");
  // Create I2C Response
  wippersnapper_signal_v1_I2CResponse msgi2cResponse =
      wippersnapper_signal_v1_I2CResponse_init_zero;

  if (field->tag == wippersnapper_signal_v1_I2CRequest_req_i2c_scan_tag) {
    WS_DEBUG_PRINTLN("I2C Scan Request");

    // Decode I2CBusScanRequest
    wippersnapper_i2c_v1_I2CBusScanRequest msgScanReq =
        wippersnapper_i2c_v1_I2CBusScanRequest_init_zero;
    if (!pb_decode(stream, wippersnapper_i2c_v1_I2CBusScanRequest_fields,
                   &msgScanReq)) {
      WS_DEBUG_PRINTLN(
          "ERROR: Could not decode wippersnapper_i2c_v1_I2CBusScanRequest");
      return false; // fail out if we can't decode the request
    }

    // Empty response message
    wippersnapper_i2c_v1_I2CBusScanResponse scanResp =
        wippersnapper_i2c_v1_I2CBusScanResponse_init_zero;

    // Check I2C bus
    if (!initializeI2CBus(msgScanReq.bus_init_request, 0)) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize I2C Bus");
      msgi2cResponse.payload.resp_i2c_scan.bus_response =
          WS._i2cPort0->getBusStatus();
      if (!encodeI2CResponse(&msgi2cResponse)) {
        WS_DEBUG_PRINTLN("ERROR: encoding I2C Response!");
        return false;
      }
      publishI2CResponse(&msgi2cResponse);
      return true;
    }

    // Scan I2C bus
    scanResp = WS._i2cPort0->scanAddresses();

    // Fill I2CResponse
    msgi2cResponse.which_payload =
        wippersnapper_signal_v1_I2CResponse_resp_i2c_scan_tag;
    memcpy(msgi2cResponse.payload.resp_i2c_scan.addresses_found,
           scanResp.addresses_found, sizeof(scanResp.addresses_found));
    msgi2cResponse.payload.resp_i2c_scan.addresses_found_count =
        scanResp.addresses_found_count;

    msgi2cResponse.payload.resp_i2c_scan.bus_response = scanResp.bus_response;
    // Encode I2CResponse
    if (!encodeI2CResponse(&msgi2cResponse)) {
      return false;
    }
  } else if (
      field->tag ==
      wippersnapper_signal_v1_I2CRequest_req_i2c_device_init_requests_tag) {
    WS_DEBUG_PRINTLN("I2C Device LIST Init Request Found!");

    // Decode stream
    wippersnapper_i2c_v1_I2CDeviceInitRequests msgI2CDeviceInitRequestList =
        wippersnapper_i2c_v1_I2CDeviceInitRequests_init_zero;
    // Set up callback
    msgI2CDeviceInitRequestList.list.funcs.decode =
        cbDecodeI2CDeviceInitRequestList;
    msgI2CDeviceInitRequestList.list.arg = field->pData;
    // Decode each sub-message
    if (!pb_decode(stream, wippersnapper_i2c_v1_I2CDeviceInitRequests_fields,
                   &msgI2CDeviceInitRequestList)) {
      WS_DEBUG_PRINTLN("ERROR: Could not decode I2CDeviceInitRequests");
      is_success = false;
    }
    // return so we don't publish an empty message, we already published within
    // cbDecodeI2CDeviceInitRequestList() for each device
    return is_success;
  } else if (field->tag ==
             wippersnapper_signal_v1_I2CRequest_req_i2c_device_init_tag) {
    WS_DEBUG_PRINTLN("I2C Device Init Request Found!");

    // Decode stream into an I2CDeviceInitRequest
    wippersnapper_i2c_v1_I2CDeviceInitRequest msgI2CDeviceInitRequest =
        wippersnapper_i2c_v1_I2CDeviceInitRequest_init_zero;
    // Decode stream into struct, msgI2CDeviceInitRequest
    if (!pb_decode(stream, wippersnapper_i2c_v1_I2CDeviceInitRequest_fields,
                   &msgI2CDeviceInitRequest)) {
      WS_DEBUG_PRINTLN("ERROR: Could not decode I2CDeviceInitRequest message.");
      return false; // fail out if we can't decode
    }

    // Create empty response
    msgi2cResponse = wippersnapper_signal_v1_I2CResponse_init_zero;
    msgi2cResponse.which_payload =
        wippersnapper_signal_v1_I2CResponse_resp_i2c_device_init_tag;

    // Check I2C bus
    if (!initializeI2CBus(msgI2CDeviceInitRequest.i2c_bus_init_req, 0)) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize I2C Bus");
      msgi2cResponse.payload.resp_i2c_device_init.bus_response =
          WS._i2cPort0->getBusStatus();
      if (!encodeI2CResponse(&msgi2cResponse)) {
        WS_DEBUG_PRINTLN("ERROR: encoding I2C Response!");
        return false;
      }
      publishI2CResponse(&msgi2cResponse);
      return true;
    }

    // Initialize I2C device
    WS._i2cPort0->initI2CDevice(&msgI2CDeviceInitRequest);

    // Fill device's address and bus status
    msgi2cResponse.payload.resp_i2c_device_init.i2c_device_address =
        msgI2CDeviceInitRequest.i2c_device_address;
    msgi2cResponse.payload.resp_i2c_device_init.bus_response =
        WS._i2cPort0->getBusStatus();

    // Encode response
    if (!encodeI2CResponse(&msgi2cResponse)) {
      return false;
    }
  } else if (field->tag ==
             wippersnapper_signal_v1_I2CRequest_req_i2c_device_update_tag) {
    WS_DEBUG_PRINTLN("=> INCOMING REQUEST: I2CDeviceUpdateRequest");

    // New I2CDeviceUpdateRequest message
    wippersnapper_i2c_v1_I2CDeviceUpdateRequest msgI2CDeviceUpdateRequest =
        wippersnapper_i2c_v1_I2CDeviceUpdateRequest_init_zero;

    // Decode stream into message
    if (!pb_decode(stream, wippersnapper_i2c_v1_I2CDeviceUpdateRequest_fields,
                   &msgI2CDeviceUpdateRequest)) {
      WS_DEBUG_PRINTLN(
          "ERROR: Could not decode I2CDeviceUpdateRequest message.");
      return false; // fail out if we can't decode
    }

    // Empty I2C response to fill out
    msgi2cResponse = wippersnapper_signal_v1_I2CResponse_init_zero;
    msgi2cResponse.which_payload =
        wippersnapper_signal_v1_I2CResponse_resp_i2c_device_update_tag;

    // Update I2C device's properties
    WS._i2cPort0->updateI2CDeviceProperties(&msgI2CDeviceUpdateRequest);

    // Fill address
    msgi2cResponse.payload.resp_i2c_device_update.i2c_device_address =
        msgI2CDeviceUpdateRequest.i2c_device_address;
    msgi2cResponse.payload.resp_i2c_device_update.bus_response =
        WS._i2cPort0->getBusStatus();

    // Encode response
    if (!encodeI2CResponse(&msgi2cResponse)) {
      return false;
    }
  } else if (field->tag ==
             wippersnapper_signal_v1_I2CRequest_req_i2c_device_deinit_tag) {
    WS_DEBUG_PRINTLN("NEW COMMAND: I2C Device Deinit");
    // Decode stream into an I2CDeviceDeinitRequest
    wippersnapper_i2c_v1_I2CDeviceDeinitRequest msgI2CDeviceDeinitRequest =
        wippersnapper_i2c_v1_I2CDeviceDeinitRequest_init_zero;
    // Decode stream into struct, msgI2CDeviceDeinitRequest
    if (!pb_decode(stream, wippersnapper_i2c_v1_I2CDeviceDeinitRequest_fields,
                   &msgI2CDeviceDeinitRequest)) {
      WS_DEBUG_PRINTLN(
          "ERROR: Could not decode I2CDeviceDeinitRequest message.");
      return false; // fail out if we can't decode
    }

    // Empty I2C response to fill out
    msgi2cResponse = wippersnapper_signal_v1_I2CResponse_init_zero;
    msgi2cResponse.which_payload =
        wippersnapper_signal_v1_I2CResponse_resp_i2c_device_deinit_tag;

    // Deinitialize I2C device
    WS._i2cPort0->deinitI2CDevice(&msgI2CDeviceDeinitRequest);
    // Fill deinit response
    msgi2cResponse.payload.resp_i2c_device_deinit.i2c_device_address =
        msgI2CDeviceDeinitRequest.i2c_device_address;
    msgi2cResponse.payload.resp_i2c_device_deinit.bus_response =
        WS._i2cPort0->getBusStatus();

    // Encode response
    if (!encodeI2CResponse(&msgi2cResponse)) {
      return false;
    }
  } else {
    WS_DEBUG_PRINTLN("ERROR: Undefined I2C message tag");
    return false; // fail out, we didn't encode anything to publish
  }
  // Publish the I2CResponse
  publishI2CResponse(&msgi2cResponse);
  return is_success;
}

/******************************************************************************************/
/*!
    @brief    Decodes a servo message and dispatches to the servo component.
    @param    stream
              Incoming data stream from buffer.
    @param    field
              Protobuf message's tag type.
    @param    arg
              Optional arguments from decoder calling function.
    @returns  True if decoded and executed successfully, False otherwise.
*/
/******************************************************************************************/
bool cbDecodeServoMsg(pb_istream_t *stream, const pb_field_t *field,
                      void **arg) {
  bool is_success = true;
  WS_DEBUG_PRINTLN("Decoding Servo Message...");

  if (field->tag == wippersnapper_signal_v1_ServoRequest_servo_attach_tag) {
    WS_DEBUG_PRINTLN("GOT: Servo Attach");
    // Attempt to decode contents of servo_attach message
    wippersnapper_servo_v1_ServoAttachReq msgServoAttachReq =
        wippersnapper_servo_v1_ServoAttachReq_init_zero;
    if (!pb_decode(stream, wippersnapper_servo_v1_ServoAttachReq_fields,
                   &msgServoAttachReq)) {
      WS_DEBUG_PRINTLN(
          "ERROR: Could not decode wippersnapper_servo_v1_ServoAttachReq");
      return false; // fail out if we can't decode the request
    }
    // execute servo attach request
    char *servoPin = msgServoAttachReq.servo_pin + 1;
    bool attached = true;
    if (!WS._servoComponent->servo_attach(
            atoi(servoPin), msgServoAttachReq.min_pulse_width,
            msgServoAttachReq.max_pulse_width, msgServoAttachReq.servo_freq)) {
      WS_DEBUG_PRINTLN("ERROR: Unable to attach servo to pin!");
      attached = false;
    }

    // Create and fill a servo response message
    size_t msgSz; // message's encoded size
    wippersnapper_signal_v1_ServoResp msgServoResp =
        wippersnapper_signal_v1_ServoResp_init_zero;
    msgServoResp.which_payload =
        wippersnapper_signal_v1_ServoResp_servo_attach_resp_tag;
    msgServoResp.payload.servo_attach_resp.attach_success = attached;

    // Encode and publish response back to broker
    memset(WS._buffer_outgoing, 0, sizeof(WS._buffer_outgoing));
    pb_ostream_t ostream = pb_ostream_from_buffer(WS._buffer_outgoing,
                                                  sizeof(WS._buffer_outgoing));
    if (!pb_encode(&ostream, wippersnapper_signal_v1_ServoResp_fields,
                   &msgServoResp)) {
      WS_DEBUG_PRINTLN("ERROR: Unable to encode servo response message!");
      return false;
    }
    pb_get_encoded_size(&msgSz, wippersnapper_signal_v1_ServoResp_fields,
                        &msgServoResp);
    WS_DEBUG_PRINT("-> Servo Attach Response...");
    WS._mqtt->publish(WS._topic_signal_servo_device, WS._buffer_outgoing, msgSz,
                      1);
    WS_DEBUG_PRINTLN("Published!");
    return true;
  } else if (field->tag ==
             wippersnapper_signal_v1_ServoRequest_servo_write_tag) {
    WS_DEBUG_PRINTLN("GOT: Servo Write");
    // TODO: perform servo write
  } else if (field->tag ==
             wippersnapper_signal_v1_ServoRequest_servo_detach_tag) {
    WS_DEBUG_PRINTLN("GOT: Servo Detach");

    // Attempt to decode contents of servo detach message
    wippersnapper_servo_v1_ServoDetachReq msgServoDetachReq =
        wippersnapper_servo_v1_ServoDetachReq_init_zero;

    if (!pb_decode(stream, wippersnapper_servo_v1_ServoDetachReq_fields,
                   &msgServoDetachReq)) {
      WS_DEBUG_PRINTLN(
          "ERROR: Could not decode wippersnapper_servo_v1_ServoDetachReq");
      return false; // fail out if we can't decode the request
    }
    // execute servo detach request
    char *servoPin = msgServoDetachReq.servo_pin + 1;
    WS._servoComponent->servo_detach(atoi(servoPin));
  } else {
    WS_DEBUG_PRINTLN("Unable to decode servo message type!");
    return false;
  }

  return true;
}

/**************************************************************************/
/*!
    @brief    Called when the device recieves a new message from the
              /servo/ topic.
    @param    data
              Incoming data from MQTT broker.
    @param    len
              Length of incoming data.
*/
/**************************************************************************/
void cbServoMsg(char *data, uint16_t len) {
  WS_DEBUG_PRINTLN("* NEW MESSAGE [Topic: Servo]: ");
  WS_DEBUG_PRINT(len);
  WS_DEBUG_PRINTLN(" bytes.");
  // zero-out current buffer
  memset(WS._buffer, 0, sizeof(WS._buffer));
  // copy mqtt data into buffer
  memcpy(WS._buffer, data, len);
  WS.bufSize = len;

  // Zero-out existing servo message
  wippersnapper_signal_v1_ServoRequest msgServo =
      wippersnapper_signal_v1_ServoRequest_init_zero;

  // Set up the payload callback, which will set up the callbacks for
  // each oneof payload field once the field tag is known
  WS.msgServo.cb_payload.funcs.decode = cbDecodeServoMsg;

  // Decode servo message from buffer
  pb_istream_t istream = pb_istream_from_buffer(WS._buffer, WS.bufSize);
  if (!pb_decode(&istream, wippersnapper_signal_v1_ServoRequest_fields,
                 &WS.msgServo))
    WS_DEBUG_PRINTLN("ERROR: Unable to decode servo message");
}

/**************************************************************************/
/*!
    @brief    Called when i2c signal sub-topic receives a new message and
              attempts to decode a signal request message.
    @param    data
              Incoming data from MQTT broker.
    @param    len
              Length of incoming data.
*/
/**************************************************************************/
void cbSignalI2CReq(char *data, uint16_t len) {
  WS_DEBUG_PRINTLN("* NEW MESSAGE [Topic: Signal-I2C]: ");
  WS_DEBUG_PRINT(len);
  WS_DEBUG_PRINTLN(" bytes.");
  // zero-out current buffer
  memset(WS._buffer, 0, sizeof(WS._buffer));
  // copy mqtt data into buffer
  memcpy(WS._buffer, data, len);
  WS.bufSize = len;

  // Zero-out existing I2C signal msg.
  WS.msgSignalI2C = wippersnapper_signal_v1_I2CRequest_init_zero;

  // Set up the payload callback, which will set up the callbacks for
  // each oneof payload field once the field tag is known
  WS.msgSignalI2C.cb_payload.funcs.decode = cbDecodeSignalRequestI2C;

  // Decode I2C signal request
  pb_istream_t istream = pb_istream_from_buffer(WS._buffer, WS.bufSize);
  if (!pb_decode(&istream, wippersnapper_signal_v1_I2CRequest_fields,
                 &WS.msgSignalI2C))
    WS_DEBUG_PRINTLN("ERROR: Unable to decode I2C message");
}

/****************************************************************************/
/*!
    @brief    Handles MQTT messages on signal topic until timeout.
    @param    outgoingSignalMsg
                Empty signal message struct.
    @param    pinName
                Name of pin.
    @param    pinVal
                Value of pin.
    @returns  True if pinEvent message encoded successfully, false otherwise.
*/
/****************************************************************************/
bool Wippersnapper::encodePinEvent(
    wippersnapper_signal_v1_CreateSignalRequest *outgoingSignalMsg,
    uint8_t pinName, int pinVal) {
  bool is_success = true;
  outgoingSignalMsg->which_payload =
      wippersnapper_signal_v1_CreateSignalRequest_pin_event_tag;
  // fill the pin_event message
  sprintf(outgoingSignalMsg->payload.pin_event.pin_name, "D%d", pinName);
  sprintf(outgoingSignalMsg->payload.pin_event.pin_value, "%d", pinVal);

  // Encode signal message
  pb_ostream_t stream =
      pb_ostream_from_buffer(WS._buffer_outgoing, sizeof(WS._buffer_outgoing));
  if (!pb_encode(&stream, wippersnapper_signal_v1_CreateSignalRequest_fields,
                 outgoingSignalMsg)) {
    WS_DEBUG_PRINTLN("ERROR: Unable to encode signal message");
    is_success = false;
  }

  return is_success;
}

/**************************************************************************/
/*!
    @brief    Called when broker responds to a device's publish across
              the registration topic.
    @param    data
                Data from MQTT broker.
    @param    len
                Length of data received from MQTT broker.
*/
/**************************************************************************/
void cbRegistrationStatus(char *data, uint16_t len) {
  // call decoder for registration response msg
  WS.decodeRegistrationResp(data, len);
}

/**************************************************************************/
/*!
    @brief    Attempts to re-connect to the MQTT broker, retries with
                an exponential backoff + jitter interval.
*/
/**************************************************************************/
void retryMQTTConnection() {
  // MQTT broker's connack return code
  int8_t rc;
  // amount of times we've attempted to re-connect to IO's MQTT broker
  int retries = 0;
  // maximum backoff time, in millis
  double maxBackoff = 60000;
  // current backoff time, in millis
  double backoff;
  // randomized jitter to prevent multi-client collisions
  long jitter;

  bool notConnected = true;
  while (notConnected == true) {
    WS_DEBUG_PRINTLN("Retrying connection...");
    // attempt reconnection, save return code (rc)
    rc = WS._mqtt->connect(WS._username, WS._key);
    switch (rc) {
    case WS_MQTT_CONNECTED:
      WS_DEBUG_PRINTLN("Re-connected to IO MQTT!");
      notConnected = false;
      break;
    case WS_MQTT_INVALID_PROTOCOL:
      WS_DEBUG_PRINTLN("Invalid MQTT protocol");
      break;
    case WS_MQTT_INVALID_CID:
      WS_DEBUG_PRINTLN("client ID rejected");
      break;
    case WS_MQTT_SERVICE_UNAVALIABLE:
      WS_DEBUG_PRINTLN("MQTT service unavailable");
      break;
    case WS_MQTT_INVALID_USER_PASS:
      WS_DEBUG_PRINTLN("malformed user/pass");
      break;
    case WS_MQTT_UNAUTHORIZED:
      WS_DEBUG_PRINTLN("unauthorized");
      break;
    case WS_MQTT_THROTTLED:
      WS_DEBUG_PRINTLN("ERROR: Throttled");
      break;
    case WS_MQTT_BANNED:
      WS_DEBUG_PRINTLN("ERROR: Temporarily banned");
      break;
    default:
      break;
    }
    retries++;
    if (notConnected) {
      WS_DEBUG_PRINTLN("Not connected, delaying...");
      // calculate a jitter value btween 0ms and 100ms
      jitter = random(0, 100);
      // calculate exponential backoff w/jitter
      backoff = (pow(2, retries) * 1000) + jitter;
      backoff = min(backoff, maxBackoff);
      WS_DEBUG_PRINT("Delaying for ");
      WS_DEBUG_PRINT(backoff);
      WS_DEBUG_PRINTLN("ms...");
      // delay for backoff millis
      delay(backoff);
    } else {
      WS_DEBUG_PRINTLN("Connected to MQTT broker!")
      // reset backoff param and retries
      backoff = 0;
    }
  }
}

/**************************************************************************/
/*!
    @brief    Called when client receives a message published across the
                Adafruit IO MQTT /error special topic.
    @param    errorData
                Data from MQTT broker.
    @param    len
                Length of data received from MQTT broker.
*/
/**************************************************************************/
void cbErrorTopic(char *errorData, uint16_t len) {
  WS_DEBUG_PRINT("IO Ban Error: ");
  WS_DEBUG_PRINTLN(errorData);
  // Disconnect client from broker
  WS_DEBUG_PRINT("Disconnecting from MQTT..");
  if (!WS._mqtt->disconnect()) {
    WS_DEBUG_PRINTLN("ERROR: Unable to disconnect from MQTT broker!");
  }
  // WDT reset
  for (;;) {
    delay(100);
  }
}

/**************************************************************************/
/*!
    @brief    Called when client receives a message published across the
                Adafruit IO MQTT /throttle special topic. Delays until
                throttle is released.
    @param    throttleData
                Throttle message from Adafruit IO.
    @param    len
                Length of data received from MQTT broker.
*/
/**************************************************************************/
void cbThrottleTopic(char *throttleData, uint16_t len) {
  WS_DEBUG_PRINT("IO Throttle Error: ");
  WS_DEBUG_PRINTLN(throttleData);
  char *throttleMessage;
  // Parse out # of seconds from message buffer
  throttleMessage = strtok(throttleData, ",");
  throttleMessage = strtok(NULL, " ");
  // Convert from seconds to to millis
  int throttleDuration = atoi(throttleMessage) * 1000;

  WS_DEBUG_PRINT("Device is throttled for ");
  WS_DEBUG_PRINT(throttleDuration);
  WS_DEBUG_PRINTLN("ms and blocking command execution.");
  if (throttleDuration < WS_KEEPALIVE_INTERVAL_MS) {
    delay(WS_KEEPALIVE_INTERVAL_MS);
  } else {
    // round to nearest millis to prevent delaying for less time than req'd.
    float throttleLoops = ceil(throttleDuration / WS_KEEPALIVE_INTERVAL_MS);
    // block the run() loop
    while (throttleLoops > 0) {
      delay(WS_KEEPALIVE_INTERVAL_MS);
      WS.feedWDT();
      WS._mqtt->ping();
      throttleLoops--;
    }
  }
  WS_DEBUG_PRINTLN("Device is un-throttled, resumed command execution");
}

/**************************************************************************/
/*!
    @brief    Builds MQTT topics for handling errors returned from the
                Adafruit IO broker.
    @returns  True if memory for error topics allocated successfully,
                False otherwise.
*/
/**************************************************************************/
bool Wippersnapper::buildErrorTopics() {
  bool is_success = true;
  // dynamically allocate memory for err topic
  WS._err_topic = (char *)malloc(
      sizeof(char) * (strlen(WS._username) + strlen(TOPIC_IO_ERRORS) + 1));

  if (WS._err_topic) { // build error topic
    strcpy(WS._err_topic, WS._username);
    strcat(WS._err_topic, TOPIC_IO_ERRORS);
  } else { // malloc failed
    WS._err_topic = 0;
    is_success = false;
  }

  // dynamically allocate memory for throttle topic
  WS._throttle_topic = (char *)malloc(
      sizeof(char) * (strlen(WS._username) + strlen(TOPIC_IO_THROTTLE) + 1));

  if (WS._throttle_topic) { // build throttle topic
    strcpy(WS._throttle_topic, WS._username);
    strcat(WS._throttle_topic, TOPIC_IO_THROTTLE);
  } else { // malloc failed
    WS._throttle_topic = 0;
    is_success = false;
  }
  return is_success;
}

/**************************************************************************/
/*!
    @brief    Subscribes to user-specific Adafruit IO MQTT topics
*/
/**************************************************************************/
void Wippersnapper::subscribeErrorTopics() {
  // Subscribe to error topic
  _err_sub = new Adafruit_MQTT_Subscribe(WS._mqtt, WS._err_topic);
  WS._mqtt->subscribe(_err_sub);
  _err_sub->setCallback(cbErrorTopic);

  // Subscribe to throttle topic
  _throttle_sub = new Adafruit_MQTT_Subscribe(WS._mqtt, WS._throttle_topic);
  WS._mqtt->subscribe(_throttle_sub);
  _throttle_sub->setCallback(cbThrottleTopic);
}

/**************************************************************************/
/*!
    @brief    Generates device-specific Wippersnapper control topics.
    @returns  True if memory for control topics allocated successfully,
                False otherwise.
*/
/**************************************************************************/
bool Wippersnapper::buildWSTopics() {
  bool is_success = true;

  // Validate that we've correctly pulled configuration keys from the FS
  if (WS._username == NULL || WS._key == NULL || WS._network_ssid == NULL ||
      WS._network_pass == NULL)
    return false;

  // Create device unique identifier
  // Move the top 3 bytes from the UID
  for (int i = 5; i > 2; i--) {
    WS._macAddr[6 - 1 - i] = WS._macAddr[i];
  }
  snprintf(WS.sUID, sizeof(WS.sUID), "%02d%02d%02d", WS._macAddr[0],
           WS._macAddr[1], WS._macAddr[2]);

  // Set machine_name
  WS._boardId = BOARD_ID;
  // Conversion to match integer UID sent by encodePubRegistrationReq()
  int32_t iUID = atoi(WS.sUID);
  char mac_uid[13];
  itoa(iUID, mac_uid, 10);

  _device_uid = (char *)malloc(sizeof(char) + strlen("io-wipper-") +
                               strlen(WS._boardId) + strlen(mac_uid) + 1);
  strcpy(_device_uid, "io-wipper-");
  strcat(_device_uid, WS._boardId);
  strcat(_device_uid, mac_uid);

  // Initialize MQTT client with UID as the ClientID
  setupMQTTClient(_device_uid);

  // Global registration topic
  WS._topic_description =
      (char *)malloc(sizeof(char) * strlen(WS._username) + strlen("/wprsnpr") +
                     strlen(TOPIC_INFO) + strlen("status") + 1);

  // Registration status topic
  WS._topic_description_status =
      (char *)malloc(sizeof(char) * strlen(WS._username) + strlen("/wprsnpr/") +
                     strlen(_device_uid) + strlen(TOPIC_INFO) +
                     strlen("status/") + strlen("broker") + 1);

  // Registration status completion topic
  WS._topic_description_status_complete =
      (char *)malloc(sizeof(char) * strlen(WS._username) + strlen("/wprsnpr/") +
                     strlen(_device_uid) + strlen(TOPIC_INFO) +
                     strlen("status") + strlen("/device/complete") + 1);

  // Topic to signal pin configuration complete from device to broker
  WS._topic_device_pin_config_complete =
      (char *)malloc(sizeof(char) * strlen(WS._username) + strlen("/wprsnpr/") +
                     strlen(_device_uid) + strlen(TOPIC_SIGNALS) +
                     strlen("device/pinConfigComplete") + 1);

  // Topic for signals from device to broker
  WS._topic_signal_device = (char *)malloc(
      sizeof(char) * strlen(WS._username) + strlen("/wprsnpr/") +
      strlen(_device_uid) + strlen(TOPIC_SIGNALS) + strlen("device") + 1);

  // Topic for signals from broker to device
  WS._topic_signal_brkr = (char *)malloc(
      sizeof(char) * strlen(WS._username) + strlen("/wprsnpr/") +
      strlen(_device_uid) + strlen(TOPIC_SIGNALS) + strlen("broker") + 1);

  // Topic for i2c signals from broker to device
  WS._topic_signal_i2c_brkr = (char *)malloc(
      sizeof(char) * strlen(WS._username) + +strlen("/") + strlen(_device_uid) +
      strlen("/wprsnpr/") + strlen(TOPIC_SIGNALS) + strlen("broker") +
      strlen(TOPIC_I2C) + 1);

  // Topic for i2c signals from device to broker
  WS._topic_signal_i2c_device = (char *)malloc(
      sizeof(char) * strlen(WS._username) + +strlen("/") + strlen(_device_uid) +
      strlen("/wprsnpr/") + strlen(TOPIC_SIGNALS) + strlen("device") +
      strlen(TOPIC_I2C) + 1);

  // Topic for servo messages from broker->device
  WS._topic_signal_servo_brkr = (char *)malloc(
      sizeof(char) * strlen(WS._username) + strlen("/") + strlen(_device_uid) +
      strlen("/wprsnpr/signals/broker/servo") + 1);

  // Topic for servo messages from device->broker
  WS._topic_signal_servo_device = (char *)malloc(
      sizeof(char) * strlen(WS._username) + strlen("/") + strlen(_device_uid) +
      strlen("/wprsnpr/signals/device/servo") + 1);

  // Create global registration topic
  if (WS._topic_description != NULL) {
    strcpy(WS._topic_description, WS._username);
    strcat(WS._topic_description, "/wprsnpr");
    strcat(WS._topic_description, TOPIC_INFO);
    strcat(WS._topic_description, "status");
  } else { // malloc failed
    is_success = false;
  }

  // Create registration status topic
  if (WS._topic_description_status != NULL) {
    strcpy(WS._topic_description_status, WS._username);
    strcat(WS._topic_description_status, "/wprsnpr/");
    strcat(WS._topic_description_status, _device_uid);
    strcat(WS._topic_description_status, TOPIC_INFO);
    strcat(WS._topic_description_status, "status");
    strcat(WS._topic_description_status, "/broker");
  } else { // malloc failed
    is_success = false;
  }

  // Create registration status complete topic
  if (WS._topic_description_status_complete != NULL) {
    strcpy(WS._topic_description_status_complete, WS._username);
    strcat(WS._topic_description_status_complete, "/wprsnpr/");
    strcat(WS._topic_description_status_complete, _device_uid);
    strcat(WS._topic_description_status_complete, TOPIC_INFO);
    strcat(WS._topic_description_status_complete, "status");
    strcat(WS._topic_description_status_complete, "/device/complete");
  } else { // malloc failed
    is_success = false;
  }

  // Create device-to-broker signal topic
  if (WS._topic_signal_device != NULL) {
    strcpy(WS._topic_signal_device, WS._username);
    strcat(WS._topic_signal_device, "/wprsnpr/");
    strcat(WS._topic_signal_device, _device_uid);
    strcat(WS._topic_signal_device, TOPIC_SIGNALS);
    strcat(WS._topic_signal_device, "device");
  } else { // malloc failed
    is_success = false;
  }

  // Create device-to-broker signal topic
  if (WS._topic_device_pin_config_complete != NULL) {
    strcpy(WS._topic_device_pin_config_complete, WS._username);
    strcat(WS._topic_device_pin_config_complete, "/wprsnpr/");
    strcat(WS._topic_device_pin_config_complete, _device_uid);
    strcat(WS._topic_device_pin_config_complete, TOPIC_SIGNALS);
    strcat(WS._topic_device_pin_config_complete, "device/pinConfigComplete");
  } else { // malloc failed
    is_success = false;
  }

  // Create broker-to-device signal topic
  if (WS._topic_signal_brkr != NULL) {
    strcpy(WS._topic_signal_brkr, WS._username);
    strcat(WS._topic_signal_brkr, "/wprsnpr/");
    strcat(WS._topic_signal_brkr, _device_uid);
    strcat(WS._topic_signal_brkr, TOPIC_SIGNALS);
    strcat(WS._topic_signal_brkr, "broker");
  } else { // malloc failed
    is_success = false;
  }

  // Create device-to-broker i2c signal topic
  if (WS._topic_signal_i2c_brkr != NULL) {
    strcpy(WS._topic_signal_i2c_brkr, WS._username);
    strcat(WS._topic_signal_i2c_brkr, TOPIC_WS);
    strcat(WS._topic_signal_i2c_brkr, _device_uid);
    strcat(WS._topic_signal_i2c_brkr, TOPIC_SIGNALS);
    strcat(WS._topic_signal_i2c_brkr, "broker");
    strcat(WS._topic_signal_i2c_brkr, TOPIC_I2C);
  } else { // malloc failed
    is_success = false;
  }

  // Create broker-to-device i2c signal topic
  if (WS._topic_signal_i2c_device != NULL) {
    strcpy(WS._topic_signal_i2c_device, WS._username);
    strcat(WS._topic_signal_i2c_device, TOPIC_WS);
    strcat(WS._topic_signal_i2c_device, _device_uid);
    strcat(WS._topic_signal_i2c_device, TOPIC_SIGNALS);
    strcat(WS._topic_signal_i2c_device, "device");
    strcat(WS._topic_signal_i2c_device, TOPIC_I2C);
  } else { // malloc failed
    is_success = false;
  }

  // Create device-to-broker servo signal topic
  if (WS._topic_signal_i2c_device != NULL) {
    strcpy(WS._topic_signal_i2c_device, WS._username);
    strcat(WS._topic_signal_i2c_device, TOPIC_WS);
    strcat(WS._topic_signal_i2c_device, _device_uid);
    strcat(WS._topic_signal_i2c_device, TOPIC_SIGNALS);
    strcat(WS._topic_signal_i2c_device, "broker/servo");
  } else { // malloc failed
    is_success = false;
  }

  // Create broker-to-device i2c signal topic
  if (WS._topic_signal_i2c_device != NULL) {
    strcpy(WS._topic_signal_i2c_device, WS._username);
    strcat(WS._topic_signal_i2c_device, TOPIC_WS);
    strcat(WS._topic_signal_i2c_device, _device_uid);
    strcat(WS._topic_signal_i2c_device, TOPIC_SIGNALS);
    strcat(WS._topic_signal_i2c_device, "device/servo");
  } else { // malloc failed
    is_success = false;
  }

  return is_success;
}

/**************************************************************************/
/*!
    @brief    Subscribes to device-specific MQTT control topics.
*/
/**************************************************************************/
void Wippersnapper::subscribeWSTopics() {
  // Subscribe to signal topic
  _topic_signal_brkr_sub =
      new Adafruit_MQTT_Subscribe(WS._mqtt, WS._topic_signal_brkr, 1);
  WS._mqtt->subscribe(_topic_signal_brkr_sub);
  _topic_signal_brkr_sub->setCallback(cbSignalTopic);

  // Subscribe to signal's I2C sub-topic
  _topic_signal_i2c_sub =
      new Adafruit_MQTT_Subscribe(WS._mqtt, WS._topic_signal_i2c_brkr, 1);
  WS._mqtt->subscribe(_topic_signal_i2c_sub);
  _topic_signal_i2c_sub->setCallback(cbSignalI2CReq);

  // Subscribe to servo sub-topic
  _topic_signal_servo_sub =
      new Adafruit_MQTT_Subscribe(WS._mqtt, WS._topic_signal_servo_brkr, 1);
  WS._mqtt->subscribe(_topic_signal_servo_sub);
  _topic_signal_servo_sub->setCallback(cbServoMsg);

  // Subscribe to registration status topic
  _topic_description_sub =
      new Adafruit_MQTT_Subscribe(WS._mqtt, WS._topic_description_status, 1);
  WS._mqtt->subscribe(_topic_description_sub);
  _topic_description_sub->setCallback(cbRegistrationStatus);
}

/**************************************************************************/
/*!
    @brief    Writes an error message to the serial and the filesystem,
                blinks WS_LED_STATUS_ERROR_RUNTIME pattern and hangs.
    @param    error
              The error message to write to the serial and filesystem.
*/
/**************************************************************************/
void Wippersnapper::errorWriteHang(String error) {
  // Print error
  WS_DEBUG_PRINTLN(error);
#ifdef USE_TINYUSB
  _fileSystem->writeToBootOut(error.c_str());
#endif
  // Signal and hang forever
  while (1) {
    WS.feedWDT();
    statusLEDBlink(WS_LED_STATUS_ERROR_RUNTIME);
    delay(1000);
  }
}

/**************************************************************************/
/*!
    @brief    Checks network and MQTT connectivity. Handles network
              re-connection and mqtt re-establishment.
*/
/**************************************************************************/
void Wippersnapper::runNetFSM() {
  WS.feedWDT();
  // Initial state
  fsm_net_t fsmNetwork;
  fsmNetwork = FSM_NET_CHECK_MQTT;
  int maxAttempts;
  while (fsmNetwork != FSM_NET_CONNECTED) {
    switch (fsmNetwork) {
    case FSM_NET_CHECK_MQTT:
      // WS_DEBUG_PRINTLN("FSM_NET_CHECK_MQTT");
      if (WS._mqtt->connected()) {
        fsmNetwork = FSM_NET_CONNECTED;
        return;
      }
      fsmNetwork = FSM_NET_CHECK_NETWORK;
      break;
    case FSM_NET_CHECK_NETWORK:
      // WS_DEBUG_PRINTLN("FSM_NET_CHECK_NETWORK");
      if (networkStatus() == WS_NET_CONNECTED) {
        // WS_DEBUG_PRINTLN("Connected to WiFi");
        fsmNetwork = FSM_NET_ESTABLISH_MQTT;
        break;
      }
      fsmNetwork = FSM_NET_ESTABLISH_NETWORK;
      break;
    case FSM_NET_ESTABLISH_NETWORK:
      // WS_DEBUG_PRINTLN("FSM_NET_ESTABLISH_NETWORK");
      // Attempt to connect to wireless network
      maxAttempts = 5;
      while (maxAttempts > 0) {
        // blink before we connect
        statusLEDBlink(WS_LED_STATUS_WIFI_CONNECTING);
        WS.feedWDT();
        // attempt to connect
        WS_DEBUG_PRINTLN("Attempting to connect to WiFi...");
        _connect();
        WS.feedWDT();
        // blink to simulate a delay to allow wifi connection to process
        statusLEDBlink(WS_LED_STATUS_WIFI_CONNECTING);
        // did we connect?
        if (networkStatus() == WS_NET_CONNECTED)
          break;
        maxAttempts--;
      }
      // Validate connection
      if (networkStatus() != WS_NET_CONNECTED)
        haltError("ERROR: Unable to connect to WiFi, rebooting soon...",
                  WS_LED_STATUS_WIFI_CONNECTING);
      fsmNetwork = FSM_NET_CHECK_NETWORK;
      break;
    case FSM_NET_ESTABLISH_MQTT:
      WS_DEBUG_PRINTLN("FSM_NET_ESTABLISH_MQTT");
      WS._mqtt->setKeepAliveInterval(WS_KEEPALIVE_INTERVAL);
      // Attempt to connect
      maxAttempts = 5;
      while (maxAttempts > 0) {
        statusLEDBlink(WS_LED_STATUS_MQTT_CONNECTING);
        int8_t mqttRC = WS._mqtt->connect();
        if (mqttRC == WS_MQTT_CONNECTED) {
          fsmNetwork = FSM_NET_CHECK_MQTT;
          break;
        }
        WS_DEBUG_PRINTLN(
            "Unable to connect to Adafruit IO MQTT, retrying in 3 seconds...");
        statusLEDBlink(WS_LED_STATUS_MQTT_CONNECTING);
        delay(1800);
        maxAttempts--;
      }
      if (fsmNetwork != FSM_NET_CHECK_MQTT)
        haltError(
            "ERROR: Unable to connect to Adafruit.IO MQTT, rebooting soon...",
            WS_LED_STATUS_MQTT_CONNECTING);
      break;
    default:
      break;
    }
  }
}

/**************************************************************************/
/*!
    @brief    Prints an error to the serial and halts the hardware until
              the WDT bites.
    @param    error
              The desired error to print to serial.
    @param    ledStatusColor
              The desired color to blink.
*/
/**************************************************************************/
void Wippersnapper::haltError(String error, ws_led_status_t ledStatusColor) {
  WS_DEBUG_PRINT("ERROR [WDT RESET]: ");
  WS_DEBUG_PRINTLN(error);
  for (;;) {
    statusLEDSolid(ledStatusColor);
    // let the WDT fail out and reset!
    delay(100);
  }
}

/**************************************************************************/
/*!
    @brief    Attempts to register hardware with Adafruit.io WipperSnapper.
    @returns  True if successful, False otherwise.
*/
/**************************************************************************/
bool Wippersnapper::registerBoard() {
  WS_DEBUG_PRINTLN("Registering hardware with IO...");

  // Encode and publish registration request message to broker
  runNetFSM();
  WS.feedWDT();
  WS_DEBUG_PRINT("Encoding registration request...");
  if (!encodePubRegistrationReq())
    return false;

  // Blocking, attempt to obtain broker's response message
  runNetFSM();
  WS.feedWDT();
  pollRegistrationResp();

  return true;
}

/**************************************************************************/
/*!
    @brief    Returns the board definition status
    @return   Wippersnapper board definition status
*/
/**************************************************************************/
ws_board_status_t Wippersnapper::getBoardStatus() { return WS._boardStatus; }

/**************************************************************************/
/*!
    @brief  Pings the MQTT broker within the keepalive interval
            to keep the connection alive. Blinks the keepalive LED
            every STATUS_LED_KAT_BLINK_TIME milliseconds.
*/
/**************************************************************************/
void Wippersnapper::pingBroker() {
  // ping within keepalive to keep connection open
  if (millis() > (_prv_ping + WS_KEEPALIVE_INTERVAL_MS)) {
    WS_DEBUG_PRINTLN("PING!");
    WS._mqtt->ping();
    _prv_ping = millis();
  }
  // blink status LED every STATUS_LED_KAT_BLINK_TIME millis
  if (millis() > (_prvKATBlink + STATUS_LED_KAT_BLINK_TIME)) {
    statusLEDBlink(WS_LED_STATUS_KAT);
    _prvKATBlink = millis();
  }
}

/********************************************************/
/*!
    @brief    Feeds the WDT to prevent hardware reset.
*/
/*******************************************************/
void Wippersnapper::feedWDT() {
#ifndef ESP8266
  Watchdog.reset();
#endif
}

/********************************************************/
/*!
    @brief  Enables the watchdog timer.
    @param  timeoutMS
            The desired amount of time to elapse before
            the WDT executes.
*/
/*******************************************************/
void Wippersnapper::enableWDT(int timeoutMS) {
#ifndef ESP8266
  Watchdog.disable();
  if (Watchdog.enable(timeoutMS) == 0) {
    WS_DEBUG_PRINTLN("ERROR: WDT initialization failure!");
    setStatusLEDColor(LED_ERROR);
    for (;;) {
      delay(100);
    }
  }
#endif
}

/********************************************************/
/*!
    @brief  Process all incoming packets from the
            Adafruit IO MQTT broker. Handles network
            connectivity.
*/
/*******************************************************/
void Wippersnapper::processPackets() {
  // runNetFSM(); // NOTE: Removed for now, causes error with virtual _connect
  // method when caused with WS object in another file.
  WS.feedWDT();
  // Process all incoming packets from Wippersnapper MQTT Broker
  WS._mqtt->processPackets(10);
}

/********************************************************/
/*!
    @brief  Publishes a message to the Adafruit IO
            MQTT broker. Handles network connectivity.
    @param  topic
            The MQTT topic to publish to.
    @param  payload
            The payload to publish.
    @param  bLen
            The length of the payload.
    @param  qos
            The Quality of Service to publish with.
*/
/*******************************************************/
void Wippersnapper::publish(const char *topic, uint8_t *payload, uint16_t bLen,
                            uint8_t qos) {
  // runNetFSM(); // NOTE: Removed for now, causes error with virtual _connect
  // method when caused with WS object in another file.
  WS.feedWDT();
  WS._mqtt->publish(topic, payload, bLen, qos);
}

/**************************************************************************/
/*!
    @brief    Checks validity of WipperSnapper application credentials.
    @returns  True if WipperSnapper application credentials exist
                and are valid, False otherwise.
*/
/**************************************************************************/
bool validateAppCreds() {
  // Check if null
  if (WS._username == 0 || WS._key == 0 || WS._network_ssid == 0 ||
      WS._network_pass == 0)
    return false;
  // Check credential length
  if (strlen(WS._username) == 0 || strlen(WS._key) == 0 ||
      strlen(WS._network_ssid) == 0 || strlen(WS._network_pass) == 0)
    return false;
  return true;
}

/**************************************************************************/
/*!
    @brief    Connects to Adafruit IO+ Wippersnapper broker.
*/
/**************************************************************************/
void Wippersnapper::connect() {
  WS_DEBUG_PRINTLN("Adafruit.io WipperSnapper");

  // Print all identifiers to the debug log
  WS_DEBUG_PRINTLN("-------Device Information-------");

  WS_DEBUG_PRINT("Firmware Version: ");
  WS_DEBUG_PRINTLN(WS_VERSION);

  WS_DEBUG_PRINT("Board ID: ");
  WS_DEBUG_PRINTLN(BOARD_ID);

  WS_DEBUG_PRINT("Adafruit.io User: ");
  WS_DEBUG_PRINTLN(WS._username);

  WS_DEBUG_PRINT("WiFi Network: ");
  WS_DEBUG_PRINTLN(WS._network_ssid);

  char sMAC[18] = {0};
  sprintf(sMAC, "%02X:%02X:%02X:%02X:%02X:%02X", WS._macAddr[0], WS._macAddr[1],
          WS._macAddr[2], WS._macAddr[3], WS._macAddr[4], WS._macAddr[5]);
  WS_DEBUG_PRINT("MAC Address: ");
  WS_DEBUG_PRINTLN(sMAC);
  WS_DEBUG_PRINTLN("-------------------------------");

  // enable WDT
  WS.enableWDT(WS_WDT_TIMEOUT);

  _status = WS_IDLE;
  WS._boardStatus = WS_BOARD_DEF_IDLE;

  if (!validateAppCreds())
    haltError("Unable to validate application credentials.");

  // build MQTT topics for WipperSnapper and subscribe
  if (!buildWSTopics()) {
    haltError("Unable to allocate space for MQTT topics");
  }
  if (!buildErrorTopics()) {
    haltError("Unable to allocate space for MQTT error topics");
  }
  WS_DEBUG_PRINTLN("Subscribing to MQTT topics...");
  subscribeWSTopics();
  subscribeErrorTopics();

  // Connect to Network
  WS_DEBUG_PRINTLN("Running Network FSM...");
  // Run the network fsm
  runNetFSM();
  WS.feedWDT();

  // Register hardware with Wippersnapper
  WS_DEBUG_PRINTLN("Registering hardware with WipperSnapper...")
  if (!registerBoard()) {
    haltError("Unable to register with WipperSnapper.");
  }
  runNetFSM();
  WS.feedWDT();

  // Configure hardware
  WS.pinCfgCompleted = false;
  while (!WS.pinCfgCompleted) {
    WS_DEBUG_PRINTLN(
        "Polling for message containing hardware configuration...");
    WS._mqtt->processPackets(10); // poll
  }
  // Publish that we have completed the configuration workflow
  WS.feedWDT();
  runNetFSM();
  publishPinConfigComplete();
  WS_DEBUG_PRINTLN("Hardware configured successfully!");

  // Register components
  WS_DEBUG_PRINTLN("Registering components...");
#ifdef ARDUINO_ARCH_ESP32
  WS_DEBUG_PRINT("LEDC: ");
  WS._ledc = new WipperSnapper_Component_LEDC();
  WS_DEBUG_PRINTLN("OK!");
#endif

  WS_DEBUG_PRINT("SERVO: ");
  WS._servoComponent = new ws_servo();
  WS_DEBUG_PRINTLN("OK!");

  // goto application
  statusLEDFade(GREEN, 3);
  WS_DEBUG_PRINTLN(
      "Registration and configuration complete!\nRunning application...");
}

/**************************************************************************/
/*!
    @brief    Publishes an ACK to the broker that the device has completed
              its hardware configuration.
*/
/**************************************************************************/
void Wippersnapper::publishPinConfigComplete() {
  // Publish that we've set up the pins and are ready to run
  wippersnapper_signal_v1_SignalResponse msg =
      wippersnapper_signal_v1_SignalResponse_init_zero;
  msg.which_payload =
      wippersnapper_signal_v1_SignalResponse_configuration_complete_tag;
  msg.payload.configuration_complete = true;

  // encode registration request message
  uint8_t _message_buffer[128];
  pb_ostream_t _msg_stream =
      pb_ostream_from_buffer(_message_buffer, sizeof(_message_buffer));

  bool _status =
      pb_encode(&_msg_stream,
                wippersnapper_description_v1_RegistrationComplete_fields, &msg);
  size_t _message_len = _msg_stream.bytes_written;

  // verify message encoded correctly
  if (!_status)
    haltError("Could not encode, resetting...");

  // Publish message
  WS_DEBUG_PRINTLN("Publishing to pin config complete...");
  WS.publish(WS._topic_device_pin_config_complete, _message_buffer,
             _message_len, 1);
}

/**************************************************************************/
/*!
    @brief    Processes incoming commands and handles network connection.
    @returns  Network status, as ws_status_t.
*/
/**************************************************************************/
ws_status_t Wippersnapper::run() {
  // Check networking
  runNetFSM();
  WS.feedWDT();
  pingBroker();

  // Process all incoming packets from Wippersnapper MQTT Broker
  WS._mqtt->processPackets(10);
  WS.feedWDT();

  // Process digital inputs, digitalGPIO module
  WS._digitalGPIO->processDigitalInputs();
  WS.feedWDT();

  // Process analog inputs
  WS._analogIO->processAnalogInputs();
  WS.feedWDT();

  // Process I2C sensor events
  if (WS._isI2CPort0Init)
    WS._i2cPort0->update();
  WS.feedWDT();

  return WS_NET_CONNECTED; // TODO: Make this funcn void!
}