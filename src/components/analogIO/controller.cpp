/*!
 * @file controller.cpp
 *
 * Controller for the analogio.proto API
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2024 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "controller.h"

/***********************************************************************/
/*!
    @brief  AnalogIO controller constructor
*/
/***********************************************************************/
AnalogIOController::AnalogIOController() {
  _analogio_hardware = new AnalogIOHardware();
  _analogio_model = new AnalogIOModel();
  _analogio_hardware->SetResolution(16); // Default to 16-bit resolution
  SetRefVoltage(3.3);                    // Default to 3.3V
}

/***********************************************************************/
/*!
    @brief  AnalogIO controller destructor
*/
/***********************************************************************/
AnalogIOController::~AnalogIOController() {
  delete _analogio_hardware;
  delete _analogio_model;
}

/***********************************************************************/
/*!
    @brief  Set the reference voltage for the analog pins
    @param  voltage
            The reference voltage.
*/
/***********************************************************************/
void AnalogIOController::SetRefVoltage(float voltage) {
  // To set the reference voltage, we call into the hardware
  _analogio_hardware->SetReferenceVoltage(voltage);
}

/***********************************************************************/
/*!
    @brief  Set the total number of analog pins present on the hardware.
    @param  total_pins
            The hardware's total number of analog pins.
*/
/***********************************************************************/
void AnalogIOController::SetTotalAnalogPins(uint8_t total_pins) {
  _total_analogio_pins = total_pins;
}

/***********************************************************************/
/*!
    @brief  Handles an AnalogIOAdd message from the broker and adds a
            new analog pin to the controller.
    @param  stream
            The nanopb input stream.
    @return True if the pin was successfully added, False otherwise.
*/
/***********************************************************************/
bool AnalogIOController::Handle_AnalogIOAdd(pb_istream_t *stream) {
  // Attempt to decode the incoming message into an AnalogIOAdd object
  if (!_analogio_model->DecodeAnalogIOAdd(stream)) {
    WS_DEBUG_PRINTLN("[analogio] ERROR: Unable to decode Add message");
    return false;
  }

  // Get the pin name
  uint8_t pin_name = atoi(_analogio_model->GetAnalogIOAddMsg()->pin_name + 1);

  // Create a new analogioPin object
  // TODO: Replicate this within the digitalio controller, much cleaner way to
  // assign!
  analogioPin new_pin = {
      .name = pin_name,
      .period = long(_analogio_model->GetAnalogIOAddMsg()->period) * 1000,
      .prv_period = 0,
      .read_mode = _analogio_model->GetAnalogIOAddMsg()->read_mode};

  // Initialize the pin
  _analogio_hardware->InitPin(pin_name);

  // Print out the pin's details
  WS_DEBUG_PRINTLN("[analogio] Added new pin:");
  WS_DEBUG_PRINT("Pin Name: ");
  WS_DEBUG_PRINTLN(new_pin.name);
  WS_DEBUG_PRINT("Period: ");
  WS_DEBUG_PRINTLN(new_pin.period);
  WS_DEBUG_PRINT("Read Mode: ");
  WS_DEBUG_PRINTLN(new_pin.read_mode);

  // Add the new pin to the vector
  _analogio_pins.push_back(new_pin);

  return true;
}

/***************************************************************************/
/*!
    @brief  Handles an AnalogIORemove message from the broker and removes
            the requested analog pin from the controller.
    @param  stream
            The nanopb input stream.
    @return True if the pin was successfully removed, False otherwise.
*/
/***************************************************************************/
bool AnalogIOController::Handle_AnalogIORemove(pb_istream_t *stream) {
  // Attempt to decode the incoming message into an AnalogIORemove object
  if (!_analogio_model->DecodeAnalogIORemove(stream)) {
    WS_DEBUG_PRINTLN("ERROR: Unable to decode AnalogIORemove message");
    return false;
  }

  // Get the pin name
  int pin_name = atoi(_analogio_model->GetAnalogIORemoveMsg()->pin_name + 1);

  // Remove the pin from the hardware
  _analogio_hardware->DeinitPin(pin_name);

  // Remove the pin from the vector
  // TODO: Refactor this out? TODO: Make this better??
  for (int i = 0; i < _analogio_pins.size(); i++) {
    if (_analogio_pins[i].name == pin_name) {
      _analogio_pins.erase(_analogio_pins.begin() + i);
      break;
    }
  }
  return true;
}

/***************************************************************************/
/*!
    @brief  Checks if a pin's periodic timer has expired.
    @param  pin
            The requested pin to check.
    @param  cur_time
            The current time (called from millis()).
    @return True if the pin's period has expired, False otherwise.
*/
/***************************************************************************/
bool AnalogIOController::IsPinTimerExpired(analogioPin *pin, ulong cur_time) {
  return cur_time - pin->prv_period > pin->period;
}

/***************************************************************************/
/*!
    @brief  Encodes and publishes an AnalogIOEvent message to the broker.
    @param  pin
            The pin to encode and publish.
    @param  value
            The pin's value.
    @param  read_type
            The type of read to perform on the pin.
    @return True if the message was successfully encoded and published.
*/
/***************************************************************************/
bool AnalogIOController::EncodePublishPinEvent(
    uint8_t pin, float value, wippersnapper_sensor_SensorType read_type) {
  char c_pin_name[12];
  sprintf(c_pin_name, "A%d", pin);

  if (read_type == wippersnapper_sensor_SensorType_SENSOR_TYPE_RAW) {
    if (!_analogio_model->EncodeAnalogIOEventRaw(c_pin_name, value)) {
      WS_DEBUG_PRINTLN("ERROR: Unable to encode AnalogIO raw adc message!");
      return false;
    }
  } else if (read_type == wippersnapper_sensor_SensorType_SENSOR_TYPE_VOLTAGE) {
    if (!_analogio_model->EncodeAnalogIOEventVoltage(c_pin_name, value)) {
      WS_DEBUG_PRINTLN("ERROR: Unable to encode AnalogIO voltage message!");
      return false;
    }
  } else {
    WS_DEBUG_PRINTLN("ERROR: Invalid read type for AnalogIOEvent message!");
    return false;
  }

  // Publish the AnalogIO message to the broker
  if (!WsV2._sdCardV2->mode_offline) {
    WS_DEBUG_PRINTLN("Publishing AnalogIOEvent message to broker...");
    if (!WsV2.PublishSignal(
            wippersnapper_signal_DeviceToBroker_analogio_event_tag,
            _analogio_model->GetAnalogIOEvent())) {
      WS_DEBUG_PRINTLN(
          "ERROR: Unable to publish analogio voltage event message, "
          "moving onto the next pin!");
      return false;
    }
    WS_DEBUG_PRINTLN("Published AnalogIOEvent message to broker!")
  } else {
    // Print event data
    WS_DEBUG_PRINTLN("AnalogIOEvent message:");
    WS_DEBUG_PRINT("Pin Name: ");
    WS_DEBUG_PRINTLN(c_pin_name);
    WS_DEBUG_PRINT("Value: ");
    WS_DEBUG_PRINTLN(value);
    WS_DEBUG_PRINT("Read Type: ");
    WS_DEBUG_PRINTLN(read_type);
    WS_DEBUG_PRINTLN("[analogio] Offline analogIOEvent message not published!");
    // TODO: Log out this data by calling a logging function in sdcard class
  }

  return true;
}

/***************************************************************************/
/*!
    @brief  Encodes and publishes an AnalogIOEvent message to the broker.
    @param  pin
            The requested pin.
    @param  value
            The pin's value.
    @return True if the message was successfully encoded and published,
            False othewise.
*/
/***************************************************************************/
bool AnalogIOController::EncodePublishPinValue(uint8_t pin, uint16_t value) {

  if (WsV2._sdCardV2->mode_offline) {
    return WsV2._sdCardV2->LogGPIOSensorEventToSD(
        pin, value, wippersnapper_sensor_SensorType_SENSOR_TYPE_RAW);
  } else {
    return EncodePublishPinEvent(
        pin, (float)value, wippersnapper_sensor_SensorType_SENSOR_TYPE_RAW);
  }
}

/***************************************************************************/
/*!
    @brief  Encodes and publishes an AnalogIOEvent message to the broker.
    @param  pin
            The requested pin.
    @param  value
            The pin's value, as a voltage.
    @return True if the message was successfully encoded and published,
            False othewise.
*/
/***************************************************************************/
bool AnalogIOController::EncodePublishPinVoltage(uint8_t pin, float value) {
  if (WsV2._sdCardV2->mode_offline) {
    return WsV2._sdCardV2->LogGPIOSensorEventToSD(
        pin, value, wippersnapper_sensor_SensorType_SENSOR_TYPE_VOLTAGE);
  } else {
    return EncodePublishPinEvent(
        pin, value, wippersnapper_sensor_SensorType_SENSOR_TYPE_VOLTAGE);
  }
}

/***************************************************************************/
/*!
    @brief  Update/polling loop for the AnalogIO controller.
*/
/***************************************************************************/
void AnalogIOController::update() {
  // Bail-out if the vector is empty
  if (_analogio_pins.empty())
    return;

  // Process analog input pins
  for (int i = 0; i < _analogio_pins.size(); i++) {
    // Create a pin object for this iteration
    analogioPin &pin = _analogio_pins[i];
    // Go to the next pin if the period hasn't expired yet
    ulong cur_time = millis();
    if (!IsPinTimerExpired(&pin, cur_time))
      continue;

    // Pins timer has expired, lets read the pin
    if (pin.read_mode == wippersnapper_sensor_SensorType_SENSOR_TYPE_RAW) {
      // Read the pin's raw value
      uint16_t value = _analogio_hardware->GetPinValue(pin.name);
      // Encode and publish it to the broker
      EncodePublishPinValue(pin.name, value);
      pin.prv_period = cur_time; // Reset the pin's period
    } else if (pin.read_mode ==
               wippersnapper_sensor_SensorType_SENSOR_TYPE_VOLTAGE) {
      // Convert the raw value into voltage
      float pin_value = _analogio_hardware->GetPinVoltage(pin.name);
      // Encode and publish the voltage value to the broker
      EncodePublishPinVoltage(pin.name, pin_value);
      pin.prv_period = cur_time; // Reset the pin's period
    } else {
      WS_DEBUG_PRINTLN("[analogio] ERROR: Invalid read mode for analog pin!");
    }
  }
}