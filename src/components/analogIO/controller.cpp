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

AnalogIOController::AnalogIOController() {
  _analogio_hardware = new AnalogIOHardware();
  _analogio_model = new AnalogIOModel();
  _analogio_hardware->SetResolution(16); // Default to 16-bit resolution
  SetRefVoltage(3.3);                    // Default to 3.3V
}

AnalogIOController::~AnalogIOController() {}

void AnalogIOController::SetRefVoltage(float voltage) {
  // To set the reference voltage, we call into the hardware
  _analogio_hardware->SetReferenceVoltage(voltage);
}

void AnalogIOController::SetTotalAnalogPins(uint8_t total_pins) {
  _total_analogio_pins = total_pins;
}

bool AnalogIOController::Handle_AnalogIOAdd(pb_istream_t *stream) {
  // Attempt to decode the incoming message into an AnalogIOAdd object
  if (!_analogio_model->DecodeAnalogIOAdd(stream)) {
    WS_DEBUG_PRINTLN("ERROR: Unable to decode AnalogIOAdd message");
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

  // Add the new pin to the vector
  _analogio_pins.push_back(new_pin);

  return true;
}

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
  // TODO: Refactor this out?
  for (int i = 0; i < _analogio_pins.size(); i++) {
    if (_analogio_pins[i].name == pin_name) {
      _analogio_pins.erase(_analogio_pins.begin() + i);
      break;
    }
  }
  return true;
}

bool AnalogIOController::IsPinTimerExpired(analogioPin *pin, ulong cur_time) {
  return cur_time - pin->prv_period > pin->period;
}

bool AnalogIOController::EncodePublishPinEvent(
    uint8_t pin, float value, wippersnapper_sensor_SensorType read_type) {
  char c_pin_name[12];
  sprintf(c_pin_name, "D%d", pin);

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

  // Publish the DigitalIOEvent message to the broker
  if (!WsV2.PublishSignal(
          wippersnapper_signal_DeviceToBroker_digitalio_event_tag,
          _analogio_model->GetAnalogIOEvent())) {
    WS_DEBUG_PRINTLN("ERROR: Unable to publish analogio voltage event message, "
                     "moving onto the next pin!");
    return false;
  }
  WS_DEBUG_PRINTLN("Published AnalogIOEvent message to broker!")

  return true;
}

bool AnalogIOController::EncodePublishPinValue(uint8_t pin, uint16_t value) {
  return EncodePublishPinEvent(pin, (float)value,
                               wippersnapper_sensor_SensorType_SENSOR_TYPE_RAW);
}

bool AnalogIOController::EncodePublishPinVoltage(uint8_t pin, float value) {
  return EncodePublishPinEvent(
      pin, value, wippersnapper_sensor_SensorType_SENSOR_TYPE_VOLTAGE);
}

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
    // Read the pin's raw value
    uint16_t value = _analogio_hardware->GetPinValue(pin.name);
    if (pin.read_mode == wippersnapper_sensor_SensorType_SENSOR_TYPE_RAW) {
      // Since we already read the raw value, encode and publish it to the
      // broker
      // TODO
    } else if (pin.read_mode ==
               wippersnapper_sensor_SensorType_SENSOR_TYPE_VOLTAGE) {
      // Convert the raw value into voltage
      float pin_value = _analogio_hardware->CalculatePinVoltage(value);
      // Encode and publish the voltage value to the broker
    }
  }
}