/*!
 * @file Wippersnapper_AnalogIO.cpp
 *
 * This file provides an API for interacting with
 * a board's analog IO pins.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2020-2021 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "Wippersnapper_AnalogIO.h"

/***********************************************************************************/
/*!
    @brief  Initializes Analog IO class.
    @param  totalAnalogInputPins
                Total number of analog input pins to allocate.
    @param  aRef
                ADC's voltage reference value, in volts.
*/
/***********************************************************************************/
Wippersnapper_AnalogIO::Wippersnapper_AnalogIO(int32_t totalAnalogInputPins,
                                               float aRef) {
  _totalAnalogInputPins = totalAnalogInputPins;

  // Set aref
  setAref(aRef);

  // Set ADC resolution, default to 16-bit
  setADCResolution(16);

  // Default hysterisis of 2%
  _hysterisis = 0.02;

  // allocate analog input pins
  _analog_input_pins = new analogInputPin[_totalAnalogInputPins];
  for (int pin = 0; pin < _totalAnalogInputPins; pin++) {
    // turn sampling off
    _analog_input_pins[pin].period = -1;
  }
}

/***********************************************************************************/
/*!
    @brief  Destructor for Analog IO class.
*/
/***********************************************************************************/
Wippersnapper_AnalogIO::~Wippersnapper_AnalogIO() {
  _aRef = 0.0;
  _totalAnalogInputPins = 0;
  _hysterisis = 0;
  delete _analog_input_pins;
}

/***********************************************************************************/
/*!
    @brief  Sets the device's reference voltage.
    @param  refVoltage
            The voltage reference to use during conversions.
*/
/***********************************************************************************/
void Wippersnapper_AnalogIO::setAref(float refVoltage) { _aRef = refVoltage; }

/***********************************************************************************/
/*!
    @brief  Returns the device's reference voltage.
    @returns Analog reference voltage, in volts.
*/
/***********************************************************************************/
float Wippersnapper_AnalogIO::getAref() { return _aRef; }

/***********************************************************************************/
/*!
    @brief  Sets the device's ADC resolution, either natively via calling
   Arduino API's analogReadResolution() or via scaling.
    @param  resolution
            The desired analog resolution, in bits.
*/
/***********************************************************************************/
void Wippersnapper_AnalogIO::setADCResolution(int resolution) {
// set the resolution natively in the BSP
#ifdef ARDUINO_ARCH_SAMD
  analogReadResolution(16);
  _nativeResolution = 12;
#endif

#ifdef ARDUINO_ARCH_ESP32
  scaleAnalogRead = true;
  _nativeResolution = 13;
#endif

  _adcResolution = resolution;
}

/***********************************************************************************/
/*!
    @brief    Gets the scaled ADC resolution.
    @returns  resolution
                The scaled analog resolution, in bits.
*/
/***********************************************************************************/
int Wippersnapper_AnalogIO::getADCresolution() { return _adcResolution; }

/***********************************************************************************/
/*!
    @brief    Gets the device's native ADC resolution.
    @returns  resolution
                The native analog resolution, in bits.
*/
/***********************************************************************************/
int Wippersnapper_AnalogIO::getNativeResolution() { return _nativeResolution; }

/***********************************************************************************/
/*!
    @brief  Initializes an analog output pin.
    @param  pin
                The analog pin to read from.
*/
/***********************************************************************************/
void Wippersnapper_AnalogIO::initAnalogOutputPin(int pin) {
  WS_DEBUG_PRINT("ERROR: Analog output on pin ");
  WS_DEBUG_PRINT(pin);
  WS_DEBUG_PRINTLN("not implemented yet.");
}

/***********************************************************************************/
/*!
    @brief  Initializes an analog input pin
    @param  pin
              The analog pin to read from.
    @param  period
              Time between measurements, in seconds.
   @param  pullMode
            The pin's pull value.
    @param analogReadMode
            Defines if pin will read and return an ADC value or a voltage value.
*/
/***********************************************************************************/
void Wippersnapper_AnalogIO::initAnalogInputPin(
    int pin, float period,
    wippersnapper_pin_v1_ConfigurePinRequest_Pull pullMode,
    wippersnapper_pin_v1_ConfigurePinRequest_AnalogReadMode analogReadMode) {
  WS_DEBUG_PRINT("Initialized analog input pin A");
  WS_DEBUG_PRINTLN(pin);
  if (pullMode == wippersnapper_pin_v1_ConfigurePinRequest_Pull_PULL_UP) {
    pinMode(pin, INPUT_PULLUP); // set analog input pull-up
  } else {
    pinMode(pin, INPUT); // set analog input
  }

  // Period is in seconds, cast it to long and convert it to milliseconds
  long periodMs = (long)period * 1000;
  WS_DEBUG_PRINT("Interval (ms):");
  WS_DEBUG_PRINTLN(periodMs);

  // attempt to allocate pin within _analog_input_pins[]
  for (int i = 0; i < _totalAnalogInputPins; i++) {
    if (_analog_input_pins[i].period == -1L) {
      _analog_input_pins[i].pinName = pin;
      _analog_input_pins[i].period = periodMs;
      _analog_input_pins[i].readMode = analogReadMode;
      break;
    }
  }
}

/***********************************************************************************/
/*!
    @brief  Deinitializes an analog input pin.
    @param  pin
                The analog input pin to deinitialize.
*/
/***********************************************************************************/
void Wippersnapper_AnalogIO::deinitAnalogInputPinObj(int pin) {
  // de-allocate the pin within digital_input_pins[]
  for (int i = 0; i < _totalAnalogInputPins; i++) {
    if (_analog_input_pins[i].pinName == pin) {
      _analog_input_pins[i].pinName = 0;
      _analog_input_pins[i].period = -1;
      _analog_input_pins[i].prvPinVal = 0.0;
      _analog_input_pins[i].prvPeriod = 0L;
      break;
    }
  }
}

/***********************************************************************************/
/*!
    @brief  Deinitializes an analog pin.
    @param  direction
                The analog pin's direction.
    @param  pin
                The analog pin to deinitialize.

*/
/***********************************************************************************/
void Wippersnapper_AnalogIO::deinitAnalogPin(
    wippersnapper_pin_v1_ConfigurePinRequest_Direction direction, int pin) {
  WS_DEBUG_PRINT("Deinitializing analog pin A");
  WS_DEBUG_PRINTLN(pin);
  if (direction ==
      wippersnapper_pin_v1_ConfigurePinRequest_Direction_DIRECTION_INPUT) {
    WS_DEBUG_PRINTLN("Deinitialized analog input pin obj.");
    deinitAnalogInputPinObj(pin);
  }
  pinMode(pin, INPUT); // hi-z
}

/**********************************************************/
/*!
    @brief    Reads the value of an analog pin.
                Value is always scaled to 16-bit.
    @param    pin
              The pin to be read.
    @returns  The pin's ADC value.
*/
/**********************************************************/
uint16_t Wippersnapper_AnalogIO::readAnalogPinRaw(int pin) {
  uint16_t value;
  value = analogRead(pin);

  // scale by the ADC resolution manually if not implemented by BSP
  if (scaleAnalogRead) {
    if (getADCresolution() > getNativeResolution()) {
      value = value << (getADCresolution() - getNativeResolution());
    } else {
      value = value >> (getNativeResolution() - getADCresolution());
    }
  }

  return value;
}

/**********************************************************/
/*!
    @brief    Calculates analog pin's voltage provided
               a 16-bit ADC value.
    @param    rawValue
              The value from a previous ADC reading.
    @returns  The pin's voltage.
*/
/**********************************************************/
float Wippersnapper_AnalogIO::getAnalogPinVoltage(uint16_t rawValue) {
  float pinVoltage;
  pinVoltage = rawValue * getAref() / 65536;
  return pinVoltage;
}

/******************************************************************/
/*!
    @brief    Encodes an analog input pin event into a
                signal message.
    @param    outgoingSignalMsg
              Pointer to an empty CreateSignalRequest
              message.
    @param    pinName
              Specifies the pin's name.
    @param    pinVal
              Pin value.
    @returns  True if successfully encoded a PinEvent signal
                message, False otherwise.
*/
/******************************************************************/
bool Wippersnapper_AnalogIO::encodePinEvent(
    wippersnapper_signal_v1_CreateSignalRequest *outgoingSignalMsg,
    uint8_t pinName, uint16_t pinVal) {
  bool is_success = true;
  WS_DEBUG_PRINT("Encoding signal message...");
  // fill the pin_event message
  outgoingSignalMsg->which_payload =
      wippersnapper_signal_v1_CreateSignalRequest_pin_event_tag;
  sprintf(outgoingSignalMsg->payload.pin_event.pin_name, "A%d", pinName);
  sprintf(outgoingSignalMsg->payload.pin_event.pin_value, "%u", pinVal);

  // Encode signal message
  pb_ostream_t stream =
      pb_ostream_from_buffer(WS._buffer_outgoing, sizeof(WS._buffer_outgoing));
  if (!pb_encode(&stream, wippersnapper_signal_v1_CreateSignalRequest_fields,
                 outgoingSignalMsg)) {
    WS_DEBUG_PRINTLN("ERROR: Unable to encode signal message");
    is_success = false;
  }

  WS_DEBUG_PRINTLN("Encoded!");
  return is_success;
}

/************************************************************/
/*!
    @brief    Encodes an analog input pin event into a
                signal message.
    @param    outgoingSignalMsg
                Pointer to an empty CreateSignalRequest
                message.
    @param    pinName
                Specifies the pin's name.
    @param    pinVal
                Pin voltage, in volts.
    @returns  True if encoded successfully, False otherwise.
*/
/************************************************************/
bool Wippersnapper_AnalogIO::encodePinEvent(
    wippersnapper_signal_v1_CreateSignalRequest *outgoingSignalMsg,
    uint8_t pinName, float pinVal) {
  bool is_success = true;

  // fill the pin_event message
  outgoingSignalMsg->which_payload =
      wippersnapper_signal_v1_CreateSignalRequest_pin_event_tag;
  sprintf(outgoingSignalMsg->payload.pin_event.pin_name, "A%d", pinName);
  sprintf(outgoingSignalMsg->payload.pin_event.pin_value, "%0.3f", pinVal);

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

/**********************************************************/
/*!
    @brief    Iterates thru analog inputs
*/
/**********************************************************/
void Wippersnapper_AnalogIO::processAnalogInputs() {
  long _curTime = millis();
  // Process analog input pins
  for (int i = 0; i < _totalAnalogInputPins; i++) {
    if (_analog_input_pins[i].period >
        -1L) { // validate if pin is enabled for sampling
      // pin executes on-period
      if (_curTime - _analog_input_pins[i].prvPeriod >
              _analog_input_pins[i].period &&
          _analog_input_pins[i].period != 0L) {
        WS_DEBUG_PRINT("Executing periodic event on A");
        WS_DEBUG_PRINTLN(_analog_input_pins[i].pinName);

        // init outgoing signal msg
        _outgoingSignalMsg =
            wippersnapper_signal_v1_CreateSignalRequest_init_zero;

        // Perform an analog read
        _pinValue = readAnalogPinRaw(_analog_input_pins[i].pinName);
        if (_analog_input_pins[i].readMode ==
            wippersnapper_pin_v1_ConfigurePinRequest_AnalogReadMode_ANALOG_READ_MODE_PIN_VOLTAGE) {
          // convert value to voltage
          _pinVoltage = getAnalogPinVoltage(_pinValue);
          // Attempt to encode pin event
          if (!encodePinEvent(&_outgoingSignalMsg,
                              _analog_input_pins[i].pinName, _pinVoltage)) {
            WS_DEBUG_PRINTLN(
                "ERROR: Unable to encode pinevent (analog input, voltage");
          }
        } else { // raw value
          // Attempt to encode pin event msg.
          if (!encodePinEvent(&_outgoingSignalMsg,
                              _analog_input_pins[i].pinName, _pinValue)) {
            WS_DEBUG_PRINTLN(
                "ERROR: Unable to encode pinevent (analog input, value");
          }
        }

        // Obtain size and only write out buffer to end
        size_t msgSz;
        pb_get_encoded_size(&msgSz,
                            wippersnapper_signal_v1_CreateSignalRequest_fields,
                            &_outgoingSignalMsg);
        WS_DEBUG_PRINT("Publishing pinEvent...");
        WS.publish(WS._topic_signal_device, WS._buffer_outgoing, msgSz, 1);
        WS_DEBUG_PRINTLN("Published!");

        // reset the digital pin
        _analog_input_pins[i].prvPeriod = _curTime;
      }
      // pin sample on-change
      else if (_analog_input_pins[i].period == 0L) {
        // Perform an analog read
        _pinValue = readAnalogPinRaw(_analog_input_pins[i].pinName);
        // calculate bounds
        _pinValThreshHi = _analog_input_pins[i].prvPinVal +
                          (_analog_input_pins[i].prvPinVal * _hysterisis);
        _pinValThreshLow = _analog_input_pins[i].prvPinVal -
                           (_analog_input_pins[i].prvPinVal * _hysterisis);

        if (_pinValue > _pinValThreshHi || _pinValue < _pinValThreshLow) {
          WS_DEBUG_PRINT("Executing state-based event on A");
          WS_DEBUG_PRINTLN(_analog_input_pins[i].pinName);

          // init outgoing signal msg
          _outgoingSignalMsg =
              wippersnapper_signal_v1_CreateSignalRequest_init_zero;

          if (_analog_input_pins[i].readMode ==
              wippersnapper_pin_v1_ConfigurePinRequest_AnalogReadMode_ANALOG_READ_MODE_PIN_VOLTAGE) {
            // convert value to voltage
            _pinVoltage = getAnalogPinVoltage(_pinValue);
            // Attempt to encode pin event
            if (!encodePinEvent(&_outgoingSignalMsg,
                                _analog_input_pins[i].pinName, _pinVoltage)) {
              WS_DEBUG_PRINTLN(
                  "ERROR: Unable to encode pinevent (analog input, voltage");
            }
          } else { // raw value
            // Attempt to encode pin event msg.
            if (!encodePinEvent(&_outgoingSignalMsg,
                                _analog_input_pins[i].pinName, _pinValue)) {
              WS_DEBUG_PRINTLN(
                  "ERROR: Unable to encode pinevent (analog input, value");
            }
          }

          // Obtain msg size and only write out buffer to end
          size_t msgSz;
          pb_get_encoded_size(
              &msgSz, wippersnapper_signal_v1_CreateSignalRequest_fields,
              &_outgoingSignalMsg);
          // Publish
          WS_DEBUG_PRINT("Publishing pinEvent...");
          WS.publish(WS._topic_signal_device, WS._buffer_outgoing, msgSz, 1);
          WS_DEBUG_PRINTLN("Published!");

          // set the pin value in the digital pin object for comparison on next
          // run
          _analog_input_pins[i].prvPinVal = _pinValue;

          // reset the digital pin
          _analog_input_pins[i].prvPeriod = _curTime;
        }
      }
    }
  }
}