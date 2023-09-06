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
 * Copyright (c) Brent Rubell 2020-2023 for Adafruit Industries.
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

  // allocate analog input pins
  _analog_input_pins = new analogInputPin[_totalAnalogInputPins];

  // TODO: Refactor this to use list-based initialization
  for (int pin = 0; pin < _totalAnalogInputPins; pin++) {
    // turn sampling off
    _analog_input_pins[pin].enabled = false;
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
#if defined(ARDUINO_ARCH_SAMD)
  analogReadResolution(16);
  _nativeResolution = 12;
#elif defined(ARDUINO_ARCH_ESP32)
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

  // Set analog read pull mode
  if (pullMode == wippersnapper_pin_v1_ConfigurePinRequest_Pull_PULL_UP)
    pinMode(pin, INPUT_PULLUP);
  else
    pinMode(pin, INPUT);

  // Period is in seconds, cast it to long and convert it to milliseconds
  long periodMs = (long)period * 1000;

  // TODO: Maybe pull this out into a func. or use map() lookup instead
  // attempt to allocate pin within _analog_input_pins[]
  for (int i = 0; i < _totalAnalogInputPins; i++) {
    if (_analog_input_pins[i].enabled == false) {
      _analog_input_pins[i].pinName = pin;
      _analog_input_pins[i].period = periodMs;
      _analog_input_pins[i].readMode = analogReadMode;
      _analog_input_pins[i].enabled = true;
      break;
    }
  }
  WS_DEBUG_PRINT("Configured Analog Input pin with polling time (ms):");
  WS_DEBUG_PRINTLN(periodMs);
}

/***********************************************************************************/
/*!
    @brief  Disables an analog input pin from sampling
    @param  pin
            The analog input pin to disable.
*/
/***********************************************************************************/
void Wippersnapper_AnalogIO::disableAnalogInPin(int pin) {
  for (int i = 0; i < _totalAnalogInputPins; i++) {
    if (_analog_input_pins[i].pinName == pin) {
      _analog_input_pins[i].enabled = false;
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
    disableAnalogInPin(pin);
  }
  pinMode(pin, INPUT); // hi-z
}

/**********************************************************/
/*!
    @brief    Reads the raw ADC value of an analog pin.
                Value is always scaled to 16-bit.
    @param    pin
              The pin to be read.
    @returns  The pin's ADC value.
*/
/**********************************************************/
uint16_t Wippersnapper_AnalogIO::getPinValue(int pin) {
  // get pin value
  uint16_t value = analogRead(pin);

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
    @param    pin
              The value from a previous ADC reading.
    @returns  The pin's voltage.
*/
/**********************************************************/
float Wippersnapper_AnalogIO::getPinValueVolts(int pin) {
  uint16_t rawValue = getPinValue(pin);
  return rawValue * getAref() / 65536;
}

/******************************************************************/
/*!
    @brief    Encodes an analog input pin event into a
                signal message and publish it to IO.
    @param    pinName
              Specifies the pin's name.
    @param    readMode
              Read mode - raw ADC or voltage.
    @param    pinValRaw
              Raw pin value, used if readmode is raw.
    @param    pinValVolts
              Raw pin value expressed in Volts, used if readmode is
              volts.
    @returns  True if successfully encoded a PinEvent signal
                message, False otherwise.
*/
/******************************************************************/
bool Wippersnapper_AnalogIO::encodePinEvent(
    uint8_t pinName,
    wippersnapper_pin_v1_ConfigurePinRequest_AnalogReadMode readMode,
    uint16_t pinValRaw, float pinValVolts) {
  // Create new signal message
  wippersnapper_signal_v1_CreateSignalRequest outgoingSignalMsg =
      wippersnapper_signal_v1_CreateSignalRequest_init_zero;

  // Fill payload
  outgoingSignalMsg.which_payload =
      wippersnapper_signal_v1_CreateSignalRequest_pin_event_tag;
  sprintf(outgoingSignalMsg.payload.pin_event.pin_name, "A%d", pinName);

  // Fill pinValue based on the analog read mode
  char buffer[100];
  if (readMode ==
      wippersnapper_pin_v1_ConfigurePinRequest_AnalogReadMode_ANALOG_READ_MODE_PIN_VALUE) {
    sprintf(outgoingSignalMsg.payload.pin_event.pin_value, "%u", pinValRaw);
    snprintf(buffer, 100, "[Pin] A%d read: %u\n", pinName, pinValRaw);
  } else {
    sprintf(outgoingSignalMsg.payload.pin_event.pin_value, "%0.3f",
            pinValVolts);
    snprintf(buffer, 100, "[Pin] A%d read: %0.2f\n", pinName, pinValVolts);
  }
// display analog pin read on terminal
#ifdef USE_DISPLAY
  WS._ui_helper->add_text_to_terminal(buffer);
#endif

  // Encode signal message
  pb_ostream_t stream =
      pb_ostream_from_buffer(WS._buffer_outgoing, sizeof(WS._buffer_outgoing));
  if (!pb_encode(&stream, wippersnapper_signal_v1_CreateSignalRequest_fields,
                 &outgoingSignalMsg)) {
    WS_DEBUG_PRINTLN("ERROR: Unable to encode signal message");
    return false;
  }

  // Publish out to IO
  size_t msgSz;
  pb_get_encoded_size(&msgSz,
                      wippersnapper_signal_v1_CreateSignalRequest_fields,
                      &outgoingSignalMsg);
  WS_DEBUG_PRINT("Publishing pinEvent...");
  WS.publish(WS._topic_signal_device, WS._buffer_outgoing, msgSz, 1);
  WS_DEBUG_PRINTLN("Published!");

  return true;
}

/**********************************************************/
/*!
    @brief Checks if pin's period is expired.
    @param currentTime
           The current software timer value.
    @param pin
           The desired analog pin to check
    @returns True if pin's period expired, False otherwise.
*/
/**********************************************************/
bool Wippersnapper_AnalogIO::timerExpired(long currentTime,
                                          analogInputPin pin) {
  if (currentTime - pin.prvPeriod > pin.period && pin.period != 0L)
    return true;
  return false;
}

/**********************************************************/
/*!
    @brief    Iterates thru analog inputs
*/
/**********************************************************/
void Wippersnapper_AnalogIO::update() {
  // TODO: Globally scope these, dont have them here every time
  float pinValVolts = 0.0;
  uint16_t pinValRaw = 0;
  // Process analog input pins
  for (int i = 0; i < _totalAnalogInputPins; i++) {
    // TODO: Can we collapse the conditionals below?
    if (_analog_input_pins[i].enabled == true) {

      // Does the pin execute on-period?
      if ((long)millis() - _analog_input_pins[i].prvPeriod >
              _analog_input_pins[i].period &&
          _analog_input_pins[i].period != 0L) {
        WS_DEBUG_PRINT("Executing periodic event on A");
        WS_DEBUG_PRINTLN(_analog_input_pins[i].pinName);

        // Read from analog pin
        if (_analog_input_pins[i].readMode ==
            wippersnapper_pin_v1_ConfigurePinRequest_AnalogReadMode_ANALOG_READ_MODE_PIN_VOLTAGE) {
          pinValVolts = getPinValueVolts(_analog_input_pins[i].pinName);
        } else if (
            _analog_input_pins[i].readMode ==
            wippersnapper_pin_v1_ConfigurePinRequest_AnalogReadMode_ANALOG_READ_MODE_PIN_VALUE) {
          pinValRaw = getPinValue(_analog_input_pins[i].pinName);
        } else {
          WS_DEBUG_PRINTLN("ERROR: Unable to read pin value, cannot determine "
                           "analog read mode!");
          pinValRaw = 0.0;
        }

        // Publish a new pin event
        encodePinEvent(_analog_input_pins[i].pinName,
                       _analog_input_pins[i].readMode, pinValRaw, pinValVolts);

        // IMPT - reset the digital pin
        _analog_input_pins[i].prvPeriod = millis();
      }
      // Does the pin execute on_change?
      else if (_analog_input_pins[i].period == 0L) {

        // note: on-change requires ADC DEFAULT_HYSTERISIS to check against prv
        // pin value
        uint16_t pinValRaw = getPinValue(_analog_input_pins[i].pinName);

        uint16_t _pinValThreshHi =
            _analog_input_pins[i].prvPinVal +
            (_analog_input_pins[i].prvPinVal * DEFAULT_HYSTERISIS);
        uint16_t _pinValThreshLow =
            _analog_input_pins[i].prvPinVal -
            (_analog_input_pins[i].prvPinVal * DEFAULT_HYSTERISIS);

        if (pinValRaw > _pinValThreshHi || pinValRaw < _pinValThreshLow) {
          // Perform voltage conversion if we need to
          if (_analog_input_pins[i].readMode ==
              wippersnapper_pin_v1_ConfigurePinRequest_AnalogReadMode_ANALOG_READ_MODE_PIN_VOLTAGE) {
            pinValVolts = pinValRaw * getAref() / 65536;
          }

          // Publish pin event to IO
          encodePinEvent(_analog_input_pins[i].pinName,
                         _analog_input_pins[i].readMode, pinValRaw,
                         pinValVolts);

        } else {
          // WS_DEBUG_PRINTLN("ADC has not changed enough, continue...");
          continue;
        }
        // set the pin value in the digital pin object for comparison on next
        // run
        _analog_input_pins[i].prvPinVal = pinValRaw;
      }
    }
  }
}