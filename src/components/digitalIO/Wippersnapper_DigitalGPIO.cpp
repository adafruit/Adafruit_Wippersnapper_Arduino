/*!
 * @file Wippersnapper_DigitalGPIO.cpp
 *
 * This file provides an API for interacting with
 * a board's digital GPIO pins.
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

#include "Wippersnapper_DigitalGPIO.h"

/***********************************************************************************/
/*!
    @brief  Initializes DigitalGPIO class.
    @param  totalDigitalInputPins
                Total number of digital gpio input-capable pins to allocate.
*/
/***********************************************************************************/
Wippersnapper_DigitalGPIO::Wippersnapper_DigitalGPIO(
    int32_t totalDigitalInputPins) {
  _totalDigitalInputPins = totalDigitalInputPins;
  _digital_input_pins = new digitalInputPin[_totalDigitalInputPins];
  // turn input sampling off for all digital pins
  for (int i = 0; i < _totalDigitalInputPins; i++) {
    _digital_input_pins[i].pinName = -1;
    _digital_input_pins[i].period = -1;
    _digital_input_pins[i].prvPeriod = 0L;
    _digital_input_pins[i].prvPinVal = 0;
  }
}

/*********************************************************/
/*!
    @brief    Digital GPIO destructor.
*/
/*********************************************************/
Wippersnapper_DigitalGPIO::~Wippersnapper_DigitalGPIO() {
  delete _digital_input_pins;
}


#ifdef ARDUINO_ARCH_RP2040
  void rp2040_dump_gpio_config(void) {
    WS_PRINTER.printf("GPIO | FUNC | DIR | OUT | IN  | PULL-UP | PULL-DOWN | SLEW\n");
    WS_PRINTER.printf("-----+------+-----+-----+-----+---------+-----------+-----\n");

    for (int gpio = 0; gpio < NUM_BANK0_GPIOS; gpio++) {
        WS_PRINTER.printf(" %2d  |  %2d  |  %s  |  %d  |  %d  |    %d    |     %d     |  %d\n",
            gpio,
            gpio_get_function(gpio),
            gpio_get_dir(gpio) ? "OUT" : "IN ",
            gpio_get_out_level(gpio),
            gpio_get(gpio),
            gpio_is_pulled_up(gpio),
            gpio_is_pulled_down(gpio),
            gpio_get_slew_rate(gpio));
    }
  }
#endif

/*******************************************************************************************************************************/
/*!
    @brief  Configures a digital pin to behave as an input or an output.
    @param  direction
            The pin's direction.
    @param  pinName
            The pin's name.
    @param  period
            The pin's period, in seconds.
    @param  pull
            The pin's pull mode.
*/
/*******************************************************************************************************************************/
void Wippersnapper_DigitalGPIO::initDigitalPin(
    wippersnapper_pin_v1_ConfigurePinRequest_Direction direction,
    uint8_t pinName, float period,
    wippersnapper_pin_v1_ConfigurePinRequest_Pull pull) {
  if (direction ==
      wippersnapper_pin_v1_ConfigurePinRequest_Direction_DIRECTION_OUTPUT) {

#ifdef STATUS_LED_PIN
    // deinit status led, use it as a dio component instead
    if (pinName == STATUS_LED_PIN)
      releaseStatusLED();
#endif
    pinMode(pinName, OUTPUT);

    WS_DEBUG_PRINT("Configured digital output pin on D");
    WS_DEBUG_PRINTLN(pinName);

// Initialize LOW
#if defined(ARDUINO_ESP8266_ADAFRUIT_HUZZAH)
    // The Adafruit Feather ESP8266's built-in LED is reverse wired so setting
    // the pin LOW will turn the LED on.
    digitalWrite(STATUS_LED_PIN, !0);
#else
    pinMode(pinName, OUTPUT);
    digitalWrite(pinName, LOW); // initialize LOW
#endif
  } else if (
      direction ==
      wippersnapper_pin_v1_ConfigurePinRequest_Direction_DIRECTION_INPUT) {
    WS_DEBUG_PRINT("Configuring digital input pin on D");
    WS_DEBUG_PRINT(pinName);

    if (pull == wippersnapper_pin_v1_ConfigurePinRequest_Pull_PULL_UP) {
      WS_DEBUG_PRINTLN("with internal pull-up enabled");
      pinMode(pinName, INPUT_PULLUP);
#ifdef INPUT_PULLDOWN
    } else if (pull ==
               wippersnapper_pin_v1_ConfigurePinRequest_Pull_PULL_DOWN) {
      WS_DEBUG_PRINTLN("with internal pull-down enabled");
      pinMode(pinName, INPUT_PULLDOWN);
#endif
    } else {
      pinMode(pinName, INPUT);
      WS_DEBUG_PRINT("\n");
    }

    // Period is in seconds, cast it to long and convert it to milliseconds
    long periodMs = (long)period * 1000;
    WS_DEBUG_PRINT("Interval (ms):");
    WS_DEBUG_PRINTLN(periodMs);

    // get current time
    ulong curTime = millis() - 1;

    // attempt to allocate a pinName within _digital_input_pins[]
    for (int i = 0; i < _totalDigitalInputPins; i++) {
      if (_digital_input_pins[i].period == -1L) {
        _digital_input_pins[i].pinName = pinName;
        _digital_input_pins[i].period = periodMs;
        _digital_input_pins[i].prvPeriod = curTime - periodMs;
        if (pull == wippersnapper_pin_v1_ConfigurePinRequest_Pull_PULL_UP) {
          _digital_input_pins[i].prvPinVal = HIGH;
        } else {
          _digital_input_pins[i].prvPinVal = LOW;
        }
        break;
      }
    }

#ifdef ARDUINO_ARCH_RP2040
    rp2040_dump_gpio_config();
#endif

  } else {
    WS_DEBUG_PRINTLN("ERROR: Invalid digital pin direction!");
  }
}

/********************************************************************************************************************************/
/*!
    @brief    Deinitializes a previously configured digital pin.
    @param    direction
              The pin's direction.
    @param    pinName
              The pin's name.
*/
/********************************************************************************************************************************/
void Wippersnapper_DigitalGPIO::deinitDigitalPin(
    wippersnapper_pin_v1_ConfigurePinRequest_Direction direction,
    uint8_t pinName) {
  WS_DEBUG_PRINT("Deinitializing digital pin ");
  WS_DEBUG_PRINTLN(pinName);

  if (direction ==
      wippersnapper_pin_v1_ConfigurePinRequest_Direction_DIRECTION_INPUT) {
    // de-allocate the pin within digital_input_pins[]
    for (int i = 0; i < _totalDigitalInputPins; i++) {
      if (_digital_input_pins[i].pinName == pinName) {
        _digital_input_pins[i].pinName = -1;
        _digital_input_pins[i].period = -1;
        _digital_input_pins[i].prvPeriod = 0L;
        _digital_input_pins[i].prvPinVal = 0;
        break;
      }
    }
  }
  char cstr[16];
  itoa(pinName, cstr, 10);
  pinMode(pinName, INPUT); // hi-z

#ifdef ARDUINO_ARCH_RP2040
  rp2040_dump_gpio_config();
#endif
  
// if prv. in-use by DIO, release pin back to application
#ifdef STATUS_LED_PIN
  if (pinName == STATUS_LED_PIN)
    initStatusLED();
#endif
}

/********************************************************************/
/*!
    @brief    High-level digitalRead service impl. which performs a
                digitalRead.
    @param    pinName
                The pin's name
    @returns  The pin's value.
*/
/********************************************************************/
int Wippersnapper_DigitalGPIO::digitalReadSvc(int pinName) {
  // Service using arduino `digitalRead`
  int pinVal = digitalRead(pinName);
  // how do i check the digital IO pull up state for one of the
  // _digital_input_pins, because I'm experiencing issues that resemble the
  // situation where the initially set pull up value is no longer in effect
  //  For ESP32
  // #if defined(ESP32)
  // #include "driver/gpio.h"
  // #include "esp_err.h"
  // #include "esp_log.h"
  // #include "esp_system.h"
  // #include "esp32-hal-log.h"
  // #include "esp32-hal-gpio.h"
  //   gpio_pull_mode_t pull_mode;
  // // can't find in idf, but merged issue
  // https://github.com/espressif/esp-idf/issues/12176
  //   esp_err_t result = gpio_get_pull_mode((gpio_num_t)pinName, &pull_mode);
  //   if (result == ESP_OK) {
  //     return (pull_mode == GPIO_PULLUP_ONLY);
  //   }
  // #endif
  return pinVal;
}

/*******************************************************************************/
/*!
    @brief  Writes a value to a pin.
    @param  pinName
                The pin's name.
    @param  pinValue
                The pin's value.
*/
/*******************************************************************************/
void Wippersnapper_DigitalGPIO::digitalWriteSvc(uint8_t pinName, int pinValue) {
  WS_DEBUG_PRINT("Digital Pin Event: Set ");
  WS_DEBUG_PRINT(pinName);
  WS_DEBUG_PRINT(" to ");
  WS_DEBUG_PRINTLN(pinValue);

// Write to the GPIO pin
#if defined(ARDUINO_ESP8266_ADAFRUIT_HUZZAH)
  // The Adafruit Feather ESP8266's built-in LED is reverse wired so setting the
  // pin LOW will turn the LED on.
  if (pinName == 0)
    digitalWrite(pinName, !pinValue);
  else
    digitalWrite(pinName, pinValue);
#else
  digitalWrite(pinName, pinValue);
#endif
}

/**********************************************************/
/*!
    @brief    Iterates thru digital inputs, checks if they
                should send data to the broker.
*/
/**********************************************************/
void Wippersnapper_DigitalGPIO::processDigitalInputs() {
  long curTime = millis();
  // Process digital digital pins
  for (int i = 0; i < _totalDigitalInputPins; i++) {
    if (_digital_input_pins[i].period >
        -1L) { // validate if digital pin is enabled
      // Check if digital pin executes on a time period
      if (curTime - _digital_input_pins[i].prvPeriod >
              _digital_input_pins[i].period &&
          _digital_input_pins[i].period != 0L) {
        WS_DEBUG_PRINT("Executing periodic event on D");
        WS_DEBUG_PRINTLN(_digital_input_pins[i].pinName);
        // read the pin
        int pinVal = digitalReadSvc(_digital_input_pins[i].pinName);

        // Create new signal message
        wippersnapper_signal_v1_CreateSignalRequest _outgoingSignalMsg =
            wippersnapper_signal_v1_CreateSignalRequest_init_zero;

        WS_DEBUG_PRINT("Encoding...")
        // Create and encode a pinEvent message
        if (!WS.encodePinEvent(&_outgoingSignalMsg,
                               _digital_input_pins[i].pinName, pinVal)) {
          WS_DEBUG_PRINTLN("ERROR: Unable to encode pinEvent");
          break;
        }
        WS_DEBUG_PRINTLN("Encoded!")

        // Obtain size and only write out buffer to end
        size_t msgSz;
        pb_get_encoded_size(&msgSz,
                            wippersnapper_signal_v1_CreateSignalRequest_fields,
                            &_outgoingSignalMsg);

        WS_DEBUG_PRINT("Publishing pinEvent...");
        WS.publish(WS._topic_signal_device, WS._buffer_outgoing, msgSz, 1);
        WS_DEBUG_PRINTLN("Published!");

        // reset the digital pin
        _digital_input_pins[i].prvPeriod = curTime;
      } else if (_digital_input_pins[i].period == 0L) {
        // read pin
        int pinVal = digitalReadSvc(_digital_input_pins[i].pinName);
        // only send on-change, but we don't know initial state of feed
        // (prvPinVal at boot)
        if (pinVal != _digital_input_pins[i].prvPinVal) {
          WS_DEBUG_PRINT("Executing state-based event on D");
          WS_DEBUG_PRINTLN(_digital_input_pins[i].pinName);

          // Create new signal message
          wippersnapper_signal_v1_CreateSignalRequest _outgoingSignalMsg =
              wippersnapper_signal_v1_CreateSignalRequest_init_zero;

          WS_DEBUG_PRINT("Encoding pinEvent...");
          // Create and encode a pinEvent message
          if (!WS.encodePinEvent(&_outgoingSignalMsg,
                                 _digital_input_pins[i].pinName, pinVal)) {
            WS_DEBUG_PRINTLN("ERROR: Unable to encode pinEvent");
            break;
          }
          WS_DEBUG_PRINTLN("Encoded!");

          // Obtain size and only write out buffer to end
          size_t msgSz;
          pb_get_encoded_size(
              &msgSz, wippersnapper_signal_v1_CreateSignalRequest_fields,
              &_outgoingSignalMsg);
          WS_DEBUG_PRINT("Publishing pinEvent...");
          WS.publish(WS._topic_signal_device, WS._buffer_outgoing, msgSz, 1);
          WS_DEBUG_PRINTLN("Published!");

          // set the pin value in the digital pin object for comparison on next
          // run
          _digital_input_pins[i].prvPinVal = pinVal;

          // reset the digital pin
          _digital_input_pins[i].prvPeriod = curTime;
          // } else {
          //   WS_DEBUG_PRINT("Dio: No change on ");
          //   WS_DEBUG_PRINT(_digital_input_pins[i].pinName);
          //   WS_DEBUG_PRINTLN(", skipping...");
        }
      }
    }
  }
}