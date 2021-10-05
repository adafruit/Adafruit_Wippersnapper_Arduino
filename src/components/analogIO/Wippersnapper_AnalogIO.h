/*!
 * @file Wippersnapper_AnalogIO.h
 *
 * This file provides digital GPIO control and access.
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

#ifndef WIPPERSNAPPER_ANALOGIO_H
#define WIPPERSNAPPER_ANALOGIO_H

#include "Wippersnapper.h"

/** Data about an analog input pin */
struct analogInputPin {
  int pinName; ///< Pin name
  wippersnapper_pin_v1_ConfigurePinRequest_AnalogReadMode
      readMode;    ///< Which type of read to perform
  long period;     ///< Pin timer interval, in millis, -1 if disabled.
  long prvPeriod;  ///< When Pin's timer was previously serviced, in millis
  float prvPinVal; ///< Previous pin value
};

// forward decl.
class Wippersnapper;

/**************************************************************************/
/*!
    @brief  Class that provides an interface for reading and controlling
            analog pins. Stores information about analog input pins.
*/
/**************************************************************************/
class Wippersnapper_AnalogIO {
public:
  Wippersnapper_AnalogIO(int32_t totalAnalogInputPins, float aRef);
  ~Wippersnapper_AnalogIO();

  void setAref(float refVoltage);
  float getAref();

  void initAnalogOutputPin(int pin);
  void initAnalogInputPin(
      int pin, float period,
      wippersnapper_pin_v1_ConfigurePinRequest_Pull pullMode,
      wippersnapper_pin_v1_ConfigurePinRequest_AnalogReadMode analogReadMode);
  void
  deinitAnalogPin(wippersnapper_pin_v1_ConfigurePinRequest_Direction direction,
                  int pin);
  void deinitAnalogInputPinObj(int pin);

  uint16_t readAnalogPinRaw(int pin);
  float getAnalogPinVoltage(uint16_t rawValue);

  void setADCResolution(int resolution);
  int getADCresolution();
  int getNativeResolution();

  void processAnalogInputs();

  bool
  encodePinEvent(wippersnapper_signal_v1_CreateSignalRequest *outgoingSignalMsg,
                 uint8_t pinName, float pinVal);
  bool
  encodePinEvent(wippersnapper_signal_v1_CreateSignalRequest *outgoingSignalMsg,
                 uint8_t pinName, uint16_t pinVal);

  analogInputPin *_analog_input_pins; /*!< Array of analog pin objects */
private:
  float _aRef;           /*!< Hardware's reported voltage reference */
  int _adcResolution;    /*!< Resolution returned by the analogRead() funcn. */
  int _nativeResolution; /*!< Hardware's native ADC resolution. */
  bool scaleAnalogRead = false; /*!< True if we need to manually scale the value
                                   returned by analogRead(). */
  int32_t _totalAnalogInputPins; /*!< Total number of analog input pins */

  float _hysterisis;         /*!< Hysterisis factor. */
  uint16_t _pinValThreshLow; /*!< Calculated low threshold. */
  uint16_t _pinValThreshHi;  /*!< Calculated high threshold. */

  uint16_t _pinValue; /*!< Pin's raw value from analogRead */
  float _pinVoltage;  /*!< Pin's calculated voltage, in volts. */

  wippersnapper_signal_v1_CreateSignalRequest
      _outgoingSignalMsg; /*!< Signal message to send to broker on pin event. */
};
extern Wippersnapper WS; /*!< Wippersnapper variable. */

#endif // WIPPERSNAPPER_DIGITALGPIO_H