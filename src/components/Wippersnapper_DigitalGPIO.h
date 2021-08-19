/*!
 * @file Wippersnapper_DigitalGPIO.h
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

#ifndef WIPPERSNAPPER_DIGITALGPIO_H
#define WIPPERSNAPPER_DIGITALGPIO_H

#include "Wippersnapper.h"

/** Holds data about a digital input pin */
struct digitalInputPin {
  uint8_t pinName; ///< Pin name
  long period;     ///< Timer interval, in millis, -1 if disabled.
  long prvPeriod;  ///< When timer was previously serviced, in millis
  int prvPinVal;   ///< Previous pin value
};

// forward decl.
class Wippersnapper;

/**************************************************************************/
/*!
    @brief  Class that provides functions for reading and interacting with
            digital inputs and outputs.
*/
/**************************************************************************/
class Wippersnapper_DigitalGPIO {
public:
  Wippersnapper_DigitalGPIO(int32_t totalDigitalInputPins);
  ~Wippersnapper_DigitalGPIO();

  void
  initDigitalPin(wippersnapper_pin_v1_ConfigurePinRequest_Direction direction,
                 uint8_t pinName, float period,
                 wippersnapper_pin_v1_ConfigurePinRequest_Pull pull);
  void
  deinitDigitalPin(wippersnapper_pin_v1_ConfigurePinRequest_Direction direction,
                   uint8_t pinName);

  int digitalReadSvc(int pinName);
  void digitalWriteSvc(uint8_t pinName, int pinValue);
  void processDigitalInputs();

  digitalInputPin *_digital_input_pins; /*!< Array of gpio pin objects */
private:
  int32_t
      _totalDigitalInputPins; /*!< Total number of digital-input capable pins */
};
extern Wippersnapper WS;

#endif // WIPPERSNAPPER_DIGITALGPIO_H