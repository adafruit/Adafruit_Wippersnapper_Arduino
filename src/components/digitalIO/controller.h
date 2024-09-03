/*!
 * @file controller.h
 *
 * Controller for the digitalio API
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
#ifndef WS_DIGITALIO_CONTROLLER_H
#define WS_DIGITALIO_CONTROLLER_H
#include "Wippersnapper_V2.h"

class DigitalOutputPin {
public:
  DigitalOutputPin(uint8_t pin_name);
  uint8_t pin_name;
  bool pin_value;
};

class DigitalInputPin {
public:
  DigitalInputPin(uint8_t pin_name,
                  wippersnapper_digitalio_DigitalIODirection pin_direction,
                  wippersnapper_digitalio_DigitalIOSampleMode sample_mode);
  uint8_t pin_name;
  wippersnapper_digitalio_DigitalIODirection pin_direction;
  wippersnapper_digitalio_DigitalIOSampleMode sample_mode;
  bool pin_value;
  bool prv_pin_value;
  long pin_period;
  long prv_pin_period;
};

class DigitalIOModel {
public:
  DigitalIOModel();
  ~DigitalIOModel();
  bool AddDigitalPin();

private:
  std::vector<DigitalOutputPin> _digital_output_pins;
  std::vector<DigitalInputPin> _digital_input_pins;
};
// TODO: V2 required for WS object
// extern Wippersnapper WS;
#endif // WS_CHECKIN_H