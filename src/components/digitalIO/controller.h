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

class Wippersnapper_V2;

struct DigitalOutputPin {
  uint8_t pin_name;
  bool pin_value;
};

struct DigitalInputPin {
  uint8_t pin_name;
  wippersnapper_digitalio_DigitalIODirection pin_direction;
  wippersnapper_digitalio_DigitalIOSampleMode sample_mode;
  bool pin_value;
  bool prv_pin_value;
  long pin_period;
  long prv_pin_period;
};

class DigitalIOController {
public:
  DigitalIOController();
  ~DigitalIOController();
  void SetMaxDigitalPins(uint8_t max_digital_pins);
  bool AddDigitalPin(pb_istream_t *stream);
  DigitalOutputPin *GetDigitalOutputPin(uint8_t pin_name);

private:
  std::vector<DigitalOutputPin> _digital_output_pins;
  std::vector<DigitalInputPin> _digital_input_pins;
  uint8_t _max_digital_pins;
};
extern Wippersnapper_V2 WsV2;
#endif // WS_DIGITALIO_CONTROLLER_H