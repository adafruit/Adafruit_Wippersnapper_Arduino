/*!
 * @file src/components/digital/controller.h
 *
 * Controller for the digital.proto API
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2026 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_DIGITAL_CONTROLLER_H
#define WS_DIGITAL_CONTROLLER_H
#include "model.h"
#include "wippersnapper.h"

class wippersnapper;
class DigitalModel;
class DigitalHardware;
class ExpanderHardware;

/*!
    @brief  Routes messages using the digital.proto API to the
            appropriate hardware and model classes, controls and tracks
            the state of the hardware's digital pins.
*/
class DigitalController {
public:
  DigitalController();
  ~DigitalController();
  bool Router(pb_istream_t *stream);
  bool Handle_Digital_Add(ws_digital_Add *msg);
  bool Handle_Digital_Remove(ws_digital_Remove *msg);
  bool Handle_Digital_Write(ws_digital_Write *msg);
  void update(bool force = false);
  bool UpdateComplete();
  void ResetFlags();
  void SetMaxDigitalPins(uint8_t max_digital_pins);
  bool QueryPinState(uint8_t pin_num, ws_digital_Direction *direction,
                     bool *value);

private:
  bool EncodePublishPinEvent(DigitalHardware *pin);
  bool RemovePin(uint8_t pin_num, ExpanderHardware *expander);
  DigitalHardware *GetPin(uint8_t pin_num, ExpanderHardware *expander);
  std::vector<DigitalHardware *> _pins_input;
  std::vector<DigitalHardware *> _pins_output;
  DigitalModel *_digital_model;
};
extern wippersnapper *Ws; ///< Wippersnapper V2 instance
#endif                    // WS_DIGITAL_CONTROLLER_H