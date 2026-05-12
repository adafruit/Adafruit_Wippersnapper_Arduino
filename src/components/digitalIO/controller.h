/*!
 * @file src/components/digitalIO/controller.h
 *
 * Controller for the digitalio API
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
#ifndef WS_DIGITALIO_CONTROLLER_H
#define WS_DIGITALIO_CONTROLLER_H
#include "model.h"
#include "wippersnapper.h"

class wippersnapper;
class DigitalIOModel;
class DigitalIOHardware;

/*!
    @brief  Routes messages using the digitalio.proto API to the
            appropriate hardware and model classes, controls and tracks
            the state of the hardware's digital I/O pins.
*/
class DigitalIOController {
public:
  DigitalIOController();
  ~DigitalIOController();
  bool Router(pb_istream_t *stream);
  bool Handle_DigitalIO_Add(ws_digitalio_Add *msg);
  bool Handle_DigitalIO_Remove(ws_digitalio_Remove *msg);
  bool Handle_DigitalIO_Write(ws_digitalio_Write *msg);
  void update(bool force = false);
  bool UpdateComplete();
  void ResetFlags();
  void SetMaxDigitalPins(uint8_t max_digital_pins);

private:
  bool EncodePublishPinEvent(DigitalIOHardware *pin);
  bool RemovePin(uint8_t pin_num);
  DigitalIOHardware *GetPin(uint8_t pin_num);
  std::vector<DigitalIOHardware *> _pins_input;
  std::vector<DigitalIOHardware *> _pins_output;
  DigitalIOModel *_dio_model;
};
extern wippersnapper Ws; ///< Wippersnapper V2 instance
#endif                   // WS_DIGITALIO_CONTROLLER_H