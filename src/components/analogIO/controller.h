/*!
 * @file src/components/analogIO/controller.h
 *
 * Controller for the AnalogIO API
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025-2026 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_ANALOGIO_CONTROLLER_H
#define WS_ANALOGIO_CONTROLLER_H
#include "model.h"
#include "wippersnapper.h"

class wippersnapper;    ///< Forward declaration
class AnalogIOModel;    ///< Forward declaration
class AnalogIOHardware; ///< Forward declaration

/*!
    @brief  Routes messages using the analogio.proto API to the
            appropriate hardware and model classes, controls and tracks
            the state of the hardware's analog I/O pins.
*/
class AnalogIOController {
public:
  AnalogIOController();
  ~AnalogIOController();
  // Routing
  bool Router(pb_istream_t *stream);
  bool Handle_AnalogIOAdd(ws_analogio_Add *msg);
  bool Handle_AnalogIORemove(ws_analogio_Remove *msg);
  void update(bool force = false);

  // Called by CheckinModel
  void SetMaxAnalogPins(uint8_t max_analog_pins);
  void SetRefVoltage(float voltage);

  // Sleep Mode
  bool UpdateComplete();
  void ResetFlags();

private:
  bool EncodePublishPinEvent(AnalogIOHardware *pin);
  bool RemovePin(uint8_t pin_num);
  AnalogIOHardware *GetPin(uint8_t pin_num);
  std::vector<AnalogIOHardware *> _pins; ///< Vector of analog pin objects
  AnalogIOModel *_analogio_model;        ///< AnalogIO model
  float _mcu_vref; ///< Reference voltage passed to each new pin
};
extern wippersnapper Ws; ///< Wippersnapper V2 instance
#endif                   // WS_ANALOGIO_CONTROLLER_H
