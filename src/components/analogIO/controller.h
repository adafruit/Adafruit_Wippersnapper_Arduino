/*!
 * @file src/components/analogIO/controller.h
 *
 * Controller for the AnalogIn API
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
class ExpanderHardware; ///< Forward declaration

/*!
    @brief  Routes messages using the analogin.proto API to the
            appropriate hardware and model classes, controls and tracks
            the state of the hardware's analog input pins.
*/
class AnalogIOController {
public:
  AnalogIOController();
  ~AnalogIOController();
  // Routing
  bool Router(pb_istream_t *stream);
  bool Handle_AnalogInAdd(ws_analogin_Add *msg);
  bool Handle_AnalogInRemove(ws_analogin_Remove *msg);
  void update(bool force = false);

  // Called by CheckinModel
  void SetMaxAnalogPins(uint8_t max_analog_pins);
  void SetRefVoltage(float voltage);

  // Sleep Mode
  bool UpdateComplete();
  void ResetFlags();

private:
  bool EncodePublishPinEvent(AnalogIOHardware *pin);
  bool RemovePin(uint8_t pin_num, ExpanderHardware *expander);
  AnalogIOHardware *GetPin(uint8_t pin_num, ExpanderHardware *expander);
  std::vector<AnalogIOHardware *> _pins; ///< Vector of analog pin objects
  AnalogIOModel *_analogin_model;        ///< AnalogIO model
  float _mcu_ref_voltage; ///< MCU/board reference voltage, used for native pins
                          ///< (expander pins use their own message ref_voltage)
};
extern wippersnapper *Ws; ///< Wippersnapper V2 instance
#endif                    // WS_ANALOGIO_CONTROLLER_H
