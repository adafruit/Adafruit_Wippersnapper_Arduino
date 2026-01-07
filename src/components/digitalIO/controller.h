/*!
 * @file src/components/digitalIO/controller.h
 *
 * Controller for the digitalio API
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_DIGITALIO_CONTROLLER_H
#define WS_DIGITALIO_CONTROLLER_H
#include "wippersnapper.h"
#include "hardware.h"
#include "model.h"

class wippersnapper;

/**
 * @struct DigitalIOPin
 * @brief This struct represents a digital I/O pin.
 */
struct DigitalIOPin {
  uint8_t pin_name;                     ///< The pin's name.
  ws_digitalio_Direction pin_direction; ///< The pin's direction.
  ws_digitalio_SampleMode sample_mode;  ///< The pin's sample mode.
  bool pin_value;                       ///< The pin's value.
  bool prv_pin_value;                   ///< The pin's previous value.
  ulong pin_period;                     ///< The pin's period.
  ulong prv_pin_time;                   ///< The pin's previous time.
  bool did_read_send;                   ///< True if the last read was sent to IO, False otherwise.
};

class DigitalIOModel;    // Forward declaration
class DigitalIOHardware; // Forward declaration

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
  void update(bool force_read_all = false);
  bool UpdateComplete();

  // Called once per-run, during CheckinResponse processing
  void SetMaxDigitalPins(uint8_t max_digital_pins);

private:
  bool EncodePublishPinEvent(uint8_t pin_name, bool pin_value);
  bool CheckEventPin(DigitalIOPin *pin);
  bool CheckTimerPin(DigitalIOPin *pin);
  bool IsPinTimerExpired(DigitalIOPin *pin, ulong cur_time);
  void PrintPinValue(DigitalIOPin *pin);
  int GetPinIdx(uint8_t pin_name);
  std::vector<DigitalIOPin> _pins_input;
  std::vector<DigitalIOPin> _pins_output;
  DigitalIOModel *_dio_model;
  DigitalIOHardware *_dio_hardware;
};
extern wippersnapper Ws; ///< Wippersnapper V2 instance
#endif                        // WS_DIGITALIO_CONTROLLER_H