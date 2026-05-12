/*!
 * @file src/components/digitalIO/hardware.h
 *
 * Hardware for the digitalio.proto message.
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
#ifndef WS_DIGITALIO_HARDWARE_H
#define WS_DIGITALIO_HARDWARE_H
#include "wippersnapper.h"

class ExpanderHardware;

/*!
    @brief  Represents a single digital I/O pin and provides
            hardware-level operations for reading, writing,
            and polling its state.
*/
class DigitalIOHardware {
public:
  DigitalIOHardware(uint8_t pin_name, ws_digitalio_Direction direction,
                    ws_digitalio_SampleMode sample_mode, bool initial_value,
                    ulong period, ExpanderHardware *expander_drv);
  ~DigitalIOHardware();

  // Pin operations
  void Write(bool value);
  bool ReadValue();

  // Polling
  bool CheckEvent();
  bool CheckTimer();

  // Getters
  uint8_t GetPinNum() const;
  bool GetPinValue() const;
  ws_digitalio_Direction GetDirection() const;
  ws_digitalio_SampleMode GetSampleMode() const;
  ExpanderHardware *GetExpanderDriver() const;

  // Sleep cycle support
  bool DidReadSend() const;
  void MarkSent();
  void ResetSendFlag();

private:
  bool SetMode();
  bool IsPinTimerExpired(ulong cur_time);
  bool IsStatusLEDPin() const;

  uint8_t _name;
  ws_digitalio_Direction _direction;
  ws_digitalio_SampleMode _sample_mode;
  bool _value;
  bool _prv_value;
  ulong _period;
  ulong _prv_time;
  bool _did_read_send;
  ExpanderHardware *_expander_drv;
};
#endif // WS_DIGITALIO_HARDWARE_H