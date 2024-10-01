/*!
 * @file controller.h
 *
 * Controller for the AnalogIO API
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
#ifndef WS_ANALOGIO_CONTROLLER_H
#define WS_ANALOGIO_CONTROLLER_H
#include "Wippersnapper_V2.h"
#include "hardware.h"
#include "model.h"

class Wippersnapper_V2; ///< Forward declaration
class AnalogIOModel;    ///< Forward declaration
class AnalogIOHardware; ///< Forward declaration

/**
 * @struct analogioPin
 * @brief Structure repesenting an analogio pin on the device.
 */
struct analogioPin {
  uint8_t name;     ///< The pin's name.
  float period;     ///< The pin's period, in milliseconds.
  float prv_period; ///< The pin's previous period, in milliseconds.
  wippersnapper_sensor_SensorType read_mode; ///< Type of analog read to perform
};

/**************************************************************************/
/*!
    @brief  Routes messages using the analogio.proto API to the
            appropriate hardware and model classes, controls and tracks
            the state of the hardware's digital I/O pins.
*/
/**************************************************************************/
class AnalogIOController {
public:
  AnalogIOController();
  ~AnalogIOController();
  // Routing functions
  bool Handle_AnalogIOAdd(pb_istream_t *stream);
  bool Handle_AnalogIORemove(pb_istream_t *stream);
  // Polling loop function
  void update();

  void SetTotalAnalogPins(uint8_t total_pins);
  void SetRefVoltage(float voltage);
  float GetRefVoltage(void);
  bool IsPinTimerExpired(analogioPin *pin, ulong cur_time);

private:
  AnalogIOModel *_analogio_model;          ///< AnalogIO model
  AnalogIOHardware *_analogio_hardware;    ///< AnalogIO hardware
  std::vector<analogioPin> _analogio_pins; ///< Vector of analogio pins
  uint8_t _total_analogio_pins;            ///< Total number of analogio pins
  float _ref_voltage;                      ///< The device's reference voltage
};
extern Wippersnapper_V2 WsV2; ///< Wippersnapper V2 instance
#endif                        // WS_ANALOGIO_CONTROLLER_H