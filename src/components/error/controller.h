/*!
 * @file src/components/error/controller.h
 *
 * Model for the Wippersnapper error proto API.
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
#ifndef WS_CONTROLLER_MODEL
#define WS_CONTROLLER_MODEL
#include "model.h"
#include "wippersnapper.h"

class ErrorModel;    ///< Forward declaration
class wippersnapper; ///< Forward declaration

/*!
    @brief  Provides an interface for routing and handling
            error.proto messages.
*/
class ErrorController {
public:
  ErrorController();
  ~ErrorController();
  bool Router(pb_istream_t *stream);
  bool PublishAnalogIO(const char *error_msg, const char *pin_name);
  bool PublishDigitalIO(const char *error_msg, const char *pin_name);
  bool PublishDS18x20(const char *error_msg, const char *pin_name);
  bool PublishPixels(const char *error_msg, const char *pin_name);
  bool PublishPWM(const char *error_msg, const char *pin_name);
  bool PublishServo(const char *error_msg, const char *pin_name);
  bool PublishGPS(const char *error_msg, ws_i2c_DeviceDescriptor *i2c_device);
  bool PublishGPS(const char *error_msg, ws_uart_Descriptor *uart_device);
  bool PublishI2C(const char *error_msg, ws_i2c_DeviceDescriptor *i2c_device);
  bool PublishUART(const char *error_msg, ws_uart_Descriptor *uart_device);

private:
  bool HandleThrottle(const ws_error_ErrorIOThrottle &throttle);
  bool HandleBan(const ws_error_ErrorIOBan &ban);
  bool PublishError(pb_size_t which_component_type,
                    pb_size_t which_component_id, pb_callback_t pin,
                    pb_callback_t error_msg);
  bool PublishError(pb_size_t which_component_type,
                    pb_size_t which_component_id, ws_i2c_DeviceDescriptor i2c,
                    pb_callback_t error_msg);
  bool PublishError(pb_size_t which_component_type,
                    pb_size_t which_component_id, ws_uart_Descriptor uart,
                    pb_callback_t error_msg);
  ErrorModel *_model = nullptr; ///< Instance of ErrorModel class
};
extern wippersnapper Ws; ///< Wippersnapper V2 instance
#endif                   // WS_CONTROLLER_MODEL