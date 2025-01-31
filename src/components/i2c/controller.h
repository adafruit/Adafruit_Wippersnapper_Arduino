/*!
 * @file controller.h
 *
 * Routing controller for WipperSnapper's I2C component.
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
#ifndef WS_I2C_CONTROLLER_H
#define WS_I2C_CONTROLLER_H
#include "Wippersnapper_V2.h"
#include "hardware.h"
#include "model.h"
// I2C Drivers
#include "drivers/drvBase.h" ///< Base driver class
#include "drivers/drvBME280.h"

class Wippersnapper_V2; ///< Forward declaration
class I2cModel;         ///< Forward declaration
class I2cHardware;      ///< Forward declaration

/**************************************************************************/
/*!
    @brief  Routes messages using the i2c.proto API to the
            appropriate hardware, model, and device driver classes.
*/
/**************************************************************************/
class I2cController {
public:
  I2cController();
  ~I2cController();
  void update();
  // Routing
  bool Handle_I2cBusScan(pb_istream_t *stream);
  bool Handle_I2cDeviceAddOrReplace(pb_istream_t *stream);
  bool Handle_I2cDeviceRemove(pb_istream_t *stream);
  // Publishing
  bool PublishI2cDeviceAddedorReplaced(const wippersnapper_i2c_I2cDeviceDescriptor& device_descriptor, const wippersnapper_i2c_I2cDeviceStatus& device_status);
  // Helpers
  bool IsBusStatusOK(bool is_alt_bus);
  void ConfigureMuxChannel(uint32_t mux_channel, bool is_alt_bus);
private:
  I2cModel *_i2c_model;                 ///< Pointer to an I2C model object
  I2cHardware *_i2c_bus_default;        ///< Pointer to the default I2C bus
  I2cHardware *_i2c_bus_alt;            ///< Pointer to an alternative I2C bus
  std::vector<drvBase *> _i2c_drivers; ///< Vector of ptrs to I2C device drivers
};
extern Wippersnapper_V2 WsV2; ///< Wippersnapper V2 instance
#endif                        // WS_I2C_CONTROLLER_H