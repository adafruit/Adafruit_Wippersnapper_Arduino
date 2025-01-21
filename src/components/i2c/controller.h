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
#include "drivers/drvPca9546.h"

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
  // Helpers
  bool IsBusStatusOK();
  drvBase* GetMuxDrv(uint32_t mux_address);
private:
  I2cModel *_i2c_model;       ///< I2c model
  I2cHardware *_i2c_hardware; ///< I2c hardware
  std::vector<drvBase *> _i2c_drivers;
};
extern Wippersnapper_V2 WsV2; ///< Wippersnapper V2 instance
#endif                        // WS_I2C_CONTROLLER_H