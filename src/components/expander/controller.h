/*!
 * @file src/components/expander/controller.h
 *
 * Controller for WipperSnapper's expander component, bridges between the
 * expander.proto API, the model, and the hardware layer.
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
#ifndef WS_EXPANDER_CONTROLLER_H
#define WS_EXPANDER_CONTROLLER_H
#include "drivers/drv_aw9523.h"
#include "drivers/drv_mcp23x08.h"
#include "drivers/drv_mcp23x17.h"
#include "drivers/drv_pcf8574.h"
#include "drivers/drv_pcf8575.h"
#include "drivers/drv_seesaw.h"
#include "drivers/drv_tca8418.h"
#include "hardware.h"
#include "model.h"
#include "wippersnapper.h"

class wippersnapper;    ///< Forward declaration
class ExpanderModel;    ///< Forward declaration
class ExpanderHardware; ///< Forward declaration

/*!
    @brief  Routes messages between the expander.proto API and the hardware.
*/
class ExpanderController {
public:
  ExpanderController();
  ~ExpanderController();
  bool Router(pb_istream_t *stream);
  bool Handle_Add(ws_expander_Add *msg);
  bool Handle_Remove(ws_expander_Remove *msg);
  ExpanderHardware *GetDriver(uint8_t addr);

private:
  bool AddExpander(const char *device_name, uint8_t i2c_addr, TwoWire *wire);
  ExpanderModel *_model;                      ///< Expander model instance
  std::vector<ExpanderHardware *> _expanders; ///< Expander hardware instances
};
extern wippersnapper Ws; ///< Wippersnapper V2 instance
#endif                   // WS_EXPANDER_CONTROLLER_H
