/*!
 * @file src/components/uart/controller.h
 *
 * Controller for WipperSnapper's UART component, bridges between the UART.proto
 * API, the model, and the hardware layer.
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
#ifndef WS_UART_CONTROLLER_H
#define WS_UART_CONTROLLER_H
#include "wippersnapper.h"
#include "hardware.h"
#include "model.h"
// drivers
#include "drivers/drvUartBase.h"
#include "drivers/drvUartPm25.h"
#include "drivers/drvUartUs100.h"

class wippersnapper; ///< Forward declaration
class UARTModel;        ///< Forward declaration
class UARTHardware;     ///< Forward declaration

/*!
    @brief  Routes messages between the uart.proto API and the hardware.
*/
class UARTController {
public:
  UARTController();
  ~UARTController();
  // Routing
  bool Router(pb_istream_t *stream);
  bool Handle_UartAdd(ws_uart_Add *msg);
  bool Handle_UartRemove(ws_uart_Remove *msg);
  bool Handle_UartWrite(ws_uart_Write *msg);
  // Polling
  void update();

private:
  UARTModel *_uart_model; ///< UART model
  std::vector<UARTHardware *>
      _uart_ports; ///< Vector of UART hardware instances
  std::vector<drvUartBase *>
      _uart_drivers; ///< Vector of UART device drivers (eg: PM2.5, etc.)
};
extern wippersnapper Ws; ///< Wippersnapper V2 instance
#endif                        // WS_UART_CONTROLLER_H