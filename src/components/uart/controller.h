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
#include "Wippersnapper_V2.h"
#include "hardware.h"
#include "model.h"
// drivers
#include "drivers/drvUartBase.h"

class Wippersnapper_V2; ///< Forward declaration
class UARTModel;        ///< Forward declaration
class UARTHardware;     ///< Forward declaration

/**************************************************************************/
/*!
    @brief  Routes messages between the uart.proto API and the hardware.
*/
/**************************************************************************/
class UARTController {
public:
  UARTController();
  ~UARTController();
  // Routing
  bool Handle_UartAdd(pb_istream_t *stream);
  bool Handle_UartRemove(pb_istream_t *stream);
  bool Handle_UartWrite(pb_istream_t *stream);
  // Polling
  void update();

private:
  UARTModel *_uart_model; ///< UART model
  std::vector<UARTHardware *>
      _uart_ports; ///< Vector of UART hardware instances
};
extern Wippersnapper_V2 WsV2; ///< Wippersnapper V2 instance
#endif                        // WS_UART_CONTROLLER_H