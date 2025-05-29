/*!
 * @file src/components/uart/model.h
 *
 * Model interface for the UART.proto message.
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
#ifndef WS_UART_MODEL_H
#define WS_UART_MODEL_H
#include "Wippersnapper_V2.h"

/*!
    @brief  Provides an interface for creating, encoding, and parsing
            messages from UART.proto.
*/
class UARTModel {
public:
  UARTModel();
  ~UARTModel();
  // Message API
  bool DecodeUartAdd(pb_istream_t *stream);
  wippersnapper_uart_UartAdd *GetUartAddMsg();
  bool EncodeUartAdded(const char *id, bool success);
  wippersnapper_uart_UartAdded *GetUartAddedMsg();
private:
  wippersnapper_uart_UartAdd _msg_UartAdd;         ///< wippersnapper_uart_UartAdd message
  wippersnapper_uart_UartAdded _msg_UartAdded;     ///< wippersnapper_uart_UartAdded message
  wippersnapper_uart_UartRemove _msg_UartRemove;   ///< wippersnapper_uart_UartRemove message
  wippersnapper_uart_UartWrite _msg_UartWrite;     ///< wippersnapper_uart_UartWrite message
  wippersnapper_uart_UartWritten _msg_UartWritten; ///< wippersnapper_uart_UartWritten message
};
#endif // WS_UART_MODEL_H