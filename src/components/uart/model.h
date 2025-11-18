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
#include <Adafruit_Sensor.h>
#define MAX_UART_INPUT_EVENTS 15 ///< Maximum number of UART input events

/*!
    @brief  Provides an interface for creating, encoding, and parsing
            messages from UART.proto.
*/
class UARTModel {
public:
  UARTModel();
  ~UARTModel();
  // Messaging APIs //
  // UartAdd
  bool DecodeUartAdd(pb_istream_t *stream);
  bool DecodeUartDeviceRemove(pb_istream_t *stream);
  ws_uart_Add *GetUartAddMsg();
  // UartAdded
  bool EncodeUartAdded(int32_t uart_nbr, ws_uart_DeviceType type,
                       const char *id, bool success);
  ws_uart_Added *GetUartAddedMsg();
  // UartRemove
  ws_uart_Remove *GetUartRemoveMsg();
  // UartInputEvent
  bool AddUartInputEvent(sensors_event_t &event, ws_sensor_Type sensor_type);
  bool EncodeUartInputEvent();
  ws_uart_InputEvent *GetUartInputEventMsg();
  void ClearUartInputEventMsg();
  void ConfigureUartInputEventMsg(uint32_t uart_nbr, ws_uart_DeviceType type,
                                  const char *device_id);

private:
  ws_uart_Add _msg_UartAdd;               ///< ws_uart_Add message
  ws_uart_Added _msg_UartAdded;           ///< ws_uart_Added message
  ws_uart_Remove _msg_UartRemove;         ///< ws_uart_Remove message
  ws_uart_Write _msg_UartWrite;           ///< ws_uart_Write message
  ws_uart_Written _msg_UartWritten;       ///< ws_uart_Written message
  ws_uart_InputEvent _msg_UartInputEvent; ///< ws_uart_InputEvent message
};
#endif // WS_UART_MODEL_H