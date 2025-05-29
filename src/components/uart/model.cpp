/*!
 * @file src/components/uart/model.cpp
 *
 * Model implementation for the UART.proto message.
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
#include "model.h"

/*!
    @brief  Constructs a new UARTModel.
*/
UARTModel::UARTModel() {
  memset(&_msg_UartAdd, 0, sizeof(_msg_UartAdd));
  memset(&_msg_UartAdded, 0, sizeof(_msg_UartAdded));
  memset(&_msg_UartRemove, 0, sizeof(_msg_UartRemove));
  memset(&_msg_UartWrite, 0, sizeof(_msg_UartWrite));
  memset(&_msg_UartWritten, 0, sizeof(_msg_UartWritten));
}

/*!
    @brief  Destructs the UARTModel.
*/
UARTModel::~UARTModel() {
  memset(&_msg_UartAdd, 0, sizeof(_msg_UartAdd));
  memset(&_msg_UartAdded, 0, sizeof(_msg_UartAdded));
  memset(&_msg_UartRemove, 0, sizeof(_msg_UartRemove));
  memset(&_msg_UartWrite, 0, sizeof(_msg_UartWrite));
  memset(&_msg_UartWritten, 0, sizeof(_msg_UartWritten));
}

/*!
    @brief  Decodes a UartAdd message from a protobuf input stream.
    @param  stream
            Pointer to a pb_istream_t object.
    @return True if the message was decoded successfully, False otherwise.
*/
bool UARTModel::DecodeUartAdd(pb_istream_t *stream) {
  return pb_decode(stream, wippersnapper_uart_UartAdd_fields, &_msg_UartAdd);
}

/*!
    @brief  Gets a pointer to the decoded UartAdd message.
    @return Pointer to the decoded UartAdd message.
*/
wippersnapper_uart_UartAdd *UARTModel::GetUartAddMsg() {
  return &_msg_UartAdd;
}

/*!
    @brief  Encodes a UartAdded message.
    @param  id
            The ID of the UART device.
    @param  success
            Whether the UART device was added successfully.
    @return True if the message was encoded successfully, False otherwise.
*/
bool UARTModel::EncodeUartAdded(const char *id, bool success) {
  // TODO: Implement
  return false;
}

/*!
    @brief  Gets a pointer to the encoded UartAdded message.
    @return Pointer to the encoded UartAdded message.
*/
wippersnapper_uart_UartAdded *UARTModel::GetUartAddedMsg() {
  return &_msg_UartAdded;
}