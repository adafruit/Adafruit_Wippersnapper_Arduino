/*!
 * @file src/components/uart/controller.cpp
 *
 * Controller for WipperSnapper's UART component, bridges between the UART.proto API,
 * the model, and the hardware layer.
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
#include "controller.h"

/**************************************************************************/
/*!
    @brief  Constructs a new UARTController.
*/
/**************************************************************************/
UARTController::UARTController() {
  // TODO! Needs implementation
}

/**************************************************************************/
/*!
    @brief  Destructs the UARTController.
*/
/**************************************************************************/
UARTController::~UARTController() {
 // TODO! Needs impl.
}

/**************************************************************************/
/*!
    @brief  Handles a UartAdd message.
    @param  stream
            Pointer to a pb_istream_t object.
    @return True if the message was handled successfully, False otherwise.
*/
/**************************************************************************/
bool UARTController::Handle_UartAdd(pb_istream_t *stream) {
  // Attempt to decode the UartAdd message
  if (!_uart_model->DecodeUartAdd(stream))
    return false;
  // Get ref. to the UartAdd message within the model
  wippersnapper_uart_UartAdd *add_msg = _uart_model->GetUartAddMsg();
  // TODO: fix the id field, currently it is a callback and should be a string.
  if (! add_msg->has_cfg_serial && ! add_msg->has_cfg_device) {
    WS_DEBUG_PRINTLN("[uart] ERROR: No configuration provided for UART device!");
    return false;
  }
  // Get the serial and device configuration from the UartAdd message
  wippersnapper_uart_UartSerialConfig cfg_serial = add_msg->cfg_serial;
  wippersnapper_uart_UartDeviceConfig cfg_device = add_msg->cfg_device;

  // Let's first configure the hardware serial settings
  // TODO: Implement hardware serial configuration
  // Create new uarthardware instance 
  // then, create a new UartDevice "driver" on the hardware layer (UARTHardware)

  // TODO: Publish back to IO that the UART device was added
  return true;
}

/**************************************************************************/
/*!
    @brief  Handles a UartRemove message.
    @param  stream
            Pointer to a pb_istream_t object.
    @return True if the message was handled successfully, False otherwise.
*/
/**************************************************************************/
bool UARTController::Handle_UartRemove(pb_istream_t *stream) {
    // TODO: Needs implementation
    return false;
}

/**************************************************************************/
/*!
    @brief  Handles a UartWrite message.
    @param  stream
            Pointer to a pb_istream_t object.
    @return True if the message was handled successfully, False otherwise.
*/
/**************************************************************************/
bool UARTController::Handle_UartWrite(pb_istream_t *stream) {
    // TODO: Needs implementation
    return false;
}

/**************************************************************************/
/*!
    @brief  Updates all UART devices.
*/
/**************************************************************************/
void UARTController::update() {
    // TODO: Needs implementation
}
