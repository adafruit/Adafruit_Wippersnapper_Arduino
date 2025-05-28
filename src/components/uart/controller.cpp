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

}

/**************************************************************************/
/*!
    @brief  Destructs the UARTController.
*/
/**************************************************************************/
UARTController::~UARTController() {

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
    // TODO: Needs implementation
    return false;
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
