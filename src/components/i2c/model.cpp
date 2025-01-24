/*!
 * @file model.cpp
 *
 * Model for the i2c.proto message.
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

/***********************************************************************/
/*!
    @brief  I2C constructor
*/
/***********************************************************************/
I2cModel::I2cModel() {
  _msg_i2c_bus_scan = wippersnapper_i2c_I2cBusScan_init_default;
  _msg_i2c_bus_scanned = wippersnapper_i2c_I2cBusScan_init_default;
  _msg_i2c_device_add_replace =
      wippersnapper_i2c_I2cDeviceAddOrReplace_init_default;
  _msg_i2c_device_added_replaced =
      wippersnapper_i2c_I2cDeviceAddedOrReplaced_init_default;
  _msg_i2c_device_remove = wippersnapper_i2c_I2cDeviceRemove_init_default;
  _msg_i2c_device_removed = wippersnapper_i2c_I2cDeviceRemoved_init_default;
  _msg_i2c_device_event = wippersnapper_i2c_I2cDeviceEvent_init_default;
}

/****************************************************************************/
/*!
    @brief  Decodes a I2cDeviceRemove message from an input stream.
    @param    stream
                A pointer to the pb_istream_t stream.
    @returns  True if the I2cDeviceRemove message was decoded successfully,
              False otherwise.
*/
/****************************************************************************/
bool I2cModel::DecodeI2cDeviceRemove(pb_istream_t *stream) {
  _msg_i2c_device_remove = wippersnapper_i2c_I2cDeviceRemove_init_default;
  return pb_decode(stream, wippersnapper_i2c_I2cDeviceRemove_fields,
                   &_msg_i2c_device_remove);
}

/**********************************************************************/
/*!
    @brief    Returns a pointer to the I2cDeviceRemove message.
    @returns  Pointer to the I2cDeviceRemove message.
*/
/**********************************************************************/
wippersnapper_i2c_I2cDeviceRemove *I2cModel::GetI2cDeviceRemoveMsg() {
  return &_msg_i2c_device_remove;
}

/***************************************************************************/
/*!
    @brief    Decodes a I2cDeviceAddReplace message from an input stream.
    @returns  True if the stream was decoded successfully, False otherwise.
*/
/***************************************************************************/
bool I2cModel::DecodeI2cDeviceAddReplace(pb_istream_t *stream) {
  _msg_i2c_device_add_replace =
      wippersnapper_i2c_I2cDeviceAddOrReplace_init_default;
  return pb_decode(stream, wippersnapper_i2c_I2cDeviceAddOrReplace_fields,
                   &_msg_i2c_device_add_replace);
}

/**********************************************************************/
/*!
    @brief    Returns a pointer to the I2cDeviceAddOrReplace message.
    @returns  Pointer to the I2cDeviceAddOrReplace message.
*/
/**********************************************************************/
wippersnapper_i2c_I2cDeviceAddOrReplace *
I2cModel::GetI2cDeviceAddOrReplaceMsg() {
  return &_msg_i2c_device_add_replace;
}

/***************************************************************************/
/*!
    @brief    Encodes a I2cDeviceAddedOrReplaced message.
    @param    device_descriptor
              The I2cDeviceDescriptor message.
    @param    bus_status
              The I2cBusStatus message.
    @param    device_status
              The I2cDeviceStatus message.
    @returns  True if the message was encoded successfully, False otherwise.
*/
/***************************************************************************/
bool I2cModel::encodeMsgI2cDeviceAddedorReplaced(
    wippersnapper_i2c_I2cDeviceDescriptor device_descriptor,
    wippersnapper_i2c_I2cBusStatus bus_status,
    wippersnapper_i2c_I2cDeviceStatus device_status) {
  size_t sz_msg;

  // Fill I2cDeviceAddedOrReplaced message
  _msg_i2c_device_added_replaced =
      wippersnapper_i2c_I2cDeviceAddedOrReplaced_init_zero;
  _msg_i2c_device_added_replaced.has_i2c_device_description = true;
  _msg_i2c_device_added_replaced.i2c_device_description = device_descriptor;
  _msg_i2c_device_added_replaced.i2c_bus_status = bus_status;
  _msg_i2c_device_added_replaced.i2c_device_status = device_status;

  // Encode message
  if (!pb_get_encoded_size(&sz_msg,
                           wippersnapper_i2c_I2cDeviceAddedOrReplaced_fields,
                           &_msg_i2c_device_added_replaced))
    return false;

  uint8_t buf[sz_msg];
  pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
  return pb_encode(&msg_stream,
                   wippersnapper_i2c_I2cDeviceAddedOrReplaced_fields,
                   &_msg_i2c_device_added_replaced);
}

/**********************************************************************/
/*!
    @brief    Returns a pointer to the I2cDeviceAddedOrReplaced message.
    @returns  Pointer to the I2cDeviceAddedOrReplaced message.
*/
/**********************************************************************/
wippersnapper_i2c_I2cDeviceAddedOrReplaced *
I2cModel::GetMsgI2cDeviceAddedOrReplaced() {
  return &_msg_i2c_device_added_replaced;
}