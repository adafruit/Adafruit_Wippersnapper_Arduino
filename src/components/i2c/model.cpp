/*!
 * @file src/components/i2c/model.cpp
 *
 * Model for the i2c.proto and i2c_output.proto messages.
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
    @brief  I2C constructor
*/
I2cModel::I2cModel() {
  memset(&_msg_i2c_bus_scan, 0, sizeof(_msg_i2c_bus_scan));
  memset(&_msg_i2c_bus_scanned, 0, sizeof(_msg_i2c_bus_scanned));
  memset(&_msg_i2c_device_add_replace, 0, sizeof(_msg_i2c_device_add_replace));
  memset(&_msg_i2c_device_added_replaced, 0,
         sizeof(_msg_i2c_device_added_replaced));
  memset(&_msg_i2c_device_remove, 0, sizeof(_msg_i2c_device_remove));
  memset(&_msg_i2c_device_removed, 0, sizeof(_msg_i2c_device_removed));
  memset(&_msg_i2c_device_event, 0, sizeof(_msg_i2c_device_event));
  memset(&_msg_i2c_device_output_write, 0,
         sizeof(_msg_i2c_device_output_write));
}

/*!
    @brief  I2C destructor
*/
I2cModel::~I2cModel() {
  memset(&_msg_i2c_bus_scan, 0, sizeof(_msg_i2c_bus_scan));
  memset(&_msg_i2c_bus_scanned, 0, sizeof(_msg_i2c_bus_scanned));
  memset(&_msg_i2c_device_add_replace, 0, sizeof(_msg_i2c_device_add_replace));
  memset(&_msg_i2c_device_added_replaced, 0,
         sizeof(_msg_i2c_device_added_replaced));
  memset(&_msg_i2c_device_remove, 0, sizeof(_msg_i2c_device_remove));
  memset(&_msg_i2c_device_removed, 0, sizeof(_msg_i2c_device_removed));
  memset(&_msg_i2c_device_event, 0, sizeof(_msg_i2c_device_event));
  memset(&_msg_i2c_device_output_write, 0,
         sizeof(_msg_i2c_device_output_write));
}

/*!
    @brief    Returns the numeric event value mapped to a sensor event.
    @param    sensor_type
                The SensorType.
    @param    event
                The sensors_event_t event.
    @returns  The value of the SensorType.
*/
float GetValueFromSensorsEvent(ws_sensor_Type sensor_type,
                               sensors_event_t *event) {
  float value = 0.0;
  switch (sensor_type) {
  case ws_sensor_Type_T_AMBIENT_TEMPERATURE:
    value = event->temperature;
    break;
  case ws_sensor_Type_T_AMBIENT_TEMPERATURE_FAHRENHEIT:
    value = event->temperature;
    break;
  case ws_sensor_Type_T_OBJECT_TEMPERATURE:
    value = event->temperature;
    break;
  case ws_sensor_Type_T_OBJECT_TEMPERATURE_FAHRENHEIT:
    value = event->temperature;
    break;
  case ws_sensor_Type_T_RAW:
    value = event->data[0];
    break;
  case ws_sensor_Type_T_RELATIVE_HUMIDITY:
    value = event->relative_humidity;
    break;
  case ws_sensor_Type_T_PRESSURE:
    value = event->pressure;
    break;
  case ws_sensor_Type_T_VOLTAGE:
    value = event->voltage;
    break;
  case ws_sensor_Type_T_CURRENT:
    value = event->current;
    break;
  case ws_sensor_Type_T_CO2:
    value = event->CO2;
    break;
  case ws_sensor_Type_T_ECO2:
    value = event->eCO2;
    break;
  case ws_sensor_Type_T_TVOC:
    value = event->tvoc;
    break;
  case ws_sensor_Type_T_VOC_INDEX:
    value = event->voc_index;
    break;
  case ws_sensor_Type_T_NOX_INDEX:
    value = event->nox_index;
    break;
  case ws_sensor_Type_T_PM10_STD:
    value = event->pm10_std;
    break;
  case ws_sensor_Type_T_PM25_STD:
    value = event->pm25_std;
    break;
  case ws_sensor_Type_T_PM100_STD:
    value = event->pm100_std;
    break;
  case ws_sensor_Type_T_GAS_RESISTANCE:
    value = event->gas_resistance;
    break;
  case ws_sensor_Type_T_ALTITUDE:
    value = event->altitude;
    break;
  case ws_sensor_Type_T_UNITLESS_PERCENT:
    value = event->unitless_percent;
    break;
  case ws_sensor_Type_T_LIGHT:
    value = event->light;
    break;
  default:
    value = 0.0;
    break;
  }
  return value;
}

/*!
    @brief  Decodes a I2cDeviceRemove message from an input stream.
    @param    stream
                A pointer to the pb_istream_t stream.
    @returns  True if the I2cDeviceRemove message was decoded successfully,
              False otherwise.
*/
bool I2cModel::DecodeI2cDeviceRemove(pb_istream_t *stream) {
  WS_DEBUG_PRINTLN("[i2c] Set _msg_i2c_device_remove...");
  memset(&_msg_i2c_device_remove, 0, sizeof(_msg_i2c_device_remove));
  bool is_success = false;
  is_success =
      pb_decode(stream, ws_i2c_DeviceRemove_fields, &_msg_i2c_device_remove);
  WS_DEBUG_PRINT("is_success: ");
  WS_DEBUG_PRINTLN(is_success);
  return is_success;
}

/*!
    @brief    Returns a pointer to the I2cDeviceRemove message.
    @returns  Pointer to the I2cDeviceRemove message.
*/
ws_i2c_DeviceRemove *I2cModel::GetI2cDeviceRemoveMsg() {
  return &_msg_i2c_device_remove;
}

/*!
    @brief    Decodes a I2cBusScan message from an input stream.
    @param    stream
                A pointer to the pb_istream_t stream.
    @returns  True if the I2cBusScan message was decoded successfully, False
              otherwise.
*/
bool I2cModel::DecodeI2cBusScan(pb_istream_t *stream) {
  memset(&_msg_i2c_bus_scan, 0, sizeof(_msg_i2c_bus_scan));
  return pb_decode(stream, ws_i2c_BusScan_fields, &_msg_i2c_bus_scan);
}

/*!
    @brief    Returns a pointer to the I2cBusScan message.
    @returns  Pointer to a I2cBusScan message.
*/
ws_i2c_BusScan *I2cModel::GetI2cBusScanMsg() { return &_msg_i2c_bus_scan; }

/*!
    @brief    Returns a pointer to the I2cBusScanned message.
    @returns  Pointer to a I2cBusScanned message.
*/
ws_i2c_BusScanned *I2cModel::GetI2cBusScannedMsg() {
  return &_msg_i2c_bus_scanned;
}

/*!
    @brief    Clears the I2cBusScanned message.
*/
void I2cModel::ClearI2cBusScanned() {
  memset(&_msg_i2c_bus_scanned, 0, sizeof(_msg_i2c_bus_scanned));
  _msg_i2c_bus_scanned.bus_found_devices_count = 0; // zero-out the count
}

/*!
    @brief    Adds a device to the I2cBusScanned message.
    @param    bus_scl
                The device's SCL pin.
    @param    bus_sda
                The device's SDA pin.
    @param    addr_device
                The device's i2c address.
    @param    addr_mux
                Optional MUX address.
    @param    mux_channel
                Optional MUX channel
    @returns  True if the device was added to the bus scan, False otherwise.
*/
bool I2cModel::AddDeviceToBusScan(const char *bus_scl, const char *bus_sda,
                                  uint32_t addr_device, uint32_t addr_mux,
                                  uint32_t mux_channel) {
  pb_size_t idx_device = _msg_i2c_bus_scanned.bus_found_devices_count;
  if (idx_device >= MAX_I2C_SCAN_DEVICES)
    return false;
  // Fill I2cDeviceDescriptor
  strcpy(_msg_i2c_bus_scanned.bus_found_devices[idx_device].bus_scl, bus_scl);
  strcpy(_msg_i2c_bus_scanned.bus_found_devices[idx_device].bus_sda, bus_sda);
  _msg_i2c_bus_scanned.bus_found_devices[idx_device].device_address =
      addr_device;
  // Optionally fill MUX info
  if (_msg_i2c_bus_scanned.bus_found_devices[idx_device].mux_address !=
      0xFFFF) {
    _msg_i2c_bus_scanned.bus_found_devices[idx_device].mux_address = addr_mux;
    _msg_i2c_bus_scanned.bus_found_devices[idx_device].mux_channel =
        mux_channel;
  }
  _msg_i2c_bus_scanned.bus_found_devices_count++;
  return true;
}

/*!
    @brief    Decodes a I2cDeviceAddReplace message from an input stream.
    @param   stream
                A pointer to the pb_istream_t stream.
    @returns  True if the stream was decoded successfully, False otherwise.
*/
bool I2cModel::DecodeI2cDeviceAddReplace(pb_istream_t *stream) {
  memset(&_msg_i2c_device_add_replace, 0, sizeof(_msg_i2c_device_add_replace));
  return pb_decode(stream, ws_i2c_DeviceAddOrReplace_fields,
                   &_msg_i2c_device_add_replace);
}

/*!
    @brief    Returns a pointer to the I2cDeviceAddOrReplace message.
    @returns  Pointer to the I2cDeviceAddOrReplace message.
*/
ws_i2c_DeviceAddOrReplace *I2cModel::GetI2cDeviceAddOrReplaceMsg() {
  return &_msg_i2c_device_add_replace;
}

/*!
    @brief    Returns a pointer to the I2cOutputAdd message.
    @returns  Pointer to the I2cOutputAdd message.
*/
ws_i2c_output_Add *I2cModel::GetI2cOutputAddMsg() {
  return &_msg_i2c_device_add_replace.output_add;
}

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
bool I2cModel::encodeMsgI2cDeviceAddedorReplaced(
    ws_i2c_DeviceDescriptor device_descriptor, ws_i2c_BusStatus bus_status,
    ws_i2c_DeviceStatus device_status) {
  size_t sz_msg;

  // Fill I2cDeviceAddedOrReplaced message
  memset(&_msg_i2c_device_added_replaced, 0,
         sizeof(_msg_i2c_device_added_replaced));
  _msg_i2c_device_added_replaced.has_device_description = true;
  _msg_i2c_device_added_replaced.device_description = device_descriptor;
  _msg_i2c_device_added_replaced.bus_status = bus_status;
  _msg_i2c_device_added_replaced.device_status = device_status;

  // Encode message
  if (!pb_get_encoded_size(&sz_msg, ws_i2c_DeviceAddedOrReplaced_fields,
                           &_msg_i2c_device_added_replaced))
    return false;

  uint8_t buf[sz_msg];
  pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
  return pb_encode(&msg_stream, ws_i2c_DeviceAddedOrReplaced_fields,
                   &_msg_i2c_device_added_replaced);
}

/*!
    @brief    Returns a pointer to the I2cDeviceAddedOrReplaced message.
    @returns  Pointer to the I2cDeviceAddedOrReplaced message.
*/
ws_i2c_DeviceAddedOrReplaced *I2cModel::GetMsgI2cDeviceAddedOrReplaced() {
  return &_msg_i2c_device_added_replaced;
}

/*!
    @brief    Clears the I2cDeviceEvent message.
*/
void I2cModel::ClearI2cDeviceEvent() {
  memset(&_msg_i2c_device_event, 0, sizeof(_msg_i2c_device_event));
  _msg_i2c_device_event.device_events_count = 0;
}

/*!
    @brief    Sets the I2cDeviceEvent message's device description.
    @param    bus_scl
                The SCL bus.
    @param    bus_sda
                The SDA bus.
    @param    addr_device
                The device address.
    @param    addr_mux
                The MUX address.
    @param    mux_channel
                The MUX channel.
*/
void I2cModel::SetI2cDeviceEventDeviceDescripton(const char *bus_scl,
                                                 const char *bus_sda,
                                                 uint32_t addr_device,
                                                 uint32_t addr_mux,
                                                 uint32_t mux_channel) {
  _msg_i2c_device_event.has_device_description = true;
  strcpy(_msg_i2c_device_event.device_description.bus_scl, bus_scl);
  strcpy(_msg_i2c_device_event.device_description.bus_sda, bus_sda);
  _msg_i2c_device_event.device_description.device_address = addr_device;
  _msg_i2c_device_event.device_description.mux_address = addr_mux;
  _msg_i2c_device_event.device_description.mux_channel = mux_channel;
}

/*!
    @brief    Adds a SensorEvent to the I2cDeviceEvent message.
    @param    event
                The sensors_event_t event.
    @param    sensor_type
                The SensorType.
    @returns  True if the SensorEvent was added successfully, False otherwise.
*/
bool I2cModel::AddI2cDeviceSensorEvent(sensors_event_t &event,
                                       ws_sensor_Type sensor_type) {
  if (_msg_i2c_device_event.device_events_count >= MAX_DEVICE_EVENTS)
    return false; // Maximum amount of events reached

  _msg_i2c_device_event.device_events[_msg_i2c_device_event.device_events_count]
      .type = sensor_type;
  float value = GetValueFromSensorsEvent(sensor_type, &event);
  _msg_i2c_device_event.device_events[_msg_i2c_device_event.device_events_count]
      .value.float_value = value;

  _msg_i2c_device_event.device_events_count++;
  return true;
}

/*!
    @brief    Encodes an I2cDeviceEvent message.
    @returns  True if the message was encoded successfully, False otherwise.
*/
bool I2cModel::EncodeI2cDeviceEvent() {
  size_t sz_msg;
  if (!pb_get_encoded_size(&sz_msg, ws_i2c_DeviceEvent_fields,
                           &_msg_i2c_device_event))
    return false;

  uint8_t buf[sz_msg];
  pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
  return pb_encode(&msg_stream, ws_i2c_DeviceEvent_fields,
                   &_msg_i2c_device_event);
}

/*!
    @brief    Returns a pointer to the I2cDeviceEvent message.
    @returns  Pointer to the I2cDeviceEvent message.
*/
ws_i2c_DeviceEvent *I2cModel::GetI2cDeviceEvent() {
  return &_msg_i2c_device_event;
}

/*!
    @brief    Decodes a I2cDeviceOutputWrite message from an input stream.
    @param    stream
              A pointer to the pb_istream_t stream.
    @returns  True if the I2cDeviceOutputWrite message was decoded successfully,
   False otherwise.
*/
bool I2cModel::DecodeI2cDeviceOutputWrite(pb_istream_t *stream) {
  memset(&_msg_i2c_device_output_write, 0,
         sizeof(_msg_i2c_device_output_write));
  return pb_decode(stream, ws_i2c_DeviceOutputWrite_fields,
                   &_msg_i2c_device_output_write);
}

/*!
    @brief    Returns a pointer to the I2cDeviceOutputWrite message.
    @returns  Pointer to the I2cDeviceOutputWrite message.
*/
ws_i2c_DeviceOutputWrite *I2cModel::GetI2cDeviceOutputWriteMsg() {
  return &_msg_i2c_device_output_write;
}

/*!
    @brief  I2cOutputModel constructor
*/
I2cOutputModel::I2cOutputModel() {
  memset(&_msg_led_backpack_write, 0, sizeof(_msg_led_backpack_write));
  memset(&_msg_char_lcd_write, 0, sizeof(_msg_char_lcd_write));
}

/*!
    @brief  I2cOutputModel destructor
*/
I2cOutputModel::~I2cOutputModel() {
  memset(&_msg_led_backpack_write, 0, sizeof(_msg_led_backpack_write));
  memset(&_msg_char_lcd_write, 0, sizeof(_msg_char_lcd_write));
}

/*!
    @brief    Decodes a LedBackpackWrite message from an input stream.
    @param    stream
              A pointer to the pb_istream_t stream.
    @returns  True if the LedBackpackWrite message was decoded successfully,
              False otherwise.
*/
bool I2cOutputModel::DecodeLedBackpackWrite(pb_istream_t *stream) {
  memset(&_msg_led_backpack_write, 0, sizeof(_msg_led_backpack_write));
  return pb_decode(stream, ws_i2c_output_LedBackpackWrite_fields,
                   &_msg_led_backpack_write);
}

/*!
    @brief    Decodes a CharLCDWrite message from an input stream.
    @param    stream
              A pointer to the pb_istream_t stream.
    @returns  True if the CharLCDWrite message was decoded successfully,
              False otherwise.
*/
bool I2cOutputModel::DecodeCharLCDWrite(pb_istream_t *stream) {
  memset(&_msg_char_lcd_write, 0, sizeof(_msg_char_lcd_write));
  return pb_decode(stream, ws_i2c_output_CharLCDWrite_fields,
                   &_msg_char_lcd_write);
}

/*!
    @brief    Returns a pointer to the LedBackpackWrite message.
    @returns  Pointer to the LedBackpackWrite message.
*/
ws_i2c_output_LedBackpackWrite *I2cOutputModel::GetLedBackpackWriteMsg() {
  return &_msg_led_backpack_write;
}

/*!
    @brief    Returns a pointer to the CharLCDWrite message.
    @returns  Pointer to the CharLCDWrite message.
*/
ws_i2c_output_CharLCDWrite *I2cOutputModel::GetCharLCDWriteMsg() {
  return &_msg_char_lcd_write;
}
