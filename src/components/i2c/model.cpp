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
  memset(&_msg_i2c_d2b, 0, sizeof(_msg_i2c_d2b));
  memset(&_msg_probed, 0, sizeof(_msg_probed));
  _probe_spaces_count = 0;
  _probe_addresses_count = 0;
  memset(&_msg_i2c_add, 0, sizeof(_msg_i2c_add));
  memset(&_msg_i2c_added, 0,
         sizeof(_msg_i2c_added));
  memset(&_msg_i2c_remove, 0, sizeof(_msg_i2c_remove));
  memset(&_msg_i2c_removed, 0, sizeof(_msg_i2c_removed));
  memset(&_msg_i2c_event, 0, sizeof(_msg_i2c_event));
}

/*!
    @brief  I2C destructor
*/
I2cModel::~I2cModel() {
  memset(&_msg_i2c_d2b, 0, sizeof(_msg_i2c_d2b));
  memset(&_msg_probed, 0, sizeof(_msg_probed));
  _probe_spaces_count = 0;
  _probe_addresses_count = 0;
  memset(&_msg_i2c_add, 0, sizeof(_msg_i2c_add));
  memset(&_msg_i2c_added, 0,
         sizeof(_msg_i2c_added));
  memset(&_msg_i2c_remove, 0, sizeof(_msg_i2c_remove));
  memset(&_msg_i2c_removed, 0, sizeof(_msg_i2c_removed));
  memset(&_msg_i2c_event, 0, sizeof(_msg_i2c_event));
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
  WS_DEBUG_PRINTLN("[i2c] Set _msg_i2c_remove...");
  memset(&_msg_i2c_remove, 0, sizeof(_msg_i2c_remove));
  bool is_success = false;
  is_success =
      pb_decode(stream, ws_i2c_Remove_fields, &_msg_i2c_remove);
  WS_DEBUG_PRINT("is_success: ");
  WS_DEBUG_PRINTLNVAR(is_success);
  return is_success;
}

/*!
    @brief    Returns a pointer to the I2cDeviceRemove message.
    @returns  Pointer to the I2cDeviceRemove message.
*/
ws_i2c_Remove *I2cModel::GetI2cDeviceRemoveMsg() {
  return &_msg_i2c_remove;
}

// ---- Probe decode/encode API ----

/*!
    @brief    Sets up nanopb decode callbacks on a Probe message before B2D decode.
    @param    probe
                Pointer to the ws_i2c_Probe inside the B2D union.
*/
void I2cModel::SetupProbeDecodeCallbacks(ws_i2c_Probe *probe) {
  _probe_spaces_count = 0;
  _probe_addresses_count = 0;
  probe->address_spaces.funcs.decode = cbDecodeAddressSpace;
  probe->address_spaces.arg = this;
  probe->addresses.funcs.decode = cbDecodeAddress;
  probe->addresses.arg = this;
}

bool I2cModel::cbDecodeAddressSpace(pb_istream_t *stream,
                                    const pb_field_t *field, void **arg) {
  I2cModel *model = (I2cModel *)*arg;
  if (model->_probe_spaces_count >= MAX_PROBE_SPACES)
    return false;
  ws_i2c_AddressSpace space = ws_i2c_AddressSpace_init_zero;
  if (!pb_decode(stream, ws_i2c_AddressSpace_fields, &space))
    return false;
  model->_probe_spaces[model->_probe_spaces_count] = space;
  model->_probe_spaces_count++;
  return true;
}

bool I2cModel::cbDecodeAddress(pb_istream_t *stream, const pb_field_t *field,
                               void **arg) {
  I2cModel *model = (I2cModel *)*arg;
  if (model->_probe_addresses_count >= MAX_PROBE_ADDRESSES)
    return false;
  uint32_t addr = 0;
  if (!pb_decode_varint32(stream, &addr))
    return false;
  model->_probe_addresses[model->_probe_addresses_count] = addr;
  model->_probe_addresses_count++;
  return true;
}

ws_i2c_AddressSpace *I2cModel::GetProbeAddressSpaces() {
  return _probe_spaces;
}

size_t I2cModel::GetProbeAddressSpacesCount() { return _probe_spaces_count; }

uint32_t *I2cModel::GetProbeAddresses() { return _probe_addresses; }

size_t I2cModel::GetProbeAddressesCount() { return _probe_addresses_count; }

void I2cModel::ClearProbed() {
  memset(&_msg_probed, 0, sizeof(_msg_probed));
  for (size_t i = 0; i < MAX_PROBE_SPACES; i++) {
    _found_ctx[i].count = 0;
  }
}

ws_i2c_AddressSpaceResult *I2cModel::GetNextProbedResult() {
  if (_msg_probed.results_count >= MAX_PROBE_SPACES)
    return nullptr;
  size_t idx = _msg_probed.results_count;
  _msg_probed.results_count++;
  return &_msg_probed.results[idx];
}

uint32_t *I2cModel::GetFoundAddressBuf(size_t idx) {
  if (idx >= MAX_PROBE_SPACES)
    return nullptr;
  return _found_ctx[idx].addresses;
}

size_t *I2cModel::GetFoundAddressCount(size_t idx) {
  if (idx >= MAX_PROBE_SPACES)
    return nullptr;
  return &_found_ctx[idx].count;
}

bool I2cModel::cbEncodeFoundAddresses(pb_ostream_t *stream,
                                      const pb_field_t *field,
                                      void *const *arg) {
  FoundAddressesCtx *ctx = (FoundAddressesCtx *)*arg;
  for (size_t i = 0; i < ctx->count; i++) {
    if (!pb_encode_tag_for_field(stream, field))
      return false;
    if (!pb_encode_varint(stream, ctx->addresses[i]))
      return false;
  }
  return true;
}

/*!
    @brief    Encodes the Probed message for publishing.
    @returns  True if encoding succeeded, False otherwise.
*/
bool I2cModel::EncodeProbed() {
  // Set up encode callbacks for found_addresses on each result
  for (pb_size_t i = 0; i < _msg_probed.results_count; i++) {
    _msg_probed.results[i].found_addresses.funcs.encode =
        cbEncodeFoundAddresses;
    _msg_probed.results[i].found_addresses.arg = &_found_ctx[i];
  }

  // Wrap in D2B envelope
  memset(&_msg_i2c_d2b, 0, sizeof(_msg_i2c_d2b));
  _msg_i2c_d2b.which_payload = ws_i2c_D2B_probed_tag;
  _msg_i2c_d2b.payload.probed = _msg_probed;

  // Verify we can get the encoded size
  size_t sz_msg;
  if (!pb_get_encoded_size(&sz_msg, ws_i2c_D2B_fields, &_msg_i2c_d2b))
    return false;

  return true;
}

/*!
    @brief    Returns a pointer to the I2cD2B message.
    @returns  Pointer to the I2cD2B message.
*/
ws_i2c_D2B *I2cModel::GetI2cD2B() { return &_msg_i2c_d2b; }

/*!
    @brief    Decodes a I2cDeviceAddReplace message from an input stream.
    @param   stream
                A pointer to the pb_istream_t stream.
    @returns  True if the stream was decoded successfully, False otherwise.
*/
bool I2cModel::DecodeI2cDeviceAddReplace(pb_istream_t *stream) {
  memset(&_msg_i2c_add, 0, sizeof(_msg_i2c_add));
  return pb_decode(stream, ws_i2c_Add_fields,
                   &_msg_i2c_add);
}

/*!
    @brief    Returns a pointer to the I2cDeviceAddOrReplace message.
    @returns  Pointer to the I2cDeviceAddOrReplace message.
*/
ws_i2c_Add *I2cModel::GetI2cDeviceAddOrReplaceMsg() {
  return &_msg_i2c_add;
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
    ws_i2c_Descriptor device_descriptor, ws_i2c_BusStatus bus_status,
    ws_i2c_Status device_status) {
  size_t sz_msg;

  // Fill I2cDeviceAddedOrReplaced message
  memset(&_msg_i2c_added, 0,
         sizeof(_msg_i2c_added));
  _msg_i2c_added.has_descriptor = true;
  _msg_i2c_added.descriptor = device_descriptor;
  _msg_i2c_added.bus_status = bus_status;
  _msg_i2c_added.status = device_status;

  // Encode message
  if (!pb_get_encoded_size(&sz_msg, ws_i2c_Added_fields,
                           &_msg_i2c_added))
    return false;

  uint8_t buf[sz_msg];
  pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
  return pb_encode(&msg_stream, ws_i2c_Added_fields,
                   &_msg_i2c_added);
}

/*!
    @brief    Returns a pointer to the I2cDeviceAddedOrReplaced message.
    @returns  Pointer to the I2cDeviceAddedOrReplaced message.
*/
ws_i2c_Added *I2cModel::GetMsgI2cDeviceAddedOrReplaced() {
  return &_msg_i2c_added;
}

/*!
    @brief    Clears the I2cDeviceEvent message.
*/
void I2cModel::ClearI2cDeviceEvent() {
  memset(&_msg_i2c_event, 0, sizeof(_msg_i2c_event));
  _msg_i2c_event.events_count = 0;
}

/*!
    @brief    Sets the I2cDeviceEvent message's device description.
    @param    pin_scl
                The SCL pin number.
    @param    pin_sda
                The SDA pin number.
    @param    addr_device
                The device address.
    @param    addr_mux
                The MUX address.
    @param    mux_channel
                The MUX channel.
*/
void I2cModel::SetI2cDeviceEventDeviceDescripton(uint32_t pin_scl,
                                                 uint32_t pin_sda,
                                                 uint32_t addr_device,
                                                 uint32_t addr_mux,
                                                 uint32_t mux_channel) {
  _msg_i2c_event.has_descriptor = true;
  _msg_i2c_event.descriptor.has_address_space = true;
  _msg_i2c_event.descriptor.address_space.pin_scl = pin_scl;
  _msg_i2c_event.descriptor.address_space.pin_sda = pin_sda;
  _msg_i2c_event.descriptor.address = addr_device;
  _msg_i2c_event.descriptor.address_space.mux_address = addr_mux;
  _msg_i2c_event.descriptor.address_space.mux_channel = mux_channel;
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
                                       ws_i2c_Add_TypesEntry type_entry) {
  if (_msg_i2c_event.events_count >= MAX_DEVICE_EVENTS)
    return false; // Maximum amount of events reached

  pb_size_t idx = _msg_i2c_event.events_count;
  _msg_i2c_event.events[idx].key = type_entry.key;
  _msg_i2c_event.events[idx].has_value = true;
  _msg_i2c_event.events[idx].value.type = type_entry.value;

  float value = GetValueFromSensorsEvent(type_entry.value, &event);
  _msg_i2c_event.events[idx].value.which_value =
      ws_sensor_Event_float_value_tag;
  _msg_i2c_event.events[idx].value.value.float_value = value;

  _msg_i2c_event.events_count++;
  return true;
}

/*!
    @brief    Encodes an I2cDeviceEvent message.
    @returns  True if the message was encoded successfully, False otherwise.
*/
bool I2cModel::EncodeI2cDeviceEvent() {
  size_t sz_msg;
  if (!pb_get_encoded_size(&sz_msg, ws_i2c_Event_fields,
                           &_msg_i2c_event))
    return false;

  uint8_t buf[sz_msg];
  pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
  return pb_encode(&msg_stream, ws_i2c_Event_fields,
                   &_msg_i2c_event);
}

/*!
    @brief    Returns a pointer to the I2cDeviceEvent message.
    @returns  Pointer to the I2cDeviceEvent message.
*/
ws_i2c_Event *I2cModel::GetI2cDeviceEvent() {
  return &_msg_i2c_event;
}
