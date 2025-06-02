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
  memset(&_msg_UartInputEvent, 0, sizeof(_msg_UartInputEvent));
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
  memset(&_msg_UartInputEvent, 0, sizeof(_msg_UartInputEvent));
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
    @param  uart_nbr
            The UART port number (eg: 0, 1, 2, etc.) that the device was attached to.
    @param  type
            The category of device attached to the UART port, corresponds to its driver type.
    @param  id
            The unique identifier string for the UART device.
    @param  success
            True if the device on the UART port was successfully initialized, False otherwise.
    @return True if the message was encoded successfully, False otherwise.
*/
bool UARTModel::EncodeUartAdded(int32_t uart_nbr, wippersnapper_uart_UartDeviceType type, const char *id, bool success) {
    _msg_UartAdded.uart_nbr = uart_nbr;
    _msg_UartAdded.type = type;
    strncpy(_msg_UartAdded.device_id, id, sizeof(_msg_UartAdded.device_id) - 1);
    _msg_UartAdded.device_id[sizeof(_msg_UartAdded.device_id) - 1] = '\0';
    _msg_UartAdded.success = success;
    // Calculate the size of the encoded message
    size_t sz_msg;
    if (!pb_get_encoded_size(&sz_msg, wippersnapper_uart_UartAdded_fields,
                            &_msg_UartAdded))
        return false;
    // Attempt to encode the message into a buffer
    uint8_t buf[sz_msg];
    pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
    return pb_encode(&msg_stream, wippersnapper_uart_UartAdded_fields, &_msg_UartAdded);
}

/*!
    @brief  Gets a pointer to the encoded UartAdded message.
    @return Pointer to the encoded UartAdded message.
*/
wippersnapper_uart_UartAdded *UARTModel::GetUartAddedMsg() {
  return &_msg_UartAdded;
}

/*!
    @brief  Decodes a UartDeviceRemove message from an input stream.
    @param  stream
            Pointer to a pb_istream_t object.
    @return True if the UartDeviceRemove message was decoded successfully,
            False otherwise.
*/
bool UARTModel::DecodeUartDeviceRemove(pb_istream_t *stream) {
  memset(&_msg_UartRemove, 0, sizeof(_msg_UartRemove));
  return pb_decode(stream, wippersnapper_uart_UartRemove_fields, &_msg_UartRemove);
}

/*!
    @brief  Gets a pointer to the UartRemove message.
    @return Pointer to the UartRemove message.
*/
wippersnapper_uart_UartRemove *UARTModel::GetUartRemoveMsg() {
  return &_msg_UartRemove;
}

/*!
    @brief  Clears an UartInputEvent message.
*/
void UARTModel::ClearUartInputEventMsg() {
    memset(&_msg_UartInputEvent, 0, sizeof(_msg_UartInputEvent));
}

/*!
    @brief  Configures an UartInputEvent message with addressing information.
    @param  uart_nbr
            The UART port number (eg: 0, 1, 2, etc.) that the device is attached to.
    @param  type
            The category of device attached to the UART port, corresponds to its driver type.
    @param  device_id
            The unique identifier string for the UART device.
*/
void UARTModel::ConfigureUartInputEventMsg(uint32_t uart_nbr, wippersnapper_uart_UartDeviceType type, const char *device_id) {
    // Addressing information
    _msg_UartInputEvent.uart_nbr = uart_nbr;
    _msg_UartInputEvent.type = type;
    strncpy(_msg_UartInputEvent.device_id, device_id, sizeof(_msg_UartInputEvent.device_id) - 1);
    _msg_UartInputEvent.device_id[sizeof(_msg_UartInputEvent.device_id) - 1] = '\0';

}

/*!
    @brief  Adds a UART input event to the UartInputEvent message.
    @param  event
            Reference to a sensors_event_t structure containing sensor data.
    @param  sensor_type
            The type of sensor that produced the event.
    @return True if the event was added successfully, False otherwise.
*/
bool UARTModel::AddUartInputEvent(sensors_event_t &event, wippersnapper_sensor_SensorType sensor_type) {
    if (_msg_UartInputEvent.events_count >= MAX_UART_INPUT_EVENTS) {
        return false; // Maximum number of events reached
    }

    // Configure the sensor event
    wippersnapper_sensor_SensorEvent &sensor_event = _msg_UartInputEvent.events[_msg_UartInputEvent.events_count];
    sensor_event.type = sensor_type;
    
    // Handle different sensor types
    switch (sensor_type) {
        case wippersnapper_sensor_SensorType_SENSOR_TYPE_PM10_STD:
            sensor_event.which_value = wippersnapper_sensor_SensorEvent_float_value_tag;
            sensor_event.value.float_value = event.pm10_std;
            break;
        case wippersnapper_sensor_SensorType_SENSOR_TYPE_PM25_STD:
            sensor_event.which_value = wippersnapper_sensor_SensorEvent_float_value_tag;
            sensor_event.value.float_value = event.pm25_std;
            break;
        case wippersnapper_sensor_SensorType_SENSOR_TYPE_PM100_STD:
            sensor_event.which_value = wippersnapper_sensor_SensorEvent_float_value_tag;
            sensor_event.value.float_value = event.pm100_std;
            break;
        // TODO: Add more cases for other sensor types
        default:
            sensor_event.which_value = wippersnapper_sensor_SensorEvent_float_value_tag;
            sensor_event.value.float_value = event.data[0];
            break;
    }

    _msg_UartInputEvent.events_count++;
    return true;
}

/*!
    @brief  Encodes a UartInputEvent message.
    @return True if the message was encoded successfully, False otherwise.
*/
bool UARTModel::EncodeUartInputEvent() {
    // Calculate the size of the encoded message
    size_t sz_msg;
    if (!pb_get_encoded_size(&sz_msg, wippersnapper_uart_UartInputEvent_fields,
                            &_msg_UartInputEvent))
        return false;
    
    // Attempt to encode the message into a buffer
    uint8_t buf[sz_msg];
    pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
    return pb_encode(&msg_stream, wippersnapper_uart_UartInputEvent_fields, &_msg_UartInputEvent);
}

/*!
    @brief  Gets a pointer to the UartInputEvent message.
    @return Pointer to the UartInputEvent message.
*/
wippersnapper_uart_UartInputEvent *UARTModel::GetUartInputEventMsg() {
    return &_msg_UartInputEvent;
}