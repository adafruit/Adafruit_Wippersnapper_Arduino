/*!
 * @file src/components/i2c/model.h
 *
 * Provides high-level interfaces for messages within i2c.proto and
 * i2c_output.proto.
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
#ifndef WS_I2C_MODEL_H
#define WS_I2C_MODEL_H
#include "Wippersnapper_V2.h"
#include <Adafruit_Sensor.h>
#include <protos/i2c_output.pb.h>
#define MAX_DEVICE_EVENTS                                                      \
  15 ///< Maximum number of SensorEvents within I2cDeviceEvent
#define MAX_I2C_SCAN_DEVICES 120 ///< Maximum number of devices found on the bus

/*!
    @brief  Provides an interface for creating, encoding, and parsing
            messages from i2c.proto.
*/
class I2cModel {
public:
  I2cModel();
  ~I2cModel();
  // Decoders
  bool DecodeI2cDeviceAddReplace(pb_istream_t *stream);
  bool DecodeI2cDeviceRemove(pb_istream_t *stream);
  bool DecodeI2cBusScan(pb_istream_t *stream);
  bool DecodeI2cDeviceOutputWrite(pb_istream_t *stream);
  // Encoders
  bool encodeMsgI2cDeviceAddedorReplaced(
      ws_i2c_DeviceDescriptor i2c_device_description,
      ws_i2c_BusStatus i2c_bus_status,
      ws_i2c_DeviceStatus i2c_device_status);
  bool EncodeI2cDeviceEvent();
  // Getters
  ws_i2c_DeviceRemove *GetI2cDeviceRemoveMsg();
  ws_i2c_DeviceAddedOrReplaced *GetI2cDeviceAddOrReplaceMsg();
  ws_i2c_output_Add *GetI2cOutputAddMsg();
  ws_i2c_DeviceAddedOrReplaced *GetMsgI2cDeviceAddedOrReplaced();
  ws_i2c_DeviceEvent *GetI2cDeviceEvent();
  ws_i2c_BusScan *GetI2cBusScanMsg();
  ws_i2c_BusScanned *GetI2cBusScannedMsg();
  ws_i2c_DeviceOutputWrite *GetI2cDeviceOutputWriteMsg();
  // I2cBusScanned Message API
  void ClearI2cBusScanned();
  bool AddDeviceToBusScan(const char *bus_scl, const char *bus_sda,
                          uint32_t addr_device, uint32_t addr_mux,
                          uint32_t mux_channel);
  // DeviceEvent Message API
  void ClearI2cDeviceEvent();
  void SetI2cDeviceEventDeviceDescripton(const char *bus_scl,
                                         const char *bus_sda,
                                         uint32_t addr_device,
                                         uint32_t addr_mux,
                                         uint32_t mux_channel);
  bool AddI2cDeviceSensorEvent(sensors_event_t &event,
                               ws_sensor_Type sensor_type);

private:
  ws_i2c_BusScan _msg_i2c_bus_scan;
  ws_i2c_BusScanned _msg_i2c_bus_scanned;
  ws_i2c_DeviceAddedOrReplaced _msg_i2c_device_add_replace;
  ws_i2c_DeviceAddedOrReplaced _msg_i2c_device_added_replaced;
  ws_i2c_DeviceRemove _msg_i2c_device_remove;
  ws_i2c_DeviceRemoved _msg_i2c_device_removed;
  ws_i2c_DeviceEvent _msg_i2c_device_event;
  ws_i2c_DeviceOutputWrite _msg_i2c_device_output_write;
};

/*!
    @brief  Provides an interface for creating, encoding, and parsing
            messages from i2c_output.proto.
*/
class I2cOutputModel {
public:
  I2cOutputModel();
  ~I2cOutputModel();
  // Decoders
  bool DecodeLedBackpackWrite(pb_istream_t *stream);
  bool DecodeCharLCDWrite(pb_istream_t *stream);
  // Getters
  ws_i2c_output_LedBackpackWrite *GetLedBackpackWriteMsg();
  ws_i2c_output_CharLCDWrite *GetCharLCDWriteMsg();

private:
  ws_i2c_output_LedBackpackWrite _msg_led_backpack_write;
  ws_i2c_output_CharLCDWrite _msg_char_lcd_write;
};
#endif // WS_I2C_MODEL_H