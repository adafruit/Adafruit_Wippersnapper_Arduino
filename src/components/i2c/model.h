/*!
 * @file src/components/i2c/model.h
 *
 * Provides high-level interfaces for messages within i2c.proto and
 * display.proto (for I2C output devices like OLED, LED backpack, char LCD).
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
#include "wippersnapper.h"
#include <Adafruit_Sensor.h>
#include <protos/display.pb.h>

#define MAX_DEVICE_EVENTS    16 ///< Maximum number of SensorEvents within I2cDeviceEvent
#define MAX_I2C_SCAN_DEVICES 96 ///< Maximum number of devices found on the bus

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
      ws_i2c_BusStatus i2c_bus_status, ws_i2c_DeviceStatus i2c_device_status);
  bool EncodeI2cDeviceEvent();
  // Getters
  ws_i2c_DeviceRemove *GetI2cDeviceRemoveMsg();
  ws_i2c_DeviceAddOrReplace *GetI2cDeviceAddOrReplaceMsg();
  ws_display_Add *GetDisplayAddMsg();
  ws_i2c_DeviceAddedOrReplaced *GetMsgI2cDeviceAddedOrReplaced();
  ws_i2c_DeviceEvent *GetI2cDeviceEvent();
  ws_i2c_Scan *GetI2cBusScanMsg();
  ws_i2c_Scanned *GetI2cBusScannedMsg();
  // I2cBusScanned Message API
  void ClearI2cBusScanned();
  bool AddDeviceToBusScan(uint32_t pin_scl, uint32_t pin_sda,
                          uint32_t addr_device, uint32_t addr_mux,
                          uint32_t mux_channel);
  void setI2cBusScannedStatus(ws_i2c_BusStatus bus_status);
  bool encodeI2cScanned();
  ws_i2c_D2B *GetI2cD2B();
  // DeviceEvent Message API
  void ClearI2cDeviceEvent();
  void SetI2cDeviceEventDeviceDescripton(uint32_t pin_scl,
                                         uint32_t pin_sda,
                                         uint32_t addr_device,
                                         uint32_t addr_mux,
                                         uint32_t mux_channel);
  bool AddI2cDeviceSensorEvent(sensors_event_t &event,
                               ws_sensor_Type sensor_type);

private:
  ws_i2c_D2B _msg_i2c_d2b;
  ws_i2c_Scan _msg_i2c_bus_scan;
  ws_i2c_Scanned _msg_i2c_bus_scanned;
  ws_i2c_DeviceAddOrReplace _msg_i2c_device_add_replace;
  ws_i2c_DeviceAddedOrReplaced _msg_i2c_device_added_replaced;
  ws_i2c_DeviceRemove _msg_i2c_device_remove;
  ws_i2c_DeviceRemoved _msg_i2c_device_removed;
  ws_i2c_DeviceEvent _msg_i2c_device_event;
};

/*!
    @brief  Provides an interface for creating, encoding, and parsing
            display write messages for I2C output devices.
*/
class I2cOutputModel {
public:
  I2cOutputModel();
  ~I2cOutputModel();
  // Decoders
  bool DecodeDisplayWrite(pb_istream_t *stream);
  // Getters
  ws_display_Write *GetDisplayWriteMsg();

private:
  ws_display_Write _msg_display_write;
};
#endif // WS_I2C_MODEL_H