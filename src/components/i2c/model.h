/*!
 * @file model.h
 *
 * Provides high-level interfaces for messages within i2c.proto.
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

/**************************************************************************/
/*!
    @brief  Provides an interface for creating, encoding, and parsing
            messages from i2c.proto.
*/
/**************************************************************************/
class I2cModel {
public:
  I2cModel();
  ~I2cModel();
  bool DecodeI2cDeviceAddReplace(pb_istream_t *stream);
  wippersnapper_i2c_I2cDeviceRemove *GetI2cDeviceRemoveMsg();
  bool DecodeI2cDeviceRemove(pb_istream_t *stream);
  wippersnapper_i2c_I2cDeviceAddOrReplace *GetI2cDeviceAddOrReplaceMsg();
  bool encodeMsgI2cDeviceAddedorReplaced(
      wippersnapper_i2c_I2cDeviceDescriptor i2c_device_description,
      wippersnapper_i2c_I2cBusStatus i2c_bus_status,
      wippersnapper_i2c_I2cDeviceStatus i2c_device_status);
  wippersnapper_i2c_I2cDeviceAddedOrReplaced *GetMsgI2cDeviceAddedOrReplaced();
  // Device Event Message API
  void ClearI2cDeviceEvent();
  void SetI2cDeviceEventDeviceDescripton(const char *bus_scl,
                                         const char *bus_sda,
                                         uint32_t addr_device,
                                         uint32_t addr_mux,
                                         uint32_t mux_channel);

private:
  wippersnapper_i2c_I2cBusScan _msg_i2c_bus_scan;
  wippersnapper_i2c_I2cBusScanned _msg_i2c_bus_scanned;
  wippersnapper_i2c_I2cDeviceAddOrReplace _msg_i2c_device_add_replace;
  wippersnapper_i2c_I2cDeviceAddedOrReplaced _msg_i2c_device_added_replaced;
  wippersnapper_i2c_I2cDeviceRemove _msg_i2c_device_remove;
  wippersnapper_i2c_I2cDeviceRemoved _msg_i2c_device_removed;
  wippersnapper_i2c_I2cDeviceEvent _msg_i2c_device_event;
};
#endif // WS_I2C_MODEL_H