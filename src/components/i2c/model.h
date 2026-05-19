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
#include "wippersnapper.h"
#include <Adafruit_Sensor.h>
#include <protos/i2c_output.pb.h>

#define MAX_DEVICE_EVENTS      16 ///< Maximum number of SensorEvents within I2cEvent
#define MAX_PROBE_SPACES       16 ///< Maximum number of AddressSpaces in a Probe
#define MAX_PROBE_ADDRESSES   112 ///< Maximum number of addresses to probe

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
  bool DecodeI2cDeviceOutputWrite(pb_istream_t *stream);
  // Encoders
  bool EncodeI2cDeviceEvent();
  // Getters
  ws_i2c_Remove *GetI2cDeviceRemoveMsg();
  ws_i2c_Add *GetI2cDeviceAddOrReplaceMsg();
  ws_i2c_output_Add *GetI2cOutputAddMsg();
  ws_i2c_Event *GetI2cDeviceEvent();
  ws_i2c_D2B *GetI2cD2B();
  // Probe API — model owns decode/encode/storage
  void SetupProbeDecodeCallbacks(ws_i2c_Probe *probe);
  ws_i2c_AddressSpace *GetProbeAddressSpaces();
  size_t GetProbeAddressSpacesCount();
  uint32_t *GetProbeAddresses();
  size_t GetProbeAddressesCount();
  void ClearProbed();
  ws_i2c_AddressSpaceResult *GetNextProbedResult();
  uint32_t *GetFoundAddressBuf(size_t idx);
  size_t *GetFoundAddressCount(size_t idx);
  bool EncodeProbed();
  // DeviceEvent Message API
  void ClearI2cDeviceEvent();
  void SetI2cDeviceEventDeviceDescripton(uint32_t pin_scl,
                                         uint32_t pin_sda,
                                         uint32_t addr_device,
                                         uint32_t addr_mux,
                                         uint32_t mux_channel);
  bool AddI2cDeviceSensorEvent(sensors_event_t &event,
                               ws_i2c_Add_TypesEntry type_entry);

private:
  // Probe decode buffers
  ws_i2c_AddressSpace _probe_spaces[MAX_PROBE_SPACES];
  size_t _probe_spaces_count;
  uint32_t _probe_addresses[MAX_PROBE_ADDRESSES];
  size_t _probe_addresses_count;
  // Probe results
  ws_i2c_Probed _msg_probed;
  struct FoundAddressesCtx {
    uint32_t addresses[MAX_PROBE_ADDRESSES];
    size_t count;
  };
  FoundAddressesCtx _found_ctx[MAX_PROBE_SPACES];
  // Probe nanopb callbacks
  static bool cbDecodeAddressSpace(pb_istream_t *stream,
                                   const pb_field_t *field, void **arg);
  static bool cbDecodeAddress(pb_istream_t *stream, const pb_field_t *field,
                              void **arg);
  static bool cbEncodeFoundAddresses(pb_ostream_t *stream,
                                     const pb_field_t *field,
                                     void *const *arg);
  // Message storage
  ws_i2c_D2B _msg_i2c_d2b;
  ws_i2c_Add _msg_i2c_add;
  ws_i2c_Remove _msg_i2c_remove;
  ws_i2c_Event _msg_i2c_event;
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