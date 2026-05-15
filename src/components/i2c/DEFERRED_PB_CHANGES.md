# Deferred I2C Protobuf Changes

These changes are part of the `update-pb-api` protobuf regeneration but require
separate design work beyond mechanical renaming.

## 1. Scan -> Probe Redesign

Old `ws_i2c_Scan` (flat struct: pin_scl, pin_sda, mux_address) becomes
`ws_i2c_Probe` with `pb_callback_t` fields for `address_spaces` and `addresses`.
Response changes from `ws_i2c_Scanned` (static array of DeviceDescriptor) to
`ws_i2c_Probed` (array of `AddressSpaceResult`, each with its own bus_status and
callback-based `found_addresses`).

Requires a new architectural pattern: targeted probing of specific addresses
across multiple address spaces, rather than brute-force scanning all 112
addresses on one bus.

**Files affected:** model.h, model.cpp, controller.h, controller.cpp,
hardware.h, hardware.cpp

## 2. Sensor Types: Flat Enum Array -> Map Entries

Old `device_sensor_types` was `ws_sensor_Type[16]` (flat repeated enum).
New `types` is `ws_i2c_Add_TypesEntry[16]` where each entry is
`{uint32_t key, ws_sensor_Type value}`.

Old `device_events` was `ws_sensor_Event[16]` (flat repeated message).
New `events` is `ws_i2c_Event_EventsEntry[16]` where each entry is
`{uint32_t key, optional ws_sensor_Event value}`.

Changes how the driver layer interacts with sensor config and event publishing.

**Files affected:** model.h, model.cpp, controller.cpp (EnableSensorReads,
AddI2cDeviceSensorEvent), ws_sdcard.cpp

## 3. DeviceDescriptor -> AddressSpace + Descriptor Split

Old flat `ws_i2c_DeviceDescriptor` (scl, sda, device_address, mux_address,
mux_channel) splits into:
- `ws_i2c_AddressSpace` (pin_scl, pin_sda, mux_address, mux_channel)
- `ws_i2c_Descriptor` (address_space + address)

`device_address` moves from being a sibling of pin_scl/pin_sda into
`descriptor.address`, while pin info lives at `descriptor.address_space.pin_scl`.

**Files affected:** model.h, model.cpp, controller.h, controller.cpp,
gps/controller.cpp, error/controller.h, error/controller.cpp,
sdcard/ws_sdcard.h, sdcard/ws_sdcard.cpp
