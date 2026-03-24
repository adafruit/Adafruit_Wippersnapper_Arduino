/*!
 * @file src/components/i2c/controller.cpp
 *
 * Controller for the i2c.proto API
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
#include "drivers/drvBase.h"

/*!
    @brief     Lambda function to create a drvBase driver instance
      @param    i2c
                  The desired I2C interface.
      @param    addr
                  The desired i2c device address.
      @param    mux_channel
                  The desired I2C multiplexer channel.
      @param    driver_name
                  The i2c driver's name.
*/
using FnCreateI2CSensorDriver =
    std::function<drvBase *(TwoWire *, uint16_t, uint32_t, const char *)>;

// Factory for creating a new I2C SENSOR drivers
// NOTE: When you add a new SENSOR driver, make sure to add it to the factory!
static const std::map<std::string, FnCreateI2CSensorDriver> I2cFactorySensor = {
    {"sensor_mock",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvGenericSensorMock(i2c, addr, mux_channel, driver_name);
     }},
    {"bme280",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvBme280(i2c, addr, mux_channel, driver_name);
     }},
    {"adt7410",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvAdt7410(i2c, addr, mux_channel, driver_name);
     }},
    {"aht20",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvAhtx0(i2c, addr, mux_channel, driver_name);
     }},
    {"am2301b",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvAhtx0(i2c, addr, mux_channel, driver_name);
     }},
    {"am2315c",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvAhtx0(i2c, addr, mux_channel, driver_name);
     }},
    {"dht20",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvAhtx0(i2c, addr, mux_channel, driver_name);
     }},
    {"bh1750",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvBh1750(i2c, addr, mux_channel, driver_name);
     }},
    {"bme680",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvBme680(i2c, addr, mux_channel, driver_name);
     }},
    {"bme688",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvBme680(i2c, addr, mux_channel, driver_name);
     }},
    {"BMP280",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvBmp3xx(i2c, addr, mux_channel, driver_name);
     }},
    {"bmp388",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvBmp3xx(i2c, addr, mux_channel, driver_name);
     }},
    {"bmp390",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvBmp3xx(i2c, addr, mux_channel, driver_name);
     }},
    {"bmp280",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvBmp280(i2c, addr, mux_channel, driver_name);
     }},
    {"dps310",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvDps310(i2c, addr, mux_channel, driver_name);
     }},
    {"ds2484",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvDs2484(i2c, addr, mux_channel, driver_name);
     }},
    {"ens160",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvEns160(i2c, addr, mux_channel, driver_name);
     }},
    {"hts221",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvHts221(i2c, addr, mux_channel, driver_name);
     }},
    {"htu21d",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvHtu21d(i2c, addr, mux_channel, driver_name);
     }},
    {"ina219",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvIna219(i2c, addr, mux_channel, driver_name);
     }},
    {"lc709203f",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvLc709203f(i2c, addr, mux_channel, driver_name);
     }},
    {"lps3xhw",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvLps3xhw(i2c, addr, mux_channel, driver_name);
     }},
    {"lps22hb",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvLps22hb(i2c, addr, mux_channel, driver_name);
     }},
    {"lps25hb",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvLps25hb(i2c, addr, mux_channel, driver_name);
     }},
    {"ltr329",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvLtr329_Ltr303(i2c, addr, mux_channel, driver_name);
     }},
    {"ltr303",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvLtr329_Ltr303(i2c, addr, mux_channel, driver_name);
     }},
    {"ltr390",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvLtr390(i2c, addr, mux_channel, driver_name);
     }},
    {"max17048",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvMax1704x(i2c, addr, mux_channel, driver_name);
     }},
    {"mcp3421",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvMax1704x(i2c, addr, mux_channel, driver_name);
     }},
    {"mcp9808",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvMcp9808(i2c, addr, mux_channel, driver_name);
     }},
    {"mpl115a2",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvMpl115a2(i2c, addr, mux_channel, driver_name);
     }},
    {"mprls",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvMprls(i2c, addr, mux_channel, driver_name);
     }},
    {"ms8607",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvMs8607(i2c, addr, mux_channel, driver_name);
     }},
    {"nau7802",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvNau7802(i2c, addr, mux_channel, driver_name);
     }},
    {"pct2075",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvPct2075(i2c, addr, mux_channel, driver_name);
     }},
    {"pmsa003i",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvPm25(i2c, addr, mux_channel, driver_name);
     }},
    {"scd40",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvScd4x(i2c, addr, mux_channel, driver_name);
     }},
    {"scd41",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvScd4x(i2c, addr, mux_channel, driver_name);
     }},
    {"scd30",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvScd30(i2c, addr, mux_channel, driver_name);
     }},
    {"sgp40",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvSgp40(i2c, addr, mux_channel, driver_name);
     }},
    {"sht3x",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvSht3x(i2c, addr, mux_channel, driver_name);
     }},
    {"sht30_shell",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvSht3x(i2c, addr, mux_channel, driver_name);
     }},
    {"sht30_mesh",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvSht3x(i2c, addr, mux_channel, driver_name);
     }},
    {"sht40",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvSht4x(i2c, addr, mux_channel, driver_name);
     }},
    {"sht41",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvSht4x(i2c, addr, mux_channel, driver_name);
     }},
    {"sht45",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvSht4x(i2c, addr, mux_channel, driver_name);
     }},
    {"sen5x",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvSen5x(i2c, addr, mux_channel, driver_name);
     }},
    {"sen55",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvSen5x(i2c, addr, mux_channel, driver_name);
     }},
    {"sen54",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvSen5x(i2c, addr, mux_channel, driver_name);
     }},
    {"sen50",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvSen5x(i2c, addr, mux_channel, driver_name);
     }},
    {"shtc3",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvShtc3(i2c, addr, mux_channel, driver_name);
     }},
    {"si7021",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvSi7021(i2c, addr, mux_channel, driver_name);
     }},
    {"stemma_soil",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvSen5x(i2c, addr, mux_channel, driver_name);
     }},
    {"tmp117",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvTmp117(i2c, addr, mux_channel, driver_name);
     }},
    {"tsl2591",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvTsl2591(i2c, addr, mux_channel, driver_name);
     }},
    {"vncl4020",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvVncl4020(i2c, addr, mux_channel, driver_name);
     }},
    {"veml7700",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvVeml7700(i2c, addr, mux_channel, driver_name);
     }},
    {"vncl4040",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvVncl4040(i2c, addr, mux_channel, driver_name);
     }},
    {"vl53l0x",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvVl53l0x(i2c, addr, mux_channel, driver_name);
     }},
    {"vl53l1x",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvVl53l1x(i2c, addr, mux_channel, driver_name);
     }},
    {"vl53l4cd",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvVl53l4cd(i2c, addr, mux_channel, driver_name);
     }},
    {"vl53l4cx",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvVl53l4cx(i2c, addr, mux_channel, driver_name);
     }},
    {"vl6180x",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvVl6180x(i2c, addr, mux_channel, driver_name);
     }}}; ///< I2C driver factory

/*!
    @brief  Creates an I2C driver by name
    @param    driver_name
                The name of the I2C driver.
    @param    i2c
                The I2C bus.
    @param    addr
                The I2C device address.
    @param    i2c_mux_channel
                The I2C MUX channel.
    @param    status
                The I2cDeviceStatus message.
    @returns  A pointer to the I2C driver.
*/
drvBase *CreateI2cSensorDrv(const char *driver_name, TwoWire *i2c,
                            uint16_t addr, uint32_t i2c_mux_channel,
                            ws_i2c_DeviceStatus &status) {
  auto it = I2cFactorySensor.find(driver_name);
  if (it == I2cFactorySensor.end()) {
    status = ws_i2c_DeviceStatus_DS_FAIL_UNSUPPORTED_SENSOR;
    return nullptr;
  }

  status = ws_i2c_DeviceStatus_DS_SUCCESS;
  return it->second(i2c, addr, i2c_mux_channel, driver_name);
}

/*!
    @brief  I2cController constructor
*/
I2cController::I2cController() {
  _i2c_model = new I2cModel();
}

/*!
    @brief  I2cController destructor
*/
I2cController::~I2cController() {
  if (_i2c_model)
    delete _i2c_model;
}


/*!
    @brief  Routes messages using the i2c.proto API to the
            appropriate controller functions.
    @param  stream
            The nanopb input stream.
    @return True if the message was successfully routed, False otherwise.
*/
bool I2cController::Router(pb_istream_t *stream) {
  // Attempt to decode the I2C B2D envelope
  ws_i2c_B2D b2d = ws_i2c_B2D_init_zero;
  if (!ws_pb_decode(stream, ws_i2c_B2D_fields, &b2d)) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: Unable to decode I2C B2D envelope");
    return false;
  }

  // Route based on payload type
  bool res = false;
  switch (b2d.which_payload) {
  case ws_i2c_B2D_bus_scan_tag:
    res = Handle_I2cBusScan(&b2d.payload.bus_scan);
    break;
  case ws_i2c_B2D_device_add_replace_tag:
    res = Handle_I2cDeviceAddOrReplace(&b2d.payload.device_add_replace);
    break;
  case ws_i2c_B2D_device_remove_tag:
    res = Handle_I2cDeviceRemove(&b2d.payload.device_remove);
    break;
  default:
    WS_DEBUG_PRINTLN("[i2c] WARNING: Unsupported I2C payload");
    res = false;
    break;
  }

  return res;
}

/*!
    @brief  Removes an I2C driver from the controller and frees memory.
    @param    address
                The desired I2C device's address.
    @param    mux_channel
                The MUX channel (use 0xFFFFFFFF to match any channel).
    @returns True if the driver was removed, False otherwise.
*/
bool I2cController::RemoveDriver(uint32_t address, uint32_t mux_channel) {
  for (drvBase *driver : _i2c_drivers) {
    if (driver == nullptr)
      continue;
    if (driver->GetAddress() != address)
      continue;
    if (mux_channel != WS_I2C_MUX_CHANNEL_ANY &&
        driver->GetMuxChannel() != mux_channel)
      continue;

    std::vector<drvBase *>::iterator it =
        std::find(_i2c_drivers.begin(), _i2c_drivers.end(), driver);
    if (it != _i2c_drivers.end()) {
      _i2c_drivers.erase(it);
    }
    delete driver;
    return true;
  }

  // We didn't find the driver to remove
  WS_DEBUG_PRINTLN("[i2c] ERROR: Unable to find driver to remove!");
  return false;
}

/*!
    @brief    Returns if the I2C bus has been created successfully.
    @param    bus
              Pointer to the I2C hardware bus to check.
    @returns  True if the I2C bus has already been created, False otherwise.
*/
bool I2cController::IsBusStatusOK(I2cHardware *bus) {
  if (bus == nullptr)
    return false;
  return (bus->GetBusStatus() == ws_i2c_BusStatus_BS_SUCCESS);
}

/*!
    @brief    Publishes an I2cDeviceAddedorReplaced message to the broker
    @param    device_descriptor
                The I2cDeviceDescriptor message.
    @param    bus
                Pointer to the I2C hardware bus.
    @param    device_status
                The I2cDeviceStatus message.
    @returns  True if the I2cDeviceAddedorReplaced message was published
              successfully, False otherwise.
*/
bool I2cController::publishDeviceAddedOrReplaced(
    const ws_i2c_DeviceDescriptor &device_descriptor,
    I2cHardware *bus,
    const ws_i2c_DeviceStatus &device_status) {
  // If we're in offline mode, don't publish out to IO
  if (Ws._sdCardV2->isModeOffline())
    return true; // Back out if we're in offline mode

  // Get bus status (use provided bus, or find it from descriptor)
  ws_i2c_BusStatus bus_status = ws_i2c_BusStatus_BS_UNSPECIFIED;
  if (bus != nullptr) {
    bus_status = bus->GetBusStatus();
  }

  // Encode the I2cDeviceAddedorReplaced message and publish it to IO
  if (!_i2c_model->encodeMsgI2cDeviceAddedorReplaced(
          device_descriptor, bus_status, device_status)) {
    WS_DEBUG_PRINTLN(
        "[i2c] ERROR: Unable to encode I2cDeviceAddedorReplaced message!");
    return false;
  }
  if (!Ws.PublishD2b(ws_signal_BrokerToDevice_i2c_tag,
                     _i2c_model->GetMsgI2cDeviceAddedOrReplaced())) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: Unable to publish I2cDeviceAddedorReplaced "
                     "message to IO!");
    return false;
  }
  return true;
}

/*!
    @brief    Publishes the ws_i2c_Scanned message to IO.
    @note     Call setI2cBusScannedStatus() on the model before calling this.
    @returns  True if published successfully, False otherwise.
*/
bool I2cController::publishScan() {
  if (Ws._sdCardV2->isModeOffline())
    return true;

  if (!_i2c_model->encodeI2cScanned()) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: Unable to encode ws_i2c_Scanned message!");
    return false;
  }

  if (!Ws.PublishD2b(ws_signal_DeviceToBroker_i2c_tag,
                     _i2c_model->GetI2cD2B())) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: Unable to publish ws_i2c_Scanned message!");
    return false;
  }
  return true;
}

/*!
    @brief    Implements handling for a I2cDeviceRemove message
    @param    msg
              Pointer to the I2cDeviceRemove message.
    @returns  True if the I2cDeviceRemove message was handled, False
              otherwise.
*/
bool I2cController::Handle_I2cDeviceRemove(ws_i2c_DeviceRemove *msg) {
  // TODO [Online]: Implement the rest of this function
  // TODO: Remember to handle removal of a mux device or a device on a mux
  if (!msg->has_device_description) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: I2cDeviceRemove message missing required "
                     "device description!");
    return false;
  }

  bool did_remove = true;

  // Find the bus for this device using pin configuration
  I2cHardware *hw_bus = findOrCreateBus(msg->device_description.pin_scl,
                                        msg->device_description.pin_sda);
  if (hw_bus == nullptr) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: Failed to find I2C bus for device removal!");
    return false;
  }

  WS_DEBUG_PRINTLN("[i2c] Removing device from bus...");
  if (!hw_bus->HasMux()) {
    if (!RemoveDriver(msg->device_description.device_address)) {
      WS_DEBUG_PRINTLN("[i2c] ERROR: Failed to remove i2c device from bus!");
      did_remove = false;
    }
  } else {
    // Bus has a I2C MUX attached
    // Case 1: Is the I2C device connected to a MUX?
    if (msg->device_description.mux_address != 0xFFFF &&
        msg->device_description.mux_channel >= 0) {
      hw_bus->SelectMuxChannel(msg->device_description.mux_channel);
      if (!RemoveDriver(msg->device_description.device_address,
                        msg->device_description.mux_channel)) {
        WS_DEBUG_PRINTLN("[i2c] ERROR: Failed to remove i2c device from bus!");
        did_remove = false;
      }
    }
    // Case 2: Is the I2C device a MUX?
    if (msg->device_description.device_address ==
        msg->device_description.mux_address) {
      ws_i2c_Scanned scan_results = ws_i2c_Scanned_init_zero;
      hw_bus->ScanMux(&scan_results);
      for (int i = 0; i < scan_results.found_devices_count; i++) {
        // Select the channel and remove the device
        hw_bus->SelectMuxChannel(scan_results.found_devices[i].mux_channel);
        RemoveDriver(scan_results.found_devices[i].device_address,
                     scan_results.found_devices[i].mux_channel);
      }
      hw_bus->RemoveMux();
    }
  }

  // Publish with did_remove to the response
  // TODO !

  return true;
}

/*!
    @brief    Attempts to initialize a MUX on the bus.
    @param    bus
                Pointer to the I2C hardware bus.
    @param    name
                The device name - checks if this is a supported MUX type.
    @param    address
                The MUX's I2C address.
    @returns  DS_SUCCESS if MUX initialized, DS_UNSPECIFIED if not a MUX,
              or failure status if MUX init failed.
*/
ws_i2c_DeviceStatus I2cController::InitMux(I2cHardware *bus, const char *name,
                                           uint32_t address) {
  // Check if this is a MUX device
  if ((strcmp(name, "pca9546") != 0) && (strcmp(name, "pca9548") != 0)) {
    return ws_i2c_DeviceStatus_DS_UNSPECIFIED;
  }

  WS_DEBUG_PRINT("[i2c] Initializing MUX driver...");
  if (bus == nullptr) {
    WS_DEBUG_PRINTLN("FAILED - bus is null!");
    return ws_i2c_DeviceStatus_DS_FAIL_UNSUPPORTED_SENSOR;
  }
  if (!bus->HasMux()) {
    if (!bus->AddMuxToBus(address, name)) {
      WS_DEBUG_PRINTLN("FAILED!");
      return ws_i2c_DeviceStatus_DS_FAIL_UNSUPPORTED_SENSOR;
    }
  }
  WS_DEBUG_PRINTLN("OK!");
  return ws_i2c_DeviceStatus_DS_SUCCESS;
}

/*!
    @brief    Initializes a GPS driver via the GPS controller.
    @param    wire
                Pointer to the TwoWire bus.
    @param    address
                The GPS device's I2C address.
    @param    config
                Pointer to the GPS configuration.
    @returns  Device status indicating success or failure.
*/
ws_i2c_DeviceStatus I2cController::InitGpsDriver(TwoWire *wire, uint16_t address,
                                                  ws_gps_Config *config) {
  WS_DEBUG_PRINT("[i2c] Creating GPS driver...");
  if (!Ws._gps_controller->AddGPS(wire, address, config)) {
    WS_DEBUG_PRINTLN("FAILED!");
    return ws_i2c_DeviceStatus_DS_FAIL_UNSUPPORTED_SENSOR;
  }
  WS_DEBUG_PRINTLN("OK!");
  return ws_i2c_DeviceStatus_DS_SUCCESS;
}

/*!
    @brief    Finds an existing I2C bus by SCL/SDA pins, or creates a new one.
    @param    pin_scl
              The SCL pin number.
    @param    pin_sda
              The SDA pin number.
    @returns  Pointer to the I2cHardware bus, or nullptr if initialization
              failed.
*/
I2cHardware *I2cController::findOrCreateBus(uint32_t pin_scl, uint32_t pin_sda) {
  // Search existing buses
  for (I2cHardware *bus : _i2c_buses) {
    if (bus == nullptr)
      continue;
    if (pin_scl == (uint32_t)bus->getSCL() &&
        pin_sda == (uint32_t)bus->getSDA()) {
      return bus;
    }
  }

  // Bus not found, create new one
  WS_DEBUG_PRINTLN("[i2c] Initializing new I2C bus...");
  I2cHardware *new_bus = new I2cHardware(pin_scl, pin_sda);
  if (!new_bus->begin()) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: Failed to initialize I2C bus!");
    delete new_bus;
    return nullptr;
  }
  _i2c_buses.push_back(new_bus);
  return new_bus;
}

/*!
    @brief   Handles an I2C bus scan request.
    @param   msg
             Pointer to the I2cBusScan message.
    @returns True if the I2C bus was successfully scanned and the
             I2cBusScan message was published to IO, False otherwise.
*/
bool I2cController::Handle_I2cBusScan(ws_i2c_Scan *msg) {
  _i2c_model->ClearI2cBusScanned();
  ws_i2c_Scanned *scan_results = _i2c_model->GetI2cBusScannedMsg();

  // Find or create the bus using shared helper
  I2cHardware *bus_to_scan = findOrCreateBus(msg->pin_scl, msg->pin_sda);
  if (bus_to_scan == nullptr) {
    // We failed to find or create the bus, publish error status and back out
    _i2c_model->setI2cBusScannedStatus(ws_i2c_BusStatus_BS_ERROR_WIRING);
    publishScan();
    return false;
  }

  // Scan the bus (with or without MUX)
  bool scan_success = true;
  if (!bus_to_scan->HasMux()) {
    WS_DEBUG_PRINTLN("[i2c] Scanning bus directly...");
    if (!bus_to_scan->ScanBus(scan_results)) {
      WS_DEBUG_PRINTLN("[i2c] ERROR: Failed to scan I2C bus!");
      scan_success = false;
    }
  } else {
    WS_DEBUG_PRINTLN("[i2c] Detected MUX on bus, scanning MUX channels...");
    if (!bus_to_scan->ScanMux(scan_results)) {
      WS_DEBUG_PRINTLN("[i2c] ERROR: Failed to scan I2C MUX on bus!");
      scan_success = false;
    }
  }

  // If the scan encountered an error, publish the error status and back out
  if (!scan_success) {
    _i2c_model->setI2cBusScannedStatus(bus_to_scan->GetBusStatus());
    publishScan();
    return false;
  }

  // Print out content of scan_results
  WS_DEBUG_PRINT("[i2c] Scan found ");
  WS_DEBUG_PRINTVAR(scan_results->found_devices_count);
  WS_DEBUG_PRINTLN(" devices.");
  for (int i = 0; i < scan_results->found_devices_count; i++) {
    WS_DEBUG_PRINTLNVAR(i);
    WS_DEBUG_PRINT("Address: ");
    WS_DEBUG_PRINTHEX(scan_results->found_devices[i].device_address);
    WS_DEBUG_PRINTLN("");
    WS_DEBUG_PRINT("SCL Pin: ");
    WS_DEBUG_PRINTLNVAR(scan_results->found_devices[i].pin_scl);
    WS_DEBUG_PRINT("SDA Pin: ");
    WS_DEBUG_PRINTLNVAR(scan_results->found_devices[i].pin_sda);
    WS_DEBUG_PRINT("MUX Address: ");
    WS_DEBUG_PRINTLNVAR(scan_results->found_devices[i].mux_address);
    WS_DEBUG_PRINT("MUX Channel: ");
    WS_DEBUG_PRINTLNVAR(scan_results->found_devices[i].mux_channel);
  }

  // Set bus status and publish scan results
  _i2c_model->setI2cBusScannedStatus(ws_i2c_BusStatus_BS_SUCCESS);
  if (!publishScan()) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: Failed to publish I2C bus scan results!");
    return false;
  }
  return true;
}


/*!
    @brief    Implements handling for a I2cDeviceAddOrReplace message
    @param    msg
              Pointer to the I2cDeviceAddOrReplace message.
    @returns  True if the I2cDeviceAddOrReplace message was handled
              (created or replaced), False otherwise.
*/
bool I2cController::Handle_I2cDeviceAddOrReplace(
    ws_i2c_DeviceAddOrReplace *msg) {

  ws_i2c_DeviceStatus device_status = ws_i2c_DeviceStatus_DS_UNSPECIFIED;
  // Parse out device name and descriptor
  char device_name[15];
  strcpy(device_name, msg->device_name);
  ws_i2c_DeviceDescriptor device_descriptor = msg->device_description;

  // Should we use GPS passthrough mode for this device? (only applies to GPS devices,
  // but we check here since it's a property of the driver, not the bus)
  bool use_gps_passthrough = msg->has_gps_config;

  // Should we use the MUX for this device? We determine this based on if the mux_address field is set in the descriptor
  bool use_mux = (device_descriptor.mux_address != 0x00);

  // Attempt to remove any existing driver at this address (or mux_channel).
  RemoveDriver(device_descriptor.device_address, device_descriptor.mux_channel);

  // Attempt to find or create the I2C bus specified by the device descriptor
  I2cHardware *hw_bus = findOrCreateBus(device_descriptor.pin_scl, device_descriptor.pin_sda);
  if (hw_bus == nullptr) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: Failed to find or create I2C bus specified by device descriptor!");
    publishDeviceAddedOrReplaced(device_descriptor, nullptr, ws_i2c_DeviceStatus_DS_FAIL_INIT);
    return false;
  }

  // Before we do anything on the bus - was the bus initialized correctly?
  if (!IsBusStatusOK(hw_bus)) {
    WS_DEBUG_PRINTLN("[i2c] Bus is stuck or not operational, reset the board!");
    if (Ws._sdCardV2->isModeOffline()) {
      Ws.haltErrorV2(" ", WS_LED_STATUS_ERROR_RUNTIME,
                     false); // doesn't return, halts
    }
    // Publish back out to IO with error status from bus status check
    if (!publishDeviceAddedOrReplaced(device_descriptor, hw_bus, device_status)) {
      WS_DEBUG_PRINTLN("[i2c] ERROR: Unable to publish message to IO!");
      return false;
    }
    return false;
  }

  // Get the TwoWire bus from the hardware object for driver creation
  TwoWire *wire = hw_bus->GetBus();

  // Check we are trying to add a new MUX - if so, initialize and return
  ws_i2c_DeviceStatus mux_status = InitMux(hw_bus, device_name, device_descriptor.mux_address);
  if (mux_status == ws_i2c_DeviceStatus_DS_SUCCESS) {
    // MUX initialized successfully, publish and back out since we don't need to create a driver for the MUX itself
    publishDeviceAddedOrReplaced(device_descriptor, hw_bus, mux_status);
    return true;
  } else if (mux_status != ws_i2c_DeviceStatus_DS_UNSPECIFIED) {
    // MUX init failed, publish and back out since we can't use the MUX bus if it failed to initialize
    publishDeviceAddedOrReplaced(device_descriptor, hw_bus, mux_status);
    Ws.haltErrorV2("[i2c] Failed to initialize MUX driver!",
                   WS_LED_STATUS_ERROR_RUNTIME, false);
  }

  // Check if we are trying to add a new GPS driver - if so, initialize GPS driver and return
  if (use_gps_passthrough) {
    ws_i2c_DeviceStatus gps_status = InitGpsDriver(wire, device_descriptor.device_address, &msg->gps_config);
    publishDeviceAddedOrReplaced(device_descriptor, hw_bus, gps_status);
    return (gps_status == ws_i2c_DeviceStatus_DS_SUCCESS);
  }

  // Check if we need to configure the MUX channel for this device - if so, configure it before creating the driver
  if (use_mux) {
    if (hw_bus->HasMux()) {
      hw_bus->ClearMuxChannel(); // TODO: We may be able to remove this, test against IO first!
      hw_bus->SelectMuxChannel(device_descriptor.mux_channel);
    } else {
      WS_DEBUG_PRINTLN("[i2c] ERROR: Expected a MUX on this bus but none found!");
      publishDeviceAddedOrReplaced(device_descriptor, hw_bus, ws_i2c_DeviceStatus_DS_FAIL_UNSUPPORTED_SENSOR);
      return false;
    }
  }


  // Attempt to initialize a new sensor driver
  WS_DEBUG_PRINTLN("[i2c] Creating sensor driver...");
  drvBase *drv = CreateI2cSensorDrv(device_name, wire,
                                    device_descriptor.device_address,
                                    device_descriptor.mux_channel, device_status);

  if (drv == nullptr) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: I2C driver failed to initialize!");
    publishDeviceAddedOrReplaced(device_descriptor, hw_bus, ws_i2c_DeviceStatus_DS_FAIL_INIT);
    return false;
  }

  // Store the bus pins on the driver so we can query it
  drv->SetPins(hw_bus->getSCL(), hw_bus->getSDA());

  // Configure sensor driver settings
  drv->EnableSensorReads(msg->device_sensor_types,
                         msg->device_sensor_types_count);
  drv->SetSensorPeriod(msg->device_period);

  // Optionally configure the driver's MUX address
  if (use_mux) {
    drv->SetMuxAddress(device_descriptor.mux_address);
    WS_DEBUG_PRINTLN("[i2c] Set driver to use MUX");
  }

  // Attempt to communicate with the driver
  if (!drv->begin()) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: Failed to initialize I2C driver!");
    publishDeviceAddedOrReplaced(device_descriptor, hw_bus, ws_i2c_DeviceStatus_DS_FAIL_INIT);
    delete drv;
    return false;
  }

  WS_DEBUG_PRINTLN("[i2c] Driver initialized and added to controller: ");
  WS_DEBUG_PRINTLNVAR(device_name);
  _i2c_drivers.push_back(drv);

  // If we're using a MUX, lets clear the channel for any subsequent bus operations that may not involve the MUX
  if (use_mux) {
    hw_bus->ClearMuxChannel();
  }

  // Publish success status back out to IO
  WS_DEBUG_PRINT("[i2c] Successfully added/replaced device, publishing status to IO...");
  publishDeviceAddedOrReplaced(device_descriptor, hw_bus, ws_i2c_DeviceStatus_DS_SUCCESS);
  WS_DEBUG_PRINTLN("OK!");

  return true;
}


/*!
    @brief    Handles polling, reading, and logger for i2c devices
              attached to the I2C controller.
    @param    force
                True to force an update regardless of sensor period,
                False to respect existing sensor period.
*/
void I2cController::update(bool force) {
  if (_i2c_drivers.empty())
    return; // bail out if no drivers exist

  for (auto *drv : _i2c_drivers) {
    // Does this driver have any enabled sensors?
    size_t sensor_count = drv->GetEnabledSensorCnt();
    if (sensor_count == 0)
      continue; // bail out if driver has no sensors enabled

    if (drv->GetDidReadSend())
      continue; // bail out if driver has already read and sent data to IO

    // Did driver's period elapse yet?
    ulong cur_time = millis();
    if (cur_time - drv->GetSensorPeriodPrv() < drv->GetSensorPeriod() && !force)
      continue; // bail out if the period hasn't elapsed yet or we aren't
                // forcing an update

    // Get the I2cHardware bus for this driver using its stored pins
    I2cHardware *drv_bus = findOrCreateBus(drv->GetPinSCL(), drv->GetPinSDA());

    // Optionally configure the I2C MUX
    uint32_t mux_channel = drv->GetMuxChannel();
    WS_DEBUG_PRINTLNVAR(mux_channel);
    if (drv->HasMux() && drv_bus != nullptr) {
      drv_bus->ClearMuxChannel();
      drv_bus->SelectMuxChannel(mux_channel);
    }

    // Read the driver's sensors
    _i2c_model->ClearI2cDeviceEvent();
    bool read_succeeded = true;
    for (size_t i = 0; i < sensor_count; i++) {
      sensors_event_t event = {0};
      // Attempt to call driver's read handler function
      if (!drv->GetSensorEvent(drv->_sensors[i], &event)) {
        WS_DEBUG_PRINTLN("[i2c] ERROR: Failed to read sensor!");
        read_succeeded = false;
        continue;
      }

      // Check if this is a battery monitor reporting percentage
      if (Ws._sdCardV2 != nullptr &&
          drv->_sensors[i] == ws_sensor_Type_T_UNITLESS_PERCENT &&
          (strcmp(drv->GetDrvName(), "max17048") == 0 ||
           strcmp(drv->GetDrvName(), "lc709203f") == 0 ||
           strcmp(drv->GetDrvName(), "sensor_mock") == 0)) {
        Ws._sdCardV2->SetBatteryPercent(event.unitless_percent);
      }

      // Fill the I2cDeviceEvent's sensor_event array submsg.
      _i2c_model->AddI2cDeviceSensorEvent(event, drv->_sensors[i]);
    }

    if (!read_succeeded)
      continue;

    // Configure the DeviceEvent's DeviceDescription sub-msg
    _i2c_model->SetI2cDeviceEventDeviceDescripton(
        drv->GetPinSCL(), drv->GetPinSDA(), (uint32_t)drv->GetAddress(),
        drv->GetMuxAddress(), mux_channel);
    _i2c_model->EncodeI2cDeviceEvent();

    if (!Ws._sdCardV2->isModeOffline()) {
      // TODO: Implement online mode publishing
    } else {
      if (!Ws._sdCardV2->LogI2cDeviceEvent(_i2c_model->GetI2cDeviceEvent())) {
        WS_DEBUG_PRINTLN(
            "[i2c] ERROR: Unable to log the I2cDeviceEvent to SD!");
        statusLEDSolid(WS_LED_STATUS_FS_WRITE);
      }
    }
    drv->SetDidReadSend(true);

    cur_time = millis();
    drv->SetSensorPeriodPrv(cur_time);
  }
}

/*!
    @brief    Checks if all I2C drivers have completed their read/send cycle.
    @returns  True if all drivers have completed their read/send cycle,
              False otherwise.
*/
bool I2cController::UpdateComplete() {
  for (auto *drv : _i2c_drivers) {
    if (drv->GetEnabledSensorCnt() == 0)
      continue; // skip drivers with no enabled sensors

    if (!drv->GetDidReadSend())
      return false; // found a driver that hasn't completed its read/send
  }
  return true; // all drivers have completed their read/send
}

/*!
    @brief  Resets all I2C drivers' did_read_send flags to false.
*/
void I2cController::ResetFlags() {
  for (auto *drv : _i2c_drivers) {
    drv->SetDidReadSend(false);
  }
}


/*!
    @brief  Returns a pointer to the I2C bus by SCL/SDA pins.
    @param  pin_scl
            The SCL pin number.
    @param  pin_sda
            The SDA pin number.
    @returns  Pointer to the TwoWire bus, or nullptr if the bus doesn't exist.
*/
TwoWire *I2cController::GetI2cBus(uint32_t pin_scl, uint32_t pin_sda) {
  for (I2cHardware *bus : _i2c_buses) {
    if (bus == nullptr)
      continue;
    if (pin_scl == (uint32_t)bus->getSCL() &&
        pin_sda == (uint32_t)bus->getSDA()) {
      return bus->GetBus();
    }
  }
  return nullptr;
}

/*!
    @brief  Returns the number of I2C buses.
    @returns  The number of I2C buses.
*/
size_t I2cController::GetI2cBusCount() {
  return _i2c_buses.size();
}

/*!
    @brief  Returns a pointer to the I2C bus by index.
    @param  index
            The index of the bus.
    @returns  Pointer to the TwoWire bus, or nullptr if the index is invalid.
*/
TwoWire *I2cController::GetI2cBusByIndex(size_t index) {
  if (index < _i2c_buses.size() && _i2c_buses[index] != nullptr) {
    return _i2c_buses[index]->GetBus();
  }
  return nullptr;
}