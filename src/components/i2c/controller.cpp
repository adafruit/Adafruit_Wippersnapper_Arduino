/*!
 * @file controller.cpp
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

/*******************************************************************************/
/*!
    @brief    lambda function to create a drvBase driver instance
      @param    i2c
                  The I2C interface.
      @param    addr
                  7-bit device address.
      @param    mux_channel
                  The I2C multiplexer channel.
      @param    driver_name
                  The name of the driver.
*/
/*******************************************************************************/
using FnCreateI2CDriver =
    std::function<drvBase *(TwoWire *, uint16_t, uint32_t, const char *)>;

// Map of sensor names to lambda functions that create an I2C device driver
static const std::map<std::string, FnCreateI2CDriver> I2cFactory = {
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

/***********************************************************************/
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
/***********************************************************************/
drvBase *createI2CDriverByName(const char *driver_name, TwoWire *i2c,
                               uint16_t addr, uint32_t i2c_mux_channel,
                               wippersnapper_i2c_I2cDeviceStatus &status) {
  auto it = I2cFactory.find(driver_name);
  if (it == I2cFactory.end()) {
    status =
        wippersnapper_i2c_I2cDeviceStatus_I2C_DEVICE_STATUS_FAIL_UNSUPPORTED_SENSOR;
    return nullptr;
  }
  status = wippersnapper_i2c_I2cDeviceStatus_I2C_DEVICE_STATUS_SUCCESS;
  // Call the lambda to create the driver
  return it->second(i2c, addr, i2c_mux_channel, driver_name);
}

/***********************************************************************/
/*!
    @brief  I2cController constructor
*/
/***********************************************************************/
I2cController::I2cController() {
  _i2c_bus_alt = nullptr;
  _i2c_model = new I2cModel();
  // Initialize a default I2C bus
  _i2c_bus_default = new I2cHardware();
  _i2c_bus_default->InitBus(true);
}

/***********************************************************************/
/*!
    @brief  I2cController destructor
*/
/***********************************************************************/
I2cController::~I2cController() {
  if (_i2c_model)
    delete _i2c_model;

  if (_i2c_bus_default)
    delete _i2c_bus_default;
}

/*************************************************************************/
/*!
    @brief    Returns the status of the I2C bus
    @param    is_alt_bus
              True if the alt. I2C bus is being queried, False otherwise.
    @returns  True if the I2C bus is operational, False otherwise.
*/
/*************************************************************************/
bool I2cController::IsBusStatusOK(bool is_alt_bus) {
  if (is_alt_bus) {
    return (_i2c_bus_alt->GetBusStatus() ==
            wippersnapper_i2c_I2cBusStatus_I2C_BUS_STATUS_SUCCESS);
  }

  return (_i2c_bus_default->GetBusStatus() ==
          wippersnapper_i2c_I2cBusStatus_I2C_BUS_STATUS_SUCCESS);
}

/***********************************************************************/
/*!
    @brief    Publishes an I2cDeviceAddedorReplaced message to the broker
    @param    device_descriptor
                The I2cDeviceDescriptor message.
    @param    device_status
                The I2cDeviceStatus message.
    @returns  True if the I2cDeviceAddedorReplaced message was published
              successfully, False otherwise.
*/
/***********************************************************************/
bool I2cController::PublishI2cDeviceAddedorReplaced(
    const wippersnapper_i2c_I2cDeviceDescriptor &device_descriptor,
    const wippersnapper_i2c_I2cDeviceStatus &device_status) {
  if (!_i2c_model->encodeMsgI2cDeviceAddedorReplaced(
          device_descriptor, _i2c_bus_default->GetBusStatus(), device_status)) {
    WS_DEBUG_PRINTLN(
        "[i2c] ERROR: Unable to encode I2cDeviceAddedorReplaced message!");
    return false;
  }

  if (WsV2._sdCardV2->isModeOffline())
    return true; // Back out if we're in offline mode

  if (!WsV2.PublishSignal(
          wippersnapper_signal_DeviceToBroker_i2c_device_added_replaced_tag,
          _i2c_model->GetMsgI2cDeviceAddedOrReplaced())) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: Unable to publish I2cDeviceAddedorReplaced "
                     "message to IO!");
    return false;
  }
  return true;
}

/***********************************************************************/
/*!
    @brief    Implements handling for a I2cDeviceRemove message
    @param    stream
                A pointer to the pb_istream_t stream.
    @returns  True if the I2cDeviceRemove message was handled, False
              otherwise.
*/
/***********************************************************************/
bool I2cController::Handle_I2cDeviceRemove(pb_istream_t *stream) {
  // Attempt to decode an I2cDeviceRemove message
  WS_DEBUG_PRINTLN("[i2c] Decoding I2cDeviceRemove message...");
  if (!_i2c_model->DecodeI2cDeviceRemove(stream)) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: Unable to decode I2cDeviceRemove message!");
    return false;
  }

  // TODO: Implement the rest of this function
  WS_DEBUG_PRINTLN("[i2c] I2cDeviceRemove message not yet implemented!");

  return true;
}

/***********************************************************************/
/*!
    @brief    Implements handling for a I2cDeviceAddOrReplace message
    @param    stream
                A pointer to the pb_istream_t stream.
    @returns  True if the I2cDeviceAddOrReplace message was handled
              (created or replaced), False otherwise.
*/
/***********************************************************************/
bool I2cController::Handle_I2cDeviceAddOrReplace(pb_istream_t *stream) {
  bool use_alt_bus = false;
  bool did_set_mux_ch = false;
  // Attempt to decode an I2cDeviceAddOrReplace message
  WS_DEBUG_PRINTLN("[i2c] Decoding I2cDeviceAddOrReplace message...");
  if (!_i2c_model->DecodeI2cDeviceAddReplace(stream)) {
    WS_DEBUG_PRINTLN(
        "[i2c] ERROR: Unable to decode I2cDeviceAddOrReplace message!");
    return false;
  }
  wippersnapper_i2c_I2cDeviceStatus device_status =
      wippersnapper_i2c_I2cDeviceStatus_I2C_DEVICE_STATUS_UNSPECIFIED;
  wippersnapper_i2c_I2cDeviceDescriptor device_descriptor =
      _i2c_model->GetI2cDeviceAddOrReplaceMsg()->i2c_device_description;
  char device_name[15];
  strcpy(device_name,
         _i2c_model->GetI2cDeviceAddOrReplaceMsg()->i2c_device_name);

  // TODO: Handle Replace messages by implementing a Remove handler first...then
  // proceed to adding a new device

  // Does the device's descriptor specify a different i2c bus?
  if (strcmp(device_descriptor.i2c_bus_scl, "default") != 0) {
    WS_DEBUG_PRINTLN("[i2c] Non-default I2C bus specified!");
    if (_i2c_bus_alt == nullptr) {
      WS_DEBUG_PRINTLN("[i2c] Initializing alternative i2c bus...");
      _i2c_bus_alt = new I2cHardware();
      _i2c_bus_alt->InitBus(false, device_descriptor.i2c_bus_sda,
                            device_descriptor.i2c_bus_scl);
    }
    use_alt_bus = true;
  }

  // Before we do anything on the bus - was the bus initialized correctly?
  if (!IsBusStatusOK(use_alt_bus)) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: I2C bus is not operational or stuck, please "
                     "restart device!");
    if (!PublishI2cDeviceAddedorReplaced(device_descriptor, device_status))
      return false;
    return true;
  }

  // Mux case #1 - We are creating a mux via I2cDeviceAddorReplace message
  // TODO: Refactor
  if ((strcmp(device_name, "pca9546") == 0) ||
      (strcmp(device_name, "pca9548") == 0)) {
    WS_DEBUG_PRINTLN("[i2c] Creating a new MUX driver obj");
    if (use_alt_bus) {
      if (!_i2c_bus_alt->HasMux()) {
        WS_DEBUG_PRINT("[i2c] Adding MUX to alternate bus...");
        _i2c_bus_alt->AddMuxToBus(device_descriptor.i2c_mux_address,
                                  device_name);
      } else {
        WS_DEBUG_PRINTLN("[i2c] ERROR: Mux specified but not created");
      }
    } else {
      if (!_i2c_bus_default->HasMux()) {
        WS_DEBUG_PRINT("[i2c] Adding MUX to default bus...");
        _i2c_bus_default->AddMuxToBus(device_descriptor.i2c_mux_address,
                                      device_name);
      } else {
        WS_DEBUG_PRINTLN("[i2c] ERROR: Mux specified but not created");
      }
    }
    // TODO: Publish back out to IO instead of blindly returning true
    return true;
  }

  // Mux case #2 - We are creating a new driver that USES THE MUX via
  // I2cDeviceAddorReplace message
  if (device_descriptor.i2c_mux_address != 0x00) {
    uint32_t mux_channel = device_descriptor.i2c_mux_channel;
    if (use_alt_bus) {
      if (_i2c_bus_alt->HasMux()) {
        _i2c_bus_alt->SelectMuxChannel(mux_channel);
        WS_DEBUG_PRINT("[i2c] Selected MUX CH: ");
        WS_DEBUG_PRINTLN(mux_channel);
        did_set_mux_ch = true;
      } else {
        WS_DEBUG_PRINTLN("[i2c] ERROR: Device requests a MUX but MUX has not "
                         "been initialized first");
        device_status =
            wippersnapper_i2c_I2cDeviceStatus_I2C_DEVICE_STATUS_FAIL_INIT;
        PublishI2cDeviceAddedorReplaced(device_descriptor, device_status);
        return false;
      }
    } else {
      if (_i2c_bus_default->HasMux()) {
        _i2c_bus_default->SelectMuxChannel(mux_channel);
        WS_DEBUG_PRINT("[i2c] Selected MUX CH: ");
        WS_DEBUG_PRINTLN(mux_channel);
        did_set_mux_ch = true;
      } else {
        WS_DEBUG_PRINTLN("[i2c] ERROR: Device requests a MUX but MUX has not "
                         "been initialized first");
        device_status =
            wippersnapper_i2c_I2cDeviceStatus_I2C_DEVICE_STATUS_FAIL_INIT;
        PublishI2cDeviceAddedorReplaced(device_descriptor, device_status);
        return false;
      }
    }
  }

  WS_DEBUG_PRINTLN("Creating a new I2C driver obj");
  TwoWire *bus = nullptr;
  if (use_alt_bus) {
    bus = _i2c_bus_alt->GetBus();
  } else {
    bus = _i2c_bus_default->GetBus();
  }

  drvBase *drv = createI2CDriverByName(
      device_name, bus, device_descriptor.i2c_device_address,
      device_descriptor.i2c_mux_channel, device_status);

  // TODO: Clean up for clarity - confusing checks and returns
  if (drv != nullptr) {
    WS_DEBUG_PRINTLN("OK! Configuring sensor types and period...");
    drv->EnableSensorReads(
        _i2c_model->GetI2cDeviceAddOrReplaceMsg()->i2c_device_sensor_types,
        _i2c_model->GetI2cDeviceAddOrReplaceMsg()
            ->i2c_device_sensor_types_count);
    drv->SetSensorPeriod(
        _i2c_model->GetI2cDeviceAddOrReplaceMsg()->i2c_device_period);
    if (did_set_mux_ch)
      drv->SetMuxAddress(device_descriptor.i2c_mux_address);
    if (use_alt_bus)
      drv->EnableAltI2CBus(_i2c_model->GetI2cDeviceAddOrReplaceMsg()
                               ->i2c_device_description.i2c_bus_scl,
                           _i2c_model->GetI2cDeviceAddOrReplaceMsg()
                               ->i2c_device_description.i2c_bus_sda);

    if (drv->begin()) {
      _i2c_drivers.push_back(drv);
      WS_DEBUG_PRINTLN(
          "[i2c] I2C driver initialized and added to controller: ");
      WS_DEBUG_PRINTLN(device_name);

      // If we're using a MUX, clear the channel for any subsequent bus
      // operations that may not involve the MUX
      if (did_set_mux_ch) {
        if (use_alt_bus) {
          _i2c_bus_alt->ClearMuxChannel();
        } else {
          _i2c_bus_default->ClearMuxChannel();
        }
      }
    } else {
      WS_DEBUG_PRINTLN("[i2c] ERROR: I2C driver failed to initialize!");
      device_status =
          wippersnapper_i2c_I2cDeviceStatus_I2C_DEVICE_STATUS_FAIL_INIT;

      if (WsV2._sdCardV2->isModeOffline()) {
        WS_DEBUG_PRINTLN("[i2c] Driver failed to initialize!\n\tDid you set "
                         "the correct value for i2cDeviceName?\n\tDid you set "
                         "the correct value for"
                         "i2cDeviceAddress?");
        while (
            1) { // Keep the WIPPER drive open to allow user to edit config.json
          WsV2.feedWDTV2();
          delay(500);
        }
      }
      if (!PublishI2cDeviceAddedorReplaced(device_descriptor, device_status))
        return false;
      return true;
    }
  } else {
    WS_DEBUG_PRINTLN("[i2c] ERROR: I2C driver type not found or unsupported!");
    device_status =
        wippersnapper_i2c_I2cDeviceStatus_I2C_DEVICE_STATUS_FAIL_UNSUPPORTED_SENSOR;

    if (WsV2._sdCardV2->isModeOffline()) {
      WS_DEBUG_PRINTLN("[i2c] Driver failed to initialize!\n\tDid you set "
                       "the correct value for i2cDeviceName?\n\tDid you set "
                       "the correct value for"
                       "i2cDeviceAddress?");
      while (
          1) { // Keep the WIPPER drive open to allow user to edit config.json
        WsV2.feedWDTV2();
        delay(500);
      }
    }

    if (!PublishI2cDeviceAddedorReplaced(device_descriptor, device_status))
      return false;
    return true;
  }

  // Create and publish the I2cDeviceAddedorReplaced message to the broker
  if (!PublishI2cDeviceAddedorReplaced(device_descriptor, device_status))
    return false;
  return true;
}

/********************************************************************************/
/*!
    @brief    Enables a MUX channel on the appropriate I2C bus.
    @param    mux_channel
                Desired MUX channel to enable
    @param    is_alt_bus
                True if an alternative I2C bus is being used, False otherwise.
*/
/********************************************************************************/
void I2cController::ConfigureMuxChannel(uint32_t mux_channel, bool is_alt_bus) {
  if (is_alt_bus) {
    _i2c_bus_alt->ClearMuxChannel(); // sanity-check, may not be required
    _i2c_bus_alt->SelectMuxChannel(mux_channel);
    return;
  }
  _i2c_bus_default->ClearMuxChannel(); // sanity-check, may not be required
  _i2c_bus_default->SelectMuxChannel(mux_channel);
}

/***********************************************************************/
/*!
    @brief    Handles polling, reading, and logger for i2c devices
              attached to the I2C controller.
*/
/***********************************************************************/
void I2cController::update() {
  if (_i2c_drivers.size() == 0)
    return; // bail out if no drivers exist

  for (auto *drv : _i2c_drivers) {
    // Does this driver have any enabled sensors?
    size_t sensor_count = drv->GetEnabledSensorCnt();
    if (sensor_count == 0)
      continue; // bail out if driver has no sensors enabled

    // Did driver's period elapse yet?
    ulong cur_time = millis();
    if (cur_time - drv->GetSensorPeriodPrv() < drv->GetSensorPeriod())
      continue; // bail out if the period hasn't elapsed yet

    // Optionally configure the I2C MUX
    uint32_t mux_channel = drv->GetMuxChannel();
    if (drv->HasMux())
      ConfigureMuxChannel(mux_channel, drv->HasAltI2CBus());

    // Read the driver's sensors
    _i2c_model->ClearI2cDeviceEvent();
    for (size_t i = 0; i < sensor_count; i++) {
      sensors_event_t event;
      // Call the driver's handler function for the SensorType
      drv->GetSensorEvent(drv->_sensors[i], &event);
      // Fill the I2cDeviceEvent's sensor_event array submsg.
      _i2c_model->AddI2cDeviceSensorEvent(event, drv->_sensors[i]);
    }

    // Configure the DeviceEvent's DeviceDescription sub-msg
    _i2c_model->SetI2cDeviceEventDeviceDescripton(
        drv->GetPinSCL(), drv->GetPinSDA(), (uint32_t)drv->GetAddress(),
        drv->GetMuxAddress(), mux_channel);
    _i2c_model->EncodeI2cDeviceEvent();

    // Handle the DeviceEvent message
    if (WsV2._sdCardV2->isModeOffline()) {
      if (!WsV2._sdCardV2->LogI2cDeviceEvent(_i2c_model->GetI2cDeviceEvent())) {
        WS_DEBUG_PRINTLN(
            "[i2c] ERROR: Unable to log the I2cDeviceEvent to SD!");
        statusLEDSolid(WS_LED_STATUS_FS_WRITE);
      }
    } else {
      // TODO: This needs to be implemented for online mode
      WS_DEBUG_PRINTLN(
          "[i2c] MQTT Publish I2cDeviceEvent not yet implemented!");
    }

    cur_time = millis();
    drv->SetSensorPeriodPrv(cur_time);
  }
}