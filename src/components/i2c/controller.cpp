/*!
 * @file src/components/i2c/controller.cpp
 *
 * Controller for the i2c.proto API
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025-2026 for Adafruit Industries.
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
    @returns  A pointer to the I2C driver.
*/
drvBase *CreateI2cSensorDrv(const char *driver_name, TwoWire *i2c,
                            uint16_t addr, uint32_t i2c_mux_channel) {
  auto it = I2cFactorySensor.find(driver_name);
  if (it == I2cFactorySensor.end()) {
    return nullptr;
  }

  return it->second(i2c, addr, i2c_mux_channel, driver_name);
}

/*!
    @brief  I2cController constructor
*/
I2cController::I2cController() { _i2c_model = new I2cModel(); }

/*!
    @brief  I2cController destructor
*/
I2cController::~I2cController() {
  if (_i2c_model)
    delete _i2c_model;
}

/******************************************************************************/
/*!                             Routing                                       */
/******************************************************************************/

/*!
    @brief  Routes messages using the i2c.proto API to the
            appropriate controller functions.
    @param  stream
            The nanopb input stream.
    @return True if the message was successfully routed, False otherwise.
*/
bool I2cController::Router(pb_istream_t *stream) {
  // Save stream before decoding — Handle_Probe needs to re-decode
  // with callbacks, and ws_pb_decode consumes the stream.
  pb_istream_t saved_stream = *stream;

  // Decode B2D without callbacks to determine which payload is active.
  // Do NOT pre-initialize probe callbacks here — payload is a union, so
  // writing to the probe member clobbers add/remove storage.
  ws_i2c_B2D b2d = ws_i2c_B2D_init_zero;
  if (!ws_pb_decode(stream, ws_i2c_B2D_fields, &b2d)) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: Unable to decode I2C B2D envelope");
    return false;
  }

  // Route based on payload type
  bool res = false;
  switch (b2d.which_payload) {
  case ws_i2c_B2D_probe_tag:
    // Pass the saved_stream to Handle_Probe since ws_pb_decode above has
    // already consumed the stream.
    res = Handle_Probe(&saved_stream);
    break;
  case ws_i2c_B2D_add_tag:
    res = Handle_Add(&b2d.payload.add);
    break;
  case ws_i2c_B2D_remove_tag:
    res = Handle_Remove(&b2d.payload.remove);
    break;
  default:
    WS_DEBUG_PRINTLN("[i2c] WARNING: Unsupported I2C payload");
    res = false;
    break;
  }

  return res;
}

/*!
    @brief    Implements handling for a I2cDeviceAddOrReplace message
    @param    msg
              Pointer to the I2cDeviceAddOrReplace message.
    @returns  True if the I2cDeviceAddOrReplace message was handled
              (created or replaced), False otherwise.
*/
bool I2cController::Handle_Add(ws_i2c_Add *msg) {
  // Validate the message and its fields
  if (msg == nullptr) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: Nullptr in I2C add message!");
    Ws.error_handler->publishComponentError(ws_i2c_Descriptor{},
                                            "No I2C add message provided!");
    return false;
  }
  if (strlen(msg->name) == 0) {
    Ws.error_handler->publishComponentError(
        ws_i2c_Descriptor{}, "Empty device name in I2C add message!");
    return false;
  }
  if (msg->descriptor.address == 0) {
    Ws.error_handler->publishComponentError(
        msg->descriptor, "Invalid device address in I2C add message!");
    return false;
  }

  // Parse out device name and descriptor
  char name[16];
  if (strlen(msg->name) >= sizeof(name)) {
    Ws.error_handler->publishComponentError(
        msg->descriptor, "Name in Add message exceeds length!");
    return false;
  }
  strcpy(name, msg->name);
  ws_i2c_Descriptor descriptor = msg->descriptor;

  // Should we use the MUX for this device? We determine this based on if the
  // mux_address field is set in the descriptor
  bool use_mux = (descriptor.address_space.mux_address != 0x00);

  // Attempt to remove any existing driver at this address (or mux_channel).
  RemoveDriver(descriptor.address, descriptor.address_space.mux_channel);

  // Attempt to find/create the I2C bus specified by the device descriptor
  I2cHardware *bus = findOrCreateBus(descriptor.address_space.pin_scl,
                                     descriptor.address_space.pin_sda);
  if (bus == nullptr) {
    Ws.error_handler->publishComponentError(
        descriptor, "Failed to find/create I2C bus for device!");
    return false;
  }

  // Before we do anything on the bus - was the bus initialized correctly?
  if (!IsBusStatusOK(bus)) {
    Ws.error_handler->publishComponentError(
        descriptor, "I2C bus is not operational, reset the board!");
    return false;
  }

  // Attempt to fetch the TwoWire bus from the hardware object
  TwoWire *wire = bus->GetBus();
  // Validate pointer
  if (wire == nullptr) {
    Ws.error_handler->publishComponentError(descriptor,
                                            "Failed to fetch TwoWire object!");
    return false;
  }

  // Check we are trying to add a new MUX - if so, just initialize and return
  if (strcmp(name, "pca9546") == 0 || strcmp(name, "pca9548") == 0) {
    if (InitMux(bus, name, descriptor.address_space.mux_address)) {
      return true;
    }
    Ws.error_handler->publishComponentError(
        descriptor, "Failed to initialize MUX for device!");
    return false;
  }

  // Check if we need to configure the MUX channel for this device - if so,
  // configure it before creating the driver
  if (use_mux) {
    if (bus->HasMux()) {
      bus->ClearMuxChannel();
      bus->SelectMuxChannel(descriptor.address_space.mux_channel);
    } else {
      Ws.error_handler->publishComponentError(descriptor,
                                              "No MUX found on bus!");
      return false;
    }
  }

  // Attempt to initialize a new sensor driver
  WS_DEBUG_PRINTLN("[i2c] Creating sensor driver...");
  drvBase *drv = CreateI2cSensorDrv(name, wire, descriptor.address,
                                    descriptor.address_space.mux_channel);
  if (drv == nullptr) {
    Ws.error_handler->publishComponentError(descriptor,
                                            "Failed to create driver!");
    if (use_mux) {
      bus->ClearMuxChannel();
    }
    return false;
  }

  // Store the bus pins on the driver so we can query it
  drv->SetPins(bus->getSCL(), bus->getSDA());

  // Configure sensor driver settings
  drv->EnableSensorReads(msg->types, msg->types_count);
  drv->SetSensorPeriod(msg->period);

  // Optionally configure the driver's MUX address
  if (use_mux) {
    drv->SetMuxAddress(descriptor.address_space.mux_address);
  }

  // Attempt to communicate with the driver
  if (!drv->begin()) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: Failed to initialize I2C driver!");
    Ws.error_handler->publishComponentError(
        descriptor, "Failed to initialize driver, check wiring + address!");
    if (use_mux) {
      bus->ClearMuxChannel();
    }
    delete drv;
    return false;
  }

  WS_DEBUG_PRINTLN("[i2c] Driver initialized: ");
  WS_DEBUG_PRINTLNVAR(name);
  _i2c_drivers.push_back(drv);

  // If we're using a MUX, lets clear the channel for any subsequent bus
  // operations that may not involve the MUX
  if (use_mux) {
    bus->ClearMuxChannel();
  }

  return true;
}

/*!
    @brief    Handles an I2C Probe request. Decodes the B2D envelope with
              probe callbacks to capture repeated fields, then iterates
              each address space, probes the listed addresses on each,
              and publishes the Probed results.
    @param    stream
              Saved copy of the nanopb input stream (pre-decode).
    @returns  True if probe completed and results published, False otherwise.
*/
bool I2cController::Handle_Probe(pb_istream_t *stream) {
  // Decode B2D with probe callbacks — safe to write to the probe union
  // member here because we know the payload IS a probe.
  ws_i2c_B2D b2d = ws_i2c_B2D_init_zero;
  _i2c_model->SetupProbeDecodeCallbacks(&b2d.payload.probe);
  if (!ws_pb_decode(stream, ws_i2c_B2D_fields, &b2d)) {
    Ws.error_handler->publishComponentError(ws_i2c_Descriptor{},
                                            "Failed to decode Probe message!");
    return false;
  }

  ws_i2c_AddressSpace *spaces = _i2c_model->GetProbeAddressSpaces();
  size_t spaces_count = _i2c_model->GetProbeAddressSpacesCount();
  uint32_t *addresses = _i2c_model->GetProbeAddresses();
  size_t addresses_count = _i2c_model->GetProbeAddressesCount();

  _i2c_model->ClearProbed();

  for (size_t i = 0; i < spaces_count; i++) {
    ws_i2c_AddressSpaceResult *result = _i2c_model->GetNextProbedResult();
    if (result == nullptr) {
      ws_i2c_Descriptor desc = {};
      desc.has_address_space = true;
      desc.address_space = spaces[i];
      Ws.error_handler->publishComponentError(
          desc, "Too many address spaces to probe!");
      break;
    }

    // Find or create the bus for this address space
    I2cHardware *bus = findOrCreateBus(spaces[i].pin_scl, spaces[i].pin_sda);
    if (bus == nullptr) {
      ws_i2c_Descriptor desc = {};
      desc.has_address_space = true;
      desc.address_space = spaces[i];
      Ws.error_handler->publishComponentError(
          desc, "Failed to find or create I2C bus!");
      continue;
    }

    // Get the found-addresses buffer for this result index
    size_t result_idx = i;
    uint32_t *found_buf = _i2c_model->GetFoundAddressBuf(result_idx);
    size_t *found_count = _i2c_model->GetFoundAddressCount(result_idx);
    if (found_buf == nullptr || found_count == nullptr) {
      ws_i2c_Descriptor desc = {};
      desc.has_address_space = true;
      desc.address_space = spaces[i];
      Ws.error_handler->publishComponentError(
          desc, "Internal error: probe buffer overflow!");
      continue;
    }

    // Probe the addresses on this bus/mux channel
    if (!bus->ProbeAddresses(&spaces[i], addresses, addresses_count, result,
                             found_buf, found_count)) {
      ws_i2c_Descriptor desc = {};
      desc.has_address_space = true;
      desc.address_space = spaces[i];
      Ws.error_handler->publishComponentError(desc, "ProbeAddresses failed!");
      continue;
    }

    WS_DEBUG_PRINT("[i2c] Probed address space ");
    WS_DEBUG_PRINTVAR(i);
    WS_DEBUG_PRINT(", found ");
    WS_DEBUG_PRINTVAR(*found_count);
    WS_DEBUG_PRINTLN(" devices.");
  }

  if (!publishProbed()) {
    Ws.error_handler->publishComponentError(
        ws_i2c_Descriptor{}, "Failed to publish Probed results!");
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
bool I2cController::Handle_Remove(ws_i2c_Remove *msg) {
  // Validate message
  if (msg->descriptor.address == 0) {
    Ws.error_handler->publishComponentError(msg->descriptor,
                                            "I2c address required for remove");
    return false;
  }
  if (msg->descriptor.address_space.pin_scl == 0 ||
      msg->descriptor.address_space.pin_sda == 0) {
    Ws.error_handler->publishComponentError(msg->descriptor,
                                            "SCL/SDA required for remove");
    return false;
  }

  I2cHardware *bus = findOrCreateBus(msg->descriptor.address_space.pin_scl,
                                     msg->descriptor.address_space.pin_sda);
  if (bus == nullptr) {
    Ws.error_handler->publishComponentError(
        msg->descriptor, "Failed to find I2C bus for remove");
    return false;
  }

  // Removing the MUX itself — tear down all devices on it first
  if (msg->descriptor.address == msg->descriptor.address_space.mux_address) {
    for (uint8_t ch = 0; ch < bus->GetMuxMaxChannels(); ch++) {
      ws_i2c_AddressSpace mux_space = {};
      mux_space.pin_scl = msg->descriptor.address_space.pin_scl;
      mux_space.pin_sda = msg->descriptor.address_space.pin_sda;
      mux_space.mux_address = msg->descriptor.address;
      mux_space.mux_channel = ch;
      ws_i2c_AddressSpaceResult result = {};
      uint32_t found_buf[112];
      size_t found_count = 0;
      bus->ProbeAddresses(&mux_space, nullptr, 0, &result, found_buf,
                          &found_count);
      for (size_t j = 0; j < found_count; j++) {
        (void)RemoveDriver(found_buf[j], ch);
      }
    }
    bus->RemoveMux();
    // Removing a device behind a MUX
  } else if (msg->descriptor.address_space.mux_address != 0) {
    bus->SelectMuxChannel(msg->descriptor.address_space.mux_channel);
    if (!RemoveDriver(msg->descriptor.address,
                      msg->descriptor.address_space.mux_channel)) {
      Ws.error_handler->publishComponentError(
          msg->descriptor, "Failed to remove I2C device from MUX channel");
      return false;
    }
    // Removing a device on the bare bus
  } else {
    if (!RemoveDriver(msg->descriptor.address)) {
      Ws.error_handler->publishComponentError(
          msg->descriptor, "Failed to remove I2C device from bus");
      return false;
    }
  }

  return true;
}

/******************************************************************************/
/*!                            Publishing                                     */
/******************************************************************************/

/*!
    @brief    Publishes the ws_i2c_Probed message to IO.
    @returns  True if published successfully, False otherwise.
*/
bool I2cController::publishProbed() {
  if (Ws._sdCardV2->isModeOffline())
    return true;

  if (!_i2c_model->EncodeProbed()) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: Unable to encode ws_i2c_Probed message!");
    return false;
  }

  if (!Ws.PublishD2b(ws_signal_DeviceToBroker_i2c_tag,
                     _i2c_model->GetI2cD2B())) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: Unable to publish ws_i2c_Probed message!");
    return false;
  }

  return true;
}

/******************************************************************************/
/*!                              Update                                       */
/******************************************************************************/

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
      if (!drv->GetSensorEvent(drv->_sensors[i].value, &event)) {
        WS_DEBUG_PRINTLN("[i2c] ERROR: Failed to read sensor!");
        read_succeeded = false;
        continue;
      }

      // Check if this is a battery monitor reporting percentage
      if (Ws._sdCardV2 != nullptr &&
          drv->_sensors[i].value == ws_sensor_Type_T_UNITLESS_PERCENT &&
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

/******************************************************************************/
/*!                             Helpers                                       */
/******************************************************************************/

/*!
    @brief  Finds or creates an I2C bus by SCL/SDA pins, validates it,
            and returns the underlying TwoWire instance.
    @param  pin_scl
            The SCL pin number.
    @param  pin_sda
            The SDA pin number.
    @returns  Pointer to the TwoWire bus, or nullptr if the bus could not
              be found/created or failed validation.
*/
TwoWire *I2cController::GetOrCreateI2cBus(uint32_t pin_scl, uint32_t pin_sda) {
  I2cHardware *bus = findOrCreateBus(pin_scl, pin_sda);
  if (bus == nullptr) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: Failed to find or create I2C bus!");
    return nullptr;
  }
  if (!IsBusStatusOK(bus)) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: I2C bus is not operational!");
    return nullptr;
  }
  return bus->GetBus();
}

/*!
    @brief    Attempts to initialize a MUX on the bus.
    @param    bus
                Pointer to the I2C hardware bus.
    @param    name
                The device name - checks if this is a supported MUX type.
    @param    address
                The MUX's I2C address.
    @returns  True if the MUX was initialized successfully, False if it failed
   to initialize.
*/
bool I2cController::InitMux(I2cHardware *bus, const char *name,
                            uint32_t address) {
  WS_DEBUG_PRINT("[i2c] Initializing MUX driver...");
  if (bus == nullptr) {
    WS_DEBUG_PRINTLN("Failure - bus is nullptr!");
    return false;
  }
  if (bus->HasMux()) {
    WS_DEBUG_PRINTLN("OK (already initialized)");
    return true;
  }
  if (!bus->AddMuxToBus(address, name)) {
    WS_DEBUG_PRINTLN("Failure - unable to add MUX to bus!");
    return false;
  }
  return true;
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
    if (driver->GetMuxChannel() != mux_channel)
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
  return false;
}

/*!
    @brief  Returns the number of I2C buses.
    @returns  The number of I2C buses.
*/
size_t I2cController::GetI2cBusCount() { return _i2c_buses.size(); }

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

/******************************************************************************/
/*!                             Private                                       */
/******************************************************************************/

/*!
    @brief    Finds an existing I2C bus by SCL/SDA pins, or creates a new one.
    @param    pin_scl
              The SCL pin number.
    @param    pin_sda
              The SDA pin number.
    @returns  Pointer to the I2cHardware bus, or nullptr if initialization
              failed.
*/
I2cHardware *I2cController::findOrCreateBus(uint32_t pin_scl,
                                            uint32_t pin_sda) {
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
  WS_DEBUG_PRINT("SCL Pin: ");
  WS_DEBUG_PRINTLNVAR(pin_scl);
  WS_DEBUG_PRINT("SDA Pin: ");
  WS_DEBUG_PRINTLNVAR(pin_sda);

  I2cHardware *new_bus = new I2cHardware(pin_sda, pin_scl);
  if (!new_bus->begin()) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: Failed to initialize I2C bus!");
    delete new_bus;
    return nullptr;
  }
  _i2c_buses.push_back(new_bus);
  WS_DEBUG_PRINTLN("[i2c] I2C bus initialized successfully!");
  return new_bus;
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
  return bus->isBusInitialized();
}

/*!
    @brief  Returns a pointer to the I2C bus by SCL/SDA pins
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
