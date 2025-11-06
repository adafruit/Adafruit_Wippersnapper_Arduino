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
#include "drivers/drvOutputBase.h"

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
    @brief     Lambda function to create a drvOutputBase instance
      @param    i2c
                  The desired I2C interface.
      @param    addr
                  The desired i2c device address.
      @param    mux_channel
                  The desired I2C multiplexer channel.
      @param    driver_name
                  The i2c output driver's name.
*/
using FnCreateI2cOutputDrv =
    std::function<drvOutputBase *(TwoWire *, uint16_t, uint32_t, const char *)>;

// Factory for creating a new i2c OUTPUT driver
// NOTE: When adding a new OUTPUT driver, make sure to add it to the map below!
static const std::map<std::string, FnCreateI2cOutputDrv> I2cFactoryOutput = {
    {"quadalphanum",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvOutputBase * {
       return new drvOutQuadAlphaNum(i2c, addr, mux_channel, driver_name);
     }},
    {"7seg",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvOutputBase * {
       return new drvOut7Seg(i2c, addr, mux_channel, driver_name);
     }},
    {"charlcd",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvOutputBase * {
       return new drvOutCharLcd(i2c, addr, mux_channel, driver_name);
     }},
    {"ssd1306",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvOutputBase * {
       return new drvOutSsd1306(i2c, addr, mux_channel, driver_name);
     }}}; ///< I2C output driver factory

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
                            wippersnapper_i2c_I2cDeviceStatus &status) {
  auto it = I2cFactorySensor.find(driver_name);
  if (it == I2cFactorySensor.end()) {
    status =
        wippersnapper_i2c_I2cDeviceStatus_I2C_DEVICE_STATUS_FAIL_UNSUPPORTED_SENSOR;
    return nullptr;
  }

  status = wippersnapper_i2c_I2cDeviceStatus_I2C_DEVICE_STATUS_SUCCESS;
  return it->second(i2c, addr, i2c_mux_channel, driver_name);
}

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
drvOutputBase *CreateI2cOutputDrv(const char *driver_name, TwoWire *i2c,
                                  uint16_t addr, uint32_t i2c_mux_channel,
                                  wippersnapper_i2c_I2cDeviceStatus &status) {
  auto it = I2cFactoryOutput.find(driver_name);
  if (it == I2cFactoryOutput.end()) {
    status =
        wippersnapper_i2c_I2cDeviceStatus_I2C_DEVICE_STATUS_FAIL_UNSUPPORTED_SENSOR;
    return nullptr;
  }

  status = wippersnapper_i2c_I2cDeviceStatus_I2C_DEVICE_STATUS_SUCCESS;
  return it->second(i2c, addr, i2c_mux_channel, driver_name);
}

/*!
    @brief  I2cController constructor
*/
I2cController::I2cController() {
  _i2c_model = new I2cModel();
  _i2c_output_model = new I2cOutputModel();
  _i2c_bus_default = new I2cHardware();
}

/*!
    @brief  I2cController destructor
*/
I2cController::~I2cController() {
  if (_i2c_model)
    delete _i2c_model;

  if (_i2c_output_model)
    delete _i2c_output_model;

  if (_i2c_bus_default)
    delete _i2c_bus_default;
}

/*!
    @brief  Removes an I2C driver from the controller and frees memory
    @param    address
                The desired I2C device's address.
    @param    is_output_device
                True if the driver is an output device, False otherwise.
    @returns True if the driver was removed, False otherwise.
*/
bool I2cController::RemoveDriver(uint32_t address, bool is_output_device) {
  if (!is_output_device) {
    // Safely remove the i2c sensor driver from the vector and free memory
    for (drvBase *driver : _i2c_drivers) {
      if (driver == nullptr)
        continue;

      if (driver->GetAddress() != address)
        continue;

      auto it = std::find(_i2c_drivers.begin(), _i2c_drivers.end(), driver);
      if (it != _i2c_drivers.end()) {
        _i2c_drivers.erase(it);
      }
      delete driver;
      return true;
    }
  } else {
    // This was an output driver type, safely remove the i2c output driver from
    // the vector and free memory
    for (drvOutputBase *driver : _i2c_drivers_output) {
      if (driver == nullptr)
        continue;

      if (driver->GetAddress() != address)
        continue;

      auto it = std::find(_i2c_drivers_output.begin(),
                          _i2c_drivers_output.end(), driver);
      if (it != _i2c_drivers_output.end()) {
        _i2c_drivers_output.erase(it);
      }
      delete driver;
      return true;
    }
  }

  // We didn't find the driver to remove
  WS_DEBUG_PRINTLN("[i2c] ERROR: Unable to find driver to remove!");
  return false;
}

/*!
    @brief    Returns if the I2C bus has been created successfully.
    @param    is_alt_bus
              True if the alt. I2C bus is being queried, False otherwise.
    @returns  True if the I2C bus has already been created, False otherwise.
*/
bool I2cController::IsBusStatusOK(bool is_alt_bus) {
  bool is_ok = false;
  if (is_alt_bus) {
    is_ok = (_i2c_bus_alt->GetBusStatus() ==
             wippersnapper_i2c_I2cBusStatus_I2C_BUS_STATUS_SUCCESS);
  } else {
    is_ok = (_i2c_bus_default->GetBusStatus() ==
             wippersnapper_i2c_I2cBusStatus_I2C_BUS_STATUS_SUCCESS);
  }
  return is_ok;
}

/*!
    @brief    Publishes an I2cDeviceAddedorReplaced message to the broker
    @param    device_descriptor
                The I2cDeviceDescriptor message.
    @param    device_status
                The I2cDeviceStatus message.
    @returns  True if the I2cDeviceAddedorReplaced message was published
              successfully, False otherwise.
*/
bool I2cController::PublishI2cDeviceAddedorReplaced(
    const wippersnapper_i2c_I2cDeviceDescriptor &device_descriptor,
    const wippersnapper_i2c_I2cDeviceStatus &device_status) {
  // If we're in offline mode, don't publish out to IO
  if (WsV2._sdCardV2->isModeOffline())
    return true; // Back out if we're in offline mode
  
  // Encode the I2cDeviceAddedorReplaced message and publish it to IO
    if (!_i2c_model->encodeMsgI2cDeviceAddedorReplaced(
          device_descriptor, _i2c_bus_default->GetBusStatus(), device_status)) {
    WS_DEBUG_PRINTLN(
        "[i2c] ERROR: Unable to encode I2cDeviceAddedorReplaced message!");
    return false;
  }
  if (!WsV2.PublishSignal(
          wippersnapper_signal_DeviceToBroker_i2c_device_added_replaced_tag,
          _i2c_model->GetMsgI2cDeviceAddedOrReplaced())) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: Unable to publish I2cDeviceAddedorReplaced "
                     "message to IO!");
    return false;
  }
  return true;
}

/*!
    @brief    Implements handling for a I2cDeviceRemove message
    @param    stream
                A pointer to the pb_istream_t stream.
    @returns  True if the I2cDeviceRemove message was handled, False
              otherwise.
*/
bool I2cController::Handle_I2cDeviceRemove(pb_istream_t *stream) {
  // Attempt to decode an I2cDeviceRemove message
  WS_DEBUG_PRINTLN("[i2c] Decoding I2cDeviceRemove message...");
  if (!_i2c_model->DecodeI2cDeviceRemove(stream)) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: Unable to decode I2cDeviceRemove message!");
    return false;
  }

  // TODO [Online]: Implement the rest of this function
  // TODO: Remember - can be on either bus! (default or alt)
  // TODO: Remember to handle removal of a mux device or a device on a mux
  // strlen(descriptor.i2c_bus_sda) == 0

  wippersnapper_i2c_I2cDeviceRemove *msgRemove =
      _i2c_model->GetI2cDeviceRemoveMsg();
  if (!msgRemove->has_i2c_device_description) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: I2cDeviceRemove message missing required "
                     "device description!");
    return false;
  }

  bool did_remove = true;

  // Check for default bus
  if (strlen(msgRemove->i2c_device_description.i2c_bus_scl) == 0 &&
      strlen(msgRemove->i2c_device_description.i2c_bus_sda) == 0) {
    WS_DEBUG_PRINTLN("[i2c] Removing device from default bus...");
    if (!_i2c_bus_default->HasMux()) {
      if (!RemoveDriver(msgRemove->i2c_device_description.i2c_device_address,
                        msgRemove->is_output_device)) {
        WS_DEBUG_PRINTLN(
            "[i2c] ERROR: Failed to remove i2c device from default bus!");
        did_remove = false;
      }
    } else {
      // Bus has a I2C MUX attached
      // Case 1: Is the I2C device connected to a MUX?
      if (msgRemove->i2c_device_description.i2c_mux_address != 0xFFFF &&
          msgRemove->i2c_device_description.i2c_mux_channel >= 0) {
        _i2c_bus_default->SelectMuxChannel(
            msgRemove->i2c_device_description.i2c_mux_channel);
        if (!RemoveDriver(msgRemove->i2c_device_description.i2c_device_address,
                          msgRemove->is_output_device)) {
          WS_DEBUG_PRINTLN(
              "[i2c] ERROR: Failed to remove i2c device from default bus!");
          did_remove = false;
        }
      }
      // Case 2: Is the I2C device a MUX?
      if (msgRemove->i2c_device_description.i2c_device_address ==
          msgRemove->i2c_device_description.i2c_mux_address) {
        wippersnapper_i2c_I2cBusScanned scan_results;
        _i2c_bus_default->ScanMux(&scan_results);
        for (int i = 0; i < scan_results.i2c_bus_found_devices_count; i++) {
          // Select the channel and remove the device
          _i2c_bus_default->SelectMuxChannel(
              scan_results.i2c_bus_found_devices[i].i2c_mux_channel);
          RemoveDriver(scan_results.i2c_bus_found_devices[i].i2c_device_address,
                       msgRemove->is_output_device);
        }
        _i2c_bus_default->RemoveMux();
      }
    }
  }

  // TODO: Check for Alt. I2C Bus

  // Publush with did_remove to the response

  return true;
}

/*!
    @brief    Attempts to initialize a MUX on the bus.
    @param    name
                The name of the MUX to initialize, used to set number
                of channels.
    @param    address
                The MUX's I2C address.
    @param    is_alt_bus
                True if the alternative I2C bus is being used, False
                otherwise.
    @returns  True if the MUX was successfully initialized, False
                otherwise.
*/
bool I2cController::InitMux(const char *name, uint32_t address,
                            bool is_alt_bus) {
  if (is_alt_bus) {
    if (!_i2c_bus_alt->HasMux()) {
      if (!_i2c_bus_alt->AddMuxToBus(address, name)) {
        return false;
      }
    }
  } else {
    if (!_i2c_bus_default->HasMux()) {
      if (!_i2c_bus_default->AddMuxToBus(address, name)) {
        return false;
      }
    }
  }
  // TODO [Online]: Publish back out to IO here!
  return true;
}

/*!
    @brief   Configures the MUX channel on the bus.
    @param   stream
                Pointer to the pb_istream
    @returns True if the I2C bus was successfully scanned and the
             I2cBusScan message was published to IO, False otherwise.
*/
bool I2cController::Handle_I2cBusScan(pb_istream_t *stream) {
  WS_DEBUG_PRINTLN("[i2c] Decoding I2cDeviceAddOrReplace message...");
  // Attempt to decode I2cBusScan message
  if (!_i2c_model->DecodeI2cBusScan(stream)) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: Unable to decode I2cBusScan message!");
    return false;
  }

  _i2c_model->ClearI2cBusScanned();
  wippersnapper_i2c_I2cBusScanned *scan_results =
      _i2c_model->GetI2cBusScannedMsg();

  bool scan_success = true;
  // Case 1: Scan the default I2C bus
  if (_i2c_model->GetI2cBusScanMsg()->scan_default_bus) {
    // Was the default bus initialized correctly and ready to scan?
    WS_DEBUG_PRINT("Bus State: ");
    WS_DEBUG_PRINTLN(_i2c_bus_default->GetBusStatus());
    WS_DEBUG_PRINTLN(IsBusStatusOK());
    if (IsBusStatusOK()) {
      if (!_i2c_bus_default->ScanBus(scan_results)) {
        WS_DEBUG_PRINTLN("[i2c] ERROR: Failed to scan default I2C bus!");
        scan_success = false;
      }
    } else {
      WS_DEBUG_PRINTLN("[i2c] ERROR: Default I2C bus state is stuck, please "
                       "reset the board!");
      scan_success = false;
    }
  }

  // Case 2: Optionally scan the alternative I2C bus
  if (_i2c_model->GetI2cBusScanMsg()->scan_alt_bus) {
    // Is the alt bus initialized?
    if (_i2c_bus_alt == nullptr) {
      _i2c_bus_alt = new I2cHardware(
          _i2c_model->GetI2cBusScanMsg()->i2c_alt_bus_descriptor.i2c_bus_sda,
          _i2c_model->GetI2cBusScanMsg()->i2c_alt_bus_descriptor.i2c_bus_sda);
      // Was the default bus initialized correctly and ready to scan?
      if (IsBusStatusOK(true)) {
        if (!_i2c_bus_alt->ScanBus(scan_results)) {
          WS_DEBUG_PRINTLN("[i2c] ERROR: Failed to scan alt. I2C bus!");
          scan_success = false;
        }
      } else {
        WS_DEBUG_PRINTLN("[i2c] ERROR: alt. I2C bus state is stuck, please "
                         "reset the board!");
        scan_success = false;
      }
    }
  }

  // Case 3: Optionally scan MUX attached to the default bus
  if (_i2c_model->GetI2cBusScanMsg()->scan_default_bus_mux) {
    if (_i2c_bus_default->HasMux()) {
      if (!_i2c_bus_default->ScanMux(scan_results)) {
        WS_DEBUG_PRINTLN("[i2c] ERROR: Failed to scan I2C MUX on default bus!");
        scan_success = false;
      }
    }
  }

  // Case 4: Optionally scan MUX attached to the alt. bus
  if (_i2c_model->GetI2cBusScanMsg()->scan_alt_bus) {
    if (_i2c_bus_alt->HasMux()) {
      if (!_i2c_bus_alt->ScanMux(scan_results)) {
        WS_DEBUG_PRINTLN("[i2c] ERROR: Failed to scan I2C MUX on alt. bus!");
        scan_success = false;
      }
    }
  }

  // Printout content of scan_results
  WS_DEBUG_PRINT("[i2c] Scan found ");
  WS_DEBUG_PRINT(scan_results->i2c_bus_found_devices_count);
  WS_DEBUG_PRINTLN(" devices.");
  for (int i = 0; i < scan_results->i2c_bus_found_devices_count; i++) {
    WS_DEBUG_PRINTLN(i);
    WS_DEBUG_PRINT("Address: ");
    WS_DEBUG_PRINTLN(scan_results->i2c_bus_found_devices[i].i2c_device_address,
                     HEX);
    WS_DEBUG_PRINT("SCL: ");
    WS_DEBUG_PRINTLN(scan_results->i2c_bus_found_devices[i].i2c_bus_scl);
    WS_DEBUG_PRINT("SDA: ");
    WS_DEBUG_PRINTLN(scan_results->i2c_bus_found_devices[i].i2c_bus_sda);
    WS_DEBUG_PRINT("MUX Address: ");
    WS_DEBUG_PRINTLN(scan_results->i2c_bus_found_devices[i].i2c_mux_address);
    WS_DEBUG_PRINT("MUX Channel: ");
    WS_DEBUG_PRINTLN(scan_results->i2c_bus_found_devices[i].i2c_mux_channel);
  }

  // TODO: Encode and publish out to IO!
  // TODO: Take scan_success into account here
  return true;
}

/*!
    @brief    Handler for an I2cDeviceOutputWrite message
    @param    stream
                A pointer to the pb_istream_t stream.
    @returns  True if the callback was successfully executed by the driver,
   False otherwise.
*/
bool I2cController::Handle_I2cDeviceOutputWrite(pb_istream_t *stream) {
  WS_DEBUG_PRINTLN("[i2c] Decoding I2cDeviceOutputWrite message...");
  // Attempt to decode an I2cDeviceOutputWrite message
  if (!_i2c_model->DecodeI2cDeviceOutputWrite(stream)) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: Unable to decode I2cDeviceOutputWrite "
                     "message!");
    return false;
  }
  wippersnapper_i2c_I2cDeviceDescriptor descriptor =
      _i2c_model->GetI2cDeviceOutputWriteMsg()->i2c_device_description;

  // Attempt to find the driver
  drvOutputBase *driver = nullptr;
  for (auto *drv : _i2c_drivers_output) {
    if (drv == nullptr)
      continue;

    if (drv->GetAddress() != descriptor.i2c_device_address)
      continue;

    driver = drv;
    break;
  }

  if (driver == nullptr) {
    WS_DEBUG_PRINT("[i2c] ERROR: Unable to find driver for device at addr 0x");
    WS_DEBUG_PRINTLN(descriptor.i2c_device_address, HEX);
    return false;
  }

  // Optionally configure the I2C MUX
  uint32_t mux_channel = driver->GetMuxChannel();
  WS_DEBUG_PRINTLN(mux_channel);
  if (driver->HasMux()) {
    ConfigureMuxChannel(mux_channel, driver->HasAltI2CBus());
  }

  // Determine which driver cb function to use
  if (_i2c_model->GetI2cDeviceOutputWriteMsg()->which_output_msg ==
      wippersnapper_i2c_I2cDeviceOutputWrite_write_led_backpack_tag) {
    WS_DEBUG_PRINTLN("[i2c] Writing to LED backpack...");
    driver->WriteMessage(_i2c_model->GetI2cDeviceOutputWriteMsg()
                             ->output_msg.write_led_backpack.message);
  } else if (_i2c_model->GetI2cDeviceOutputWriteMsg()->which_output_msg ==
             wippersnapper_i2c_I2cDeviceOutputWrite_write_char_lcd_tag) {
    WS_DEBUG_PRINTLN("[i2c] Writing to char LCD...");
    if (!driver->WriteMessageCharLCD(&_i2c_model->GetI2cDeviceOutputWriteMsg()
                                          ->output_msg.write_char_lcd)) {
      WS_DEBUG_PRINTLN("[i2c] ERROR: Unable to write to char LCD!");
      return false;
    }
  } else if (_i2c_model->GetI2cDeviceOutputWriteMsg()->which_output_msg ==
             wippersnapper_i2c_I2cDeviceOutputWrite_write_oled_tag) {
    WS_DEBUG_PRINTLN("[i2c] Writing to SSD1306 OLED...");
    // Note: In the future, we can expand this to support other OLEDs by
    // creating and checking a tag within the write oled msg (e.g. SSD1327,
    // etc.)
    driver->WriteMessageSSD1306(_i2c_model->GetI2cDeviceOutputWriteMsg()
                                    ->output_msg.write_oled.message);
  } else {
    WS_DEBUG_PRINTLN("[i2c] ERROR: Unable to determine I2C Output Write type!");
    return false;
  }

  return true;
}

/*!
    @brief    Implements handling for a I2cDeviceAddOrReplace message
    @param    stream
                A pointer to the pb_istream_t stream.
    @returns  True if the I2cDeviceAddOrReplace message was handled
              (created or replaced), False otherwise.
*/
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
  // Parse out device name and descriptor
  char device_name[15];
  strcpy(device_name,
         _i2c_model->GetI2cDeviceAddOrReplaceMsg()->i2c_device_name);
  wippersnapper_i2c_I2cDeviceDescriptor device_descriptor =
      _i2c_model->GetI2cDeviceAddOrReplaceMsg()->i2c_device_description;

  // Is this an i2c output device?
  bool is_output = _i2c_model->GetI2cDeviceAddOrReplaceMsg()->is_output;

  // Is this a i2c GPS?
  bool is_gps = _i2c_model->GetI2cDeviceAddOrReplaceMsg()->is_gps;

  // TODO [Online]: Handle Replace messages by implementing the Remove handler
  // first...then proceed to adding a new device

  // Does the device's descriptor specify a different i2c bus?
  if (strcmp(device_descriptor.i2c_bus_scl, "default") != 0) {
    WS_DEBUG_PRINTLN("[i2c] Non-default I2C bus specified!");
    if (_i2c_bus_alt == nullptr) {
      WS_DEBUG_PRINTLN("[i2c] Initializing alternative i2c bus...");
      _i2c_bus_alt = new I2cHardware(device_descriptor.i2c_bus_sda,
                                     device_descriptor.i2c_bus_scl);
    }
    use_alt_bus = true;
  }

  // Before we do anything on the bus - was the bus initialized correctly?
  if (!IsBusStatusOK(use_alt_bus)) {
    WS_DEBUG_PRINTLN(
        "[i2c] I2C bus is stuck or not operational, reset the board!");
    if (WsV2._sdCardV2->isModeOffline()) {
      WsV2.haltErrorV2(" ", WS_LED_STATUS_ERROR_RUNTIME,
                       false); // doesn't return, halts
    }
    if (!PublishI2cDeviceAddedorReplaced(device_descriptor, device_status)) {
      WS_DEBUG_PRINTLN("[i2c] ERROR: Unable to publish message to IO!");
      return false;
    }
    return true;
  }

  // I2C MUX (Case #1) - We are creating an I2C mux via the
  // I2cDeviceAddorReplace message
  if ((strcmp(device_name, "pca9546") == 0) ||
      (strcmp(device_name, "pca9548") == 0)) {
    WS_DEBUG_PRINT("[i2c] Initializing MUX driver...");
    if (!InitMux(device_name, device_descriptor.i2c_mux_address, use_alt_bus)) {
      // TODO [Online]: Publish back out to IO here!
      WsV2.haltErrorV2("[i2c] Failed to initialize MUX driver!",
                       WS_LED_STATUS_ERROR_RUNTIME, false);
    }
    WS_DEBUG_PRINTLN("OK!");
    return true;
  }

  // Mux case #2 - We are creating a new driver that USES THE MUX via
  // I2cDeviceAddorReplace message
  if (device_descriptor.i2c_mux_address != 0x00) {
    if (_i2c_bus_alt->HasMux() || _i2c_bus_default->HasMux()) {
      WS_DEBUG_PRINT("[i2c] Configuring MUX channel: ");
      WS_DEBUG_PRINTLN(device_descriptor.i2c_mux_channel);
      ConfigureMuxChannel(device_descriptor.i2c_mux_channel, use_alt_bus);
      did_set_mux_ch = true;
    } else {
      WsV2.haltErrorV2("[i2c] Device requires a MUX but MUX not present "
                       "within config.json!",
                       WS_LED_STATUS_ERROR_RUNTIME, false);
    }
  }

  WS_DEBUG_PRINTLN("Creating a new I2C driver");
  // Assign I2C bus
  TwoWire *bus = nullptr;
  if (use_alt_bus) {
    bus = _i2c_bus_alt->GetBus();
  } else {
    bus = _i2c_bus_default->GetBus();
  }

  // Attempt to create the driver
  bool did_init = false;
  drvBase *drv = nullptr;
  drvOutputBase *drv_out = nullptr;
  GPSController *drv_uart_gps = nullptr;

  if (is_output) {
    WS_DEBUG_PRINT("[i2c] Creating an I2C output driver...");
    drv_out = CreateI2cOutputDrv(
        device_name, bus, device_descriptor.i2c_device_address,
        device_descriptor.i2c_mux_channel, device_status);
    if (drv_out != nullptr) {
      did_init = true;
    }
    WS_DEBUG_PRINTLN("OK!");
  } else if (is_gps) {
    WS_DEBUG_PRINT("[i2c] Creating a GPS driver...");
    if (!WsV2._gps_controller->AddGPS(bus, device_descriptor.i2c_device_address, &_i2c_model->GetI2cDeviceAddOrReplaceMsg()->gps_config)) {
      did_init = false;
      WS_DEBUG_PRINTLN("FAILURE!");
    } else {
      did_init = true;
      WS_DEBUG_PRINTLN("OK!");
      // TODO: We are doing an early-out here and should publish back to IO!
      return true;
    }
  } else {
    drv = CreateI2cSensorDrv(device_name, bus,
                             device_descriptor.i2c_device_address,
                             device_descriptor.i2c_mux_channel, device_status);
    if (drv != nullptr) {
      did_init = true;
    }
  }

  if (!did_init) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: I2C driver failed to initialize!");
    if (WsV2._sdCardV2->isModeOffline()) {
      WsV2.haltErrorV2("[i2c] Driver failed to initialize!\n\tDid you set "
                       "the correct value for i2cDeviceName?\n\tDid you set "
                       "the correct value for"
                       "i2cDeviceAddress?",
                       WS_LED_STATUS_ERROR_RUNTIME, false);
    }
  }

  // Attempt to initialize the driver
  if (did_set_mux_ch) {
    if (!is_output) {
      drv->SetMuxAddress(device_descriptor.i2c_mux_address);
    } else {
      WS_DEBUG_PRINTLN("[i2c] Setting MUX address for output driver...");
      drv_out->SetMuxAddress(device_descriptor.i2c_mux_address);
    }
    WS_DEBUG_PRINTLN("[i2c] Set driver to use MUX");
  }
  if (use_alt_bus) {
    if (!is_output) {
      drv->EnableAltI2CBus(_i2c_model->GetI2cDeviceAddOrReplaceMsg()
                               ->i2c_device_description.i2c_bus_scl,
                           _i2c_model->GetI2cDeviceAddOrReplaceMsg()
                               ->i2c_device_description.i2c_bus_sda);
    } else {
      WS_DEBUG_PRINTLN("[i2c] Setting alt. I2C bus for output driver...");
      drv_out->EnableAltI2CBus(_i2c_model->GetI2cDeviceAddOrReplaceMsg()
                                   ->i2c_device_description.i2c_bus_scl,
                               _i2c_model->GetI2cDeviceAddOrReplaceMsg()
                                   ->i2c_device_description.i2c_bus_sda);
    }
    WS_DEBUG_PRINTLN("[i2c] Set driver to use Alt I2C bus");
  }
  // Configure the driver

  if (!is_output) {
    // Configure Input-driver settings
    drv->EnableSensorReads(
        _i2c_model->GetI2cDeviceAddOrReplaceMsg()->i2c_device_sensor_types,
        _i2c_model->GetI2cDeviceAddOrReplaceMsg()
            ->i2c_device_sensor_types_count);
    drv->SetSensorPeriod(
        _i2c_model->GetI2cDeviceAddOrReplaceMsg()->i2c_device_period);
  } else {
    WS_DEBUG_PRINTLN("[i2c] Configuring output driver...");
    // Configure Output-driver settings
    pb_size_t config =
        _i2c_model->GetI2cDeviceAddOrReplaceMsg()->i2c_output_add.which_config;
    if (config ==
        wippersnapper_i2c_output_I2cOutputAdd_led_backpack_config_tag) {
      WS_DEBUG_PRINTLN("[i2c] Configuring LED backpack...");
      wippersnapper_i2c_output_LedBackpackConfig cfg =
          _i2c_model->GetI2cDeviceAddOrReplaceMsg()
              ->i2c_output_add.config.led_backpack_config;
      WS_DEBUG_PRINT("[i2c] Got cfg, calling ConfigureI2CBackpack...");
      drv_out->ConfigureI2CBackpack(cfg.brightness, cfg.alignment);
      WS_DEBUG_PRINTLN("OK!");
    } else if (config ==
               wippersnapper_i2c_output_I2cOutputAdd_char_lcd_config_tag) {
      WS_DEBUG_PRINTLN("[i2c] Configuring char LCD...");
    } else if (config ==
               wippersnapper_i2c_output_I2cOutputAdd_oled_config_tag) {
      WS_DEBUG_PRINTLN("[i2c] Configuring OLED...");
      wippersnapper_i2c_output_OledConfig cfg =
          _i2c_model->GetI2cDeviceAddOrReplaceMsg()
              ->i2c_output_add.config.oled_config;
      WS_DEBUG_PRINT("[i2c] Got cfg, calling ConfigureOLED...");
      drv_out->ConfigureSSD1306(cfg.width, cfg.height, cfg.font_size);
      WS_DEBUG_PRINTLN("OK!");
    } else {
      WS_DEBUG_PRINTLN(
          "[i2c] ERROR: Unknown config specified for output driver!");
      return false;
    }
  }

  if (!is_output) {
    if (!drv->begin()) {
      if (WsV2._sdCardV2->isModeOffline()) {
        WsV2.haltErrorV2("[i2c] Driver failed to initialize!\n\tDid you set "
                         "the correct value for i2cDeviceName?\n\tDid you set "
                         "the correct value for"
                         "i2cDeviceAddress?",
                         WS_LED_STATUS_ERROR_RUNTIME, false);
      }
    }
    _i2c_drivers.push_back(drv);
  } else {
    if (!drv_out->begin()) {
      if (WsV2._sdCardV2->isModeOffline()) {
        WsV2.haltErrorV2("[i2c] Driver failed to initialize!\n\tDid you set "
                         "the correct value for i2cDeviceName?\n\tDid you set "
                         "the correct value for"
                         "i2cDeviceAddress?",
                         WS_LED_STATUS_ERROR_RUNTIME, false);
      }
    }
    _i2c_drivers_output.push_back(drv_out);
  }

  WS_DEBUG_PRINTLN("[i2c] Driver initialized and added to controller: ");
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

  if (WsV2._sdCardV2->isModeOffline() != true) {
    // Create and publish the I2cDeviceAddedorReplaced message to the broker
    WS_DEBUG_PRINTLN("[i2c] MQTT Publish I2cDeviceAddedorReplaced not yet "
                     "implemented!");
    // TODO!
    /*     if (!PublishI2cDeviceAddedorReplaced(device_descriptor,
       device_status)) return false; */
  }

  return true;
}

/*!
    @brief    Enables a MUX channel on the appropriate I2C bus.
    @param    mux_channel
                Desired MUX channel to enable
    @param    is_alt_bus
                True if an alternative I2C bus is being used, False otherwise.
*/
void I2cController::ConfigureMuxChannel(uint32_t mux_channel, bool is_alt_bus) {
  if (is_alt_bus) {
    _i2c_bus_alt->ClearMuxChannel(); // sanity-check
    _i2c_bus_alt->SelectMuxChannel(mux_channel);
    return;
  }
  _i2c_bus_default->ClearMuxChannel(); // sanity-check
  _i2c_bus_default->SelectMuxChannel(mux_channel);
}

/*!
    @brief    Handles polling, reading, and logger for i2c devices
              attached to the I2C controller.
*/
void I2cController::update() {
  // WS_DEBUG_PRINTLN("[i2c] Updating I2C controller...");
  if (_i2c_drivers.empty())
    return; // bail out if no drivers exist

  for (auto *drv : _i2c_drivers) {
    // Does this driver have any enabled sensors?
    size_t sensor_count = drv->GetEnabledSensorCnt();
    if (sensor_count == 0)
      continue; // bail out if driver has no sensors enabled

    // Did driver's period elapse yet?
    ulong cur_time = millis();
    if (cur_time - drv->GetSensorPeriodPrv() < drv->GetSensorPeriod()) {
      continue; // bail out if the period hasn't elapsed yet
    }

    // Optionally configure the I2C MUX
    uint32_t mux_channel = drv->GetMuxChannel();
    WS_DEBUG_PRINTLN(mux_channel);
    if (drv->HasMux())
      ConfigureMuxChannel(mux_channel, drv->HasAltI2CBus());

    // Read the driver's sensors
    _i2c_model->ClearI2cDeviceEvent();
    for (size_t i = 0; i < sensor_count; i++) {
      sensors_event_t event = {0};
      // Attempt to call driver's read handler function
      if (!drv->GetSensorEvent(drv->_sensors[i], &event)) {
        WS_DEBUG_PRINTLN("[i2c] ERROR: Failed to read sensor!");
        continue;
      }
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
