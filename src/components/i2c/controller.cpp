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
/*******************************************************************************/
using FnCreateI2CDriver =
    std::function<drvBase *(TwoWire *, uint16_t, uint32_t, const char *)>;

// Factory for creating a new I2C drivers
// NOTE: When you add a new driver, make sure to add it to the factory!
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
    {"htu31d",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvHtu31d(i2c, addr, mux_channel, driver_name);
     }},
    {"hdc302x",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvHdc302x(i2c, addr, mux_channel, driver_name);
     }},
    {"ina219",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvIna219(i2c, addr, mux_channel, driver_name);
     }},
    {"ina260",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvIna260(i2c, addr, mux_channel, driver_name);
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
    {"lps28dfw",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvLps28dfw(i2c, addr, mux_channel, driver_name);
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
    {"sgp30",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvSgp30(i2c, addr, mux_channel, driver_name);
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
    {"sen60",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvSen6x(i2c, addr, mux_channel, driver_name);
     }},
    {"sen63c",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvSen6x(i2c, addr, mux_channel, driver_name);
     }},
    {"sen65",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvSen6x(i2c, addr, mux_channel, driver_name);
     }},
    {"sen66",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvSen6x(i2c, addr, mux_channel, driver_name);
     }},
    {"sen68",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvSen6x(i2c, addr, mux_channel, driver_name);
     }},
    {"sen6x",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvSen6x(i2c, addr, mux_channel, driver_name);
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
    {"vncl4200",
     [](TwoWire *i2c, uint16_t addr, uint32_t mux_channel,
        const char *driver_name) -> drvBase * {
       return new drvVncl4200(i2c, addr, mux_channel, driver_name);
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

static const std::unordered_map<uint16_t, std::vector<const char *>>
    map_address_to_drivers = {
        {0x0B, {"lc709203f"}},
        {0x12, {"pmsa003i"}},
        {0x13, {"vncl4020"}},
        {0x18, {"ds2484", "mcp9808", "mprls"}},
        {0x19, {"mcp9808"}},
        {0x1A, {"mcp9808"}},
        {0x1B, {"mcp9808"}},
        {0x1C, {"mcp9808"}},
        {0x1D, {"mcp9808"}},
        {0x1E, {"mcp9808"}},
        {0x1F, {"mcp9808"}},
        {0x23, {"bh1750"}},
        {0x28, {"pct2075"}},
        {0x29,
         {"ltr303", "pct2075", "tsl2591", "veml7700", "vl53l1x", "vl53l4cd",
          "vl53l4cx", "vl6180x"}},
        {0x2A, {"nau7802"}},
        {0x38, {"aht20", "max17048"}},
        {0x39, {"tsl2591"}},
        {0x40, {"htu21d", "ina219", "ina260", "ms8607", "si7021", "stemma_soil"}},
        {0x41, {"ina219", "ina260"}},
        {0x44, {"ina260", "sht3x", "sht4x"}},
        {0x45, {"ina260", "sht3x"}},
        {0x48, {"adt7410", "pct2075", "tmp117"}},
        {0x49, {"adt7410", "pct2075", "tmp117", "tsl2591"}},
        {0x4A, {"adt7410", "pct2075", "tmp117"}},
        {0x4B, {"adt7410", "pct2075", "tmp117"}},
        {0x4C, {"pct2075"}},
        {0x4D, {"pct2075"}},
        {0x4E, {"pct2075"}},
        {0x4F, {"pct2075"}},
        {0x51, {"vcnl4200"}},
        {0x52, {"ens160"}},
        {0x53, {"ens160", "ltr390"}},
        {0x59, {"sgp40"}},
        {0x5C, {"bh1750", "lps22hb", "lps25hb"}},
        {0x5D, {"lps22hb", "lps25hb"}},
        {0x5F, {"hts2221"}},
        {0x60, {"mpl115a2", "vncl4040"}},
        {0x61, {"scd30"}},
        {0x62, {"scd40"}},
        {0x68, {"mcp3421"}},
        {0x69, {"sen50"}},
        {0x6B, {"sen66"}},
        {0x70, {"pct2075", "shtc3"}},
        {0x71, {"pct2075"}},
        {0x72, {"pct2075"}},
        {0x73, {"pct2075"}},
        {0x74, {"pct2075"}},
        {0x75, {"pct2075"}},
        {0x76,
         {"bme280", "bme680", "bmp280", "bmp388", "bmp390", "dps310", "ms8607",
          "pct2075"}},
        {0x77,
         {"bme280", "bme680", "bmp280", "bmp388", "bmp390", "dps310",
          "pct2075"}}}; ///< I2C address to driver map

/***********************************************************************/
/*!
    @brief  Obtains possible candidate drivers for a given I2C address.
    @param    addr
                The desired I2C address.
    @returns  A vector of pointers to candidate drivers.
*/
/***********************************************************************/
std::vector<const char *> GetDriversForAddress(uint16_t addr) {
  std::vector<const char *> candidates;
  std::unordered_map<uint16_t, std::vector<const char *>>::const_iterator
      candidate = map_address_to_drivers.find(addr);

  if (candidate != map_address_to_drivers.end()) {
    candidates = candidate->second;
  }

  return candidates;
}

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
drvBase *CreateI2CDriverByName(const char *driver_name, TwoWire *i2c,
                               uint16_t addr, uint32_t i2c_mux_channel,
                               wippersnapper_i2c_I2cDeviceStatus &status) {
  auto it = I2cFactory.find(driver_name);
  if (it == I2cFactory.end()) {
    status =
        wippersnapper_i2c_I2cDeviceStatus_I2C_DEVICE_STATUS_FAIL_UNSUPPORTED_SENSOR;
    return nullptr;
  }

  status = wippersnapper_i2c_I2cDeviceStatus_I2C_DEVICE_STATUS_SUCCESS;
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
  // Initialize the default I2C bus
  _i2c_bus_default = new I2cHardware();
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
    @brief    Returns if the I2C bus has been created successfully.
    @param    is_alt_bus
              True if the alt. I2C bus is being queried, False otherwise.
    @returns  True if the I2C bus has already been created, False otherwise.
*/
/*************************************************************************/
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

  // TODO [Online]: Implement the rest of this function
  WS_DEBUG_PRINTLN("[i2c] I2cDeviceRemove message not yet implemented!");

  return true;
}

/***********************************************************************/
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
/***********************************************************************/
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

/***********************************************************************/
/*!
    @brief    Checks if a driver has already been initialized with the
              given device descriptor.
    @param    device_descriptor
                The I2cDeviceDescriptor message.
    @returns  True if a driver has already been initialized, False
              otherwise.
*/
/***********************************************************************/
bool I2cController::IsDriverInitialized(
    wippersnapper_i2c_I2cDeviceDescriptor &device_descriptor) {
  // Before we do anything, check if a driver has been already initialized with
  // the device_descriptor if so, we log and skip
  for (auto &driver : _i2c_drivers) {
    // Do they share the same address?
    if (driver->GetAddress() == device_descriptor.i2c_device_address) {
      // Okay - do they sit on different i2c buses?
      bool is_driver_bus_alt = driver->HasAltI2CBus();
      bool is_device_bus_alt =
          (strcmp(device_descriptor.i2c_bus_scl, "default") != 0) ||
          (strcmp(device_descriptor.i2c_bus_sda, "default") != 0);
      // Bus descriptors do not match, so we haven't initialized this candidate
      if (is_driver_bus_alt != is_device_bus_alt)
        continue;

      // What about the MUX?
      if (driver->HasMux() &&
          driver->GetMuxAddress() == device_descriptor.i2c_mux_address &&
          driver->GetMuxChannel() != device_descriptor.i2c_mux_channel) {
        continue;
      }

      WS_DEBUG_PRINTLN("[i2c] Descriptor already initialized...");
      return true;
    }
  }
  return false;
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
  // Parse out device name and descriptor
  char device_name[15];
  strcpy(device_name,
         _i2c_model->GetI2cDeviceAddOrReplaceMsg()->i2c_device_name);
  wippersnapper_i2c_I2cDeviceDescriptor device_descriptor =
      _i2c_model->GetI2cDeviceAddOrReplaceMsg()->i2c_device_description;

  // Did the driver initialize correctly?
  if (IsDriverInitialized(device_descriptor)) {
    WS_DEBUG_PRINTLN("[i2c] Driver already initialized, skipping...");
    return true;
  }

  // TODO [Online]: Handle Replace messages by implementing the Remove handler
  // first...then proceed to adding a new device

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
    if (!InitMux(device_name, device_descriptor.i2c_device_address,
                 use_alt_bus)) {
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
      ConfigureMuxChannel(device_descriptor.i2c_mux_channel, use_alt_bus);
      did_set_mux_ch = true;
    } else {
      WsV2.haltErrorV2("[i2c] Device requires a MUX but MUX not present "
                       "within config.json!",
                       WS_LED_STATUS_ERROR_RUNTIME, false);
    }
  }

  // Assign I2C bus
  TwoWire *bus = nullptr;
  if (use_alt_bus) {
    bus = _i2c_bus_alt->GetBus();
  } else {
    bus = _i2c_bus_default->GetBus();
  }

  drvBase *drv = nullptr;
  WS_DEBUG_PRINTLN("[i2c] Creating driver: ");
  WS_DEBUG_PRINTLN(device_name);
  if (strcmp(device_name, SCAN_DEVICE) == 0) {
    WS_DEBUG_PRINTLN("Attempting to autoconfig device found in scan...");
    // Get all possible driver candidates for this address
    WS_DEBUG_PRINT("[i2c] Obtaining driver candidates @ 0x");
    WS_DEBUG_PRINTLN(device_descriptor.i2c_device_address, HEX);
    if (device_descriptor.i2c_device_address == 0x68 ||
        device_descriptor.i2c_device_address == 0x70) {
      WS_DEBUG_PRINTLN("[i2c] Device address is shared with RTC/MUX, can not "
                       "auto-init, skipping!");
      return true;
    }

    std::vector<const char *> candidate_drivers =
        GetDriversForAddress(device_descriptor.i2c_device_address);

    // Probe each candidate to see if it communicates
    bool did_find_driver = false;
    for (const char *driverName : candidate_drivers) {
      WS_DEBUG_PRINT("[i2c] Attempting to initialize candidate: ");
      WS_DEBUG_PRINTLN(driverName);
      drv = CreateI2CDriverByName(
          driverName, bus, device_descriptor.i2c_device_address,
          device_descriptor.i2c_mux_channel, device_status);
      // Probe the driver to check if it communicates its init. sequence
      // properly
      if (!drv->begin()) {
        delete drv;
        drv = nullptr;
      } else {
        WS_DEBUG_PRINT("[i2c] Successfully initialized candidate: ");
        WS_DEBUG_PRINTLN(driverName);
        // set device_name to driverName
        strcpy(device_name, driverName);
        // Use the "default" types from the sensor driver
        drv->SetSensorTypes(true);
        drv->SetPeriod(DEFAULT_SENSOR_PERIOD);
#ifndef OFFLINE_MODE_WOKWI
        WsV2._fileSystemV2->AddI2cDeviceToFileConfig(
            device_descriptor.i2c_device_address, driverName,
            drv->GetSensorTypeStrings(), drv->GetNumSensorTypes());
#endif
        did_find_driver = true;
        break;
      }
    }
    if (!did_find_driver) {
      WS_DEBUG_PRINTLN("[i2c] ERROR - Candidates exhausted, driver not found!");
      return true; // dont cause an error in the app
    }
  } else {
    WS_DEBUG_PRINTLN("[i2c] Device in message/cfg file.");
    // Create new driver
    WS_DEBUG_PRINT("[i2c] Creating driver: ");
    WS_DEBUG_PRINTLN(device_name);
    drv = CreateI2CDriverByName(
        device_name, bus, device_descriptor.i2c_device_address,
        device_descriptor.i2c_mux_channel, device_status);
    if (drv == nullptr) {
      WS_DEBUG_PRINTLN(
          "[i2c] ERROR: I2C driver type not found or unsupported!");
      return false;
    }

    // Configure MUX and bus
    if (did_set_mux_ch) {
      drv->SetMuxAddress(device_descriptor.i2c_mux_address);
    }

    if (use_alt_bus) {
      drv->EnableAltI2CBus(_i2c_model->GetI2cDeviceAddOrReplaceMsg()
                               ->i2c_device_description.i2c_bus_scl,
                           _i2c_model->GetI2cDeviceAddOrReplaceMsg()
                               ->i2c_device_description.i2c_bus_sda);
    }
    // Configure the driver
    drv->SetSensorTypes(
        false,
        _i2c_model->GetI2cDeviceAddOrReplaceMsg()->i2c_device_sensor_types,
        _i2c_model->GetI2cDeviceAddOrReplaceMsg()
            ->i2c_device_sensor_types_count);

    drv->SetPeriod(
        _i2c_model->GetI2cDeviceAddOrReplaceMsg()->i2c_device_period);

    if (!drv->begin()) {
      if (WsV2._sdCardV2->isModeOffline()) {
        WS_DEBUG_PRINTLN("[i2c] Failed to initialize driver!\n\tDid you set "
                         "the correct value for i2cDeviceName?\n\tDid you set "
                         "the correct value for"
                         "i2cDeviceAddress?");
      }
      return true; // don't cause an error during runtime if the device is not
                   // found
    }
    WS_DEBUG_PRINTLN("[i2c] Driver successfully initialized!");
  }
  // Add the initialized driver
  _i2c_drivers.push_back(drv);

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
    /*     if (!PublishI2cDeviceAddedorReplaced(device_descriptor,
       device_status)) return false; */
  }

  return true;
}

/***********************************************************************/
/*!
    @brief    Scans the I2C bus for devices and stores the results.
    @param    default_bus
                True to scan the default I2C bus, False to scan the
                alternative I2C bus.
    @returns  True if the I2C bus was successfully scanned, False
              if the scan failed with an error.
*/
/***********************************************************************/
bool I2cController::ScanI2cBus(bool default_bus = true) {
  _i2c_bus_default->InitBus(default_bus);
  _scan_results = wippersnapper_i2c_I2cBusScanned_init_zero;
  if (!default_bus)
    return _i2c_bus_alt->ScanBus(&_scan_results);
  return _i2c_bus_default->ScanBus(&_scan_results);
}

/***********************************************************************/
/*!
    @brief    Checks if a device was found on the i2c bus. MUST be called
              after scanning was performed.
    @param    address
                The desired I2C device address.
    @returns  True if the device is on the bus, False otherwise.
*/
/***********************************************************************/
bool I2cController::WasDeviceScanned(uint32_t address) {
  pb_size_t num_found_devices = _scan_results.i2c_bus_found_devices_count;
  if (num_found_devices == 0)
    return false; // no devices found on bus, or scan was not performed

  // Check if the device was found on the bus
  for (pb_size_t i; i < num_found_devices; i++) {
    if (_scan_results.i2c_bus_found_devices[i].i2c_device_address == address)
      return true; // device found on bus!
  }
  return false; // exhausted all scanned devices, didn't find it
}

/***********************************************************************/
/*!
    @brief    Returns an i2c address of a device found on the bus.
    @param    index
                The index of the scanned device within scan_results.
    @returns  The I2C device address of the scanned device.
*/
/***********************************************************************/
uint32_t I2cController::GetScanDeviceAddress(int index) {
  if (index < 0 || index >= _scan_results.i2c_bus_found_devices_count)
    return 0;
  return _scan_results.i2c_bus_found_devices[index].i2c_device_address;
}

/***********************************************************************/
/*!
    @brief    Gets the number of devices found on the bus.
    @returns  The number of devices found on the bus.
*/
/***********************************************************************/
size_t I2cController::GetScanDeviceCount() {
  return _scan_results.i2c_bus_found_devices_count;
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
    _i2c_bus_alt->ClearMuxChannel(); // sanity-check
    _i2c_bus_alt->SelectMuxChannel(mux_channel);
    return;
  }
  _i2c_bus_default->ClearMuxChannel(); // sanity-check
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
    size_t sensor_count = drv->GetNumSensorTypes();
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