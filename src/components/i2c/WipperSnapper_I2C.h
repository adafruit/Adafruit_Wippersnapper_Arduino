/*!
 * @file WipperSnapper_I2C.h
 *
 * This component initiates I2C operations
 * using the Arduino generic TwoWire driver.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2021 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WipperSnapper_Component_I2C_H
#define WipperSnapper_Component_I2C_H

#include "Wippersnapper.h"
#include <Wire.h>

#include "drivers/WipperSnapper_I2C_Driver.h"
#include "drivers/WipperSnapper_I2C_Driver_AHTX0.h"
#include "drivers/WipperSnapper_I2C_Driver_BME280.h"
#include "drivers/WipperSnapper_I2C_Driver_BME680.h"
#include "drivers/WipperSnapper_I2C_Driver_DPS310.h"
#include "drivers/WipperSnapper_I2C_Driver_LC709203F.h"
#include "drivers/WipperSnapper_I2C_Driver_MCP9601.h"
#include "drivers/WipperSnapper_I2C_Driver_MCP9808.h"
#include "drivers/WipperSnapper_I2C_Driver_PM25.h"
#include "drivers/WipperSnapper_I2C_Driver_SCD30.h"
#include "drivers/WipperSnapper_I2C_Driver_SCD40.h"
#include "drivers/WipperSnapper_I2C_Driver_SHT3X.h"
#include "drivers/WipperSnapper_I2C_Driver_SHT4X.h"
#include "drivers/WipperSnapper_I2C_Driver_SHTC3.h"
#include "drivers/WipperSnapper_I2C_Driver_SI7021.h"
#include "drivers/WipperSnapper_I2C_Driver_STEMMA_Soil_Sensor.h"
#include "drivers/WipperSnapper_I2C_Driver_TSL2591.h"
#include "drivers/WipperSnapper_I2C_Driver_VEML7700.h"

#define I2C_TIMEOUT_MS 50 ///< Default I2C timeout, in milliseconds.

// forward decl.
class Wippersnapper;

/**************************************************************************/
/*!
    @brief  Class that provides an interface with the I2C bus.
*/
/**************************************************************************/
class WipperSnapper_Component_I2C {
public:
  WipperSnapper_Component_I2C(
      wippersnapper_i2c_v1_I2CBusInitRequest *msgInitRequest);
  ~WipperSnapper_Component_I2C();
  bool isInitialized();
  wippersnapper_i2c_v1_BusResponse getBusStatus();

  wippersnapper_i2c_v1_I2CBusScanResponse scanAddresses();
  bool
  initI2CDevice(wippersnapper_i2c_v1_I2CDeviceInitRequest *msgDeviceInitReq);

  void updateI2CDeviceProperties(
      wippersnapper_i2c_v1_I2CDeviceUpdateRequest *msgDeviceUpdateReq);
  void deinitI2CDevice(
      wippersnapper_i2c_v1_I2CDeviceDeinitRequest *msgDeviceDeinitReq);

  void update();
  void fillEventMessage(wippersnapper_signal_v1_I2CResponse *msgi2cResponse,
                        float value,
                        wippersnapper_i2c_v1_SensorType sensorType);

  bool encodePublishI2CDeviceEventMsg(
      wippersnapper_signal_v1_I2CResponse *msgi2cResponse,
      uint32_t sensorAddress);

private:
  bool _isInit = false;
  int32_t _portNum;
  TwoWire *_i2c = nullptr;
  wippersnapper_i2c_v1_BusResponse _busStatusResponse;
  std::vector<WipperSnapper_I2C_Driver *> drivers; ///< List of sensor drivers
  // Sensor driver objects
  WipperSnapper_I2C_Driver_AHTX0 *_ahtx0 = nullptr;
  WipperSnapper_I2C_Driver_DPS310 *_dps310 = nullptr;
  WipperSnapper_I2C_Driver_SCD30 *_scd30 = nullptr;
  WipperSnapper_I2C_Driver_BME280 *_bme280 = nullptr;
  WipperSnapper_I2C_Driver_BME680 *_bme680 = nullptr;
  WipperSnapper_I2C_Driver_MCP9808 *_mcp9808 = nullptr;
  WipperSnapper_I2C_Driver_MCP9601 *_mcp9601 = nullptr;
  WipperSnapper_I2C_Driver_TSL2591 *_tsl2591 = nullptr;
  WipperSnapper_I2C_Driver_VEML7700 *_veml7700 = nullptr;
  WipperSnapper_I2C_Driver_SCD40 *_scd40 = nullptr;
  WipperSnapper_I2C_Driver_PM25 *_pm25 = nullptr;
  WipperSnapper_I2C_Driver_SI7021 *_si7021 = nullptr;
  WipperSnapper_I2C_Driver_SHT4X *_sht4x = nullptr;
  WipperSnapper_I2C_Driver_SHT3X *_sht3x = nullptr;
  WipperSnapper_I2C_Driver_SHTC3 *_shtc3 = nullptr;
  WipperSnapper_I2C_Driver_LC709203F *_lc = nullptr;
  WipperSnapper_I2C_Driver_STEMMA_Soil_Sensor *_ss = nullptr;
};
extern Wippersnapper WS;

#endif // WipperSnapper_Component_I2C_H
