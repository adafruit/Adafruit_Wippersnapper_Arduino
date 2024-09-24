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
#include "drivers/WipperSnapper_I2C_Driver_ADT7410.h"
#include "drivers/WipperSnapper_I2C_Driver_AHTX0.h"
#include "drivers/WipperSnapper_I2C_Driver_BH1750.h"
#include "drivers/WipperSnapper_I2C_Driver_BME280.h"
#include "drivers/WipperSnapper_I2C_Driver_BME680.h"
#include "drivers/WipperSnapper_I2C_Driver_BMP280.h"
#include "drivers/WipperSnapper_I2C_Driver_BMP3XX.h"
#include "drivers/WipperSnapper_I2C_Driver_DPS310.h"
#include "drivers/WipperSnapper_I2C_Driver_DS2484.h"
#include "drivers/WipperSnapper_I2C_Driver_ENS160.h"
#include "drivers/WipperSnapper_I2C_Driver_HDC302X.h"
#include "drivers/WipperSnapper_I2C_Driver_HTS221.h"
#include "drivers/WipperSnapper_I2C_Driver_HTU21D.h"
#include "drivers/WipperSnapper_I2C_Driver_HTU31D.h"
#include "drivers/WipperSnapper_I2C_Driver_INA219.h"
#include "drivers/WipperSnapper_I2C_Driver_LC709203F.h"
#include "drivers/WipperSnapper_I2C_Driver_LPS22HB.h"
#include "drivers/WipperSnapper_I2C_Driver_LPS25HB.h"
#include "drivers/WipperSnapper_I2C_Driver_LPS3XHW.h"
#include "drivers/WipperSnapper_I2C_Driver_LTR329_LTR303.h"
#include "drivers/WipperSnapper_I2C_Driver_LTR390.h"
#include "drivers/WipperSnapper_I2C_Driver_MAX17048.h"
#include "drivers/WipperSnapper_I2C_Driver_MCP3421.h"
#include "drivers/WipperSnapper_I2C_Driver_MCP9808.h"
#include "drivers/WipperSnapper_I2C_Driver_MPL115A2.h"
#include "drivers/WipperSnapper_I2C_Driver_MPRLS.h"
#include "drivers/WipperSnapper_I2C_Driver_MS8607.h"
#include "drivers/WipperSnapper_I2C_Driver_NAU7802.h"
#include "drivers/WipperSnapper_I2C_Driver_PCT2075.h"
#include "drivers/WipperSnapper_I2C_Driver_PM25.h"
#include "drivers/WipperSnapper_I2C_Driver_SCD30.h"
#include "drivers/WipperSnapper_I2C_Driver_SCD4X.h"
#include "drivers/WipperSnapper_I2C_Driver_SEN5X.h"
#include "drivers/WipperSnapper_I2C_Driver_SGP30.h"
#include "drivers/WipperSnapper_I2C_Driver_SGP40.h"
#include "drivers/WipperSnapper_I2C_Driver_SHT3X.h"
#include "drivers/WipperSnapper_I2C_Driver_SHT4X.h"
#include "drivers/WipperSnapper_I2C_Driver_SHTC3.h"
#include "drivers/WipperSnapper_I2C_Driver_SI7021.h"
#include "drivers/WipperSnapper_I2C_Driver_STEMMA_Soil_Sensor.h"
#include "drivers/WipperSnapper_I2C_Driver_TMP117.h"
#include "drivers/WipperSnapper_I2C_Driver_TSL2591.h"
#include "drivers/WipperSnapper_I2C_Driver_VCNL4020.h"
#include "drivers/WipperSnapper_I2C_Driver_VCNL4040.h"
#include "drivers/WipperSnapper_I2C_Driver_VEML7700.h"
#include "drivers/WipperSnapper_I2C_Driver_VL53L0X.h"
#include "drivers/WipperSnapper_I2C_Driver_VL53L1X.h"
#include "drivers/WipperSnapper_I2C_Driver_VL53L4CD.h"
#include "drivers/WipperSnapper_I2C_Driver_VL53L4CX.h"
#include "drivers/WipperSnapper_I2C_Driver_VL6180X.h"

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

  void sensorEventRead(
      std::vector<WipperSnapper_I2C_Driver *>::iterator &iter,
      unsigned long curTime,
      wippersnapper_signal_v1_I2CResponse *msgi2cResponse,
      bool (WipperSnapper_I2C_Driver::*getEventFunc)(sensors_event_t *),
      long (WipperSnapper_I2C_Driver::*getPeriodFunc)(),
      long (WipperSnapper_I2C_Driver::*getPeriodPrvFunc)(),
      void (WipperSnapper_I2C_Driver::*setPeriodPrvFunc)(long),
      wippersnapper_i2c_v1_SensorType sensorType, const char *sensorName,
      const char *unit, sensors_event_t event,
      float sensors_event_t::*valueMember, bool &sensorsReturningFalse,
      int &retries);

  void fillEventMessage(wippersnapper_signal_v1_I2CResponse *msgi2cResponse,
                        float value,
                        wippersnapper_i2c_v1_SensorType sensorType);

  void
  displayDeviceEventMessage(wippersnapper_signal_v1_I2CResponse *msgi2cResponse,
                            uint32_t sensorAddress);

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
  WipperSnapper_I2C_Driver_DS2484 *_ds2484 = nullptr;
  WipperSnapper_I2C_Driver_ENS160 *_ens160 = nullptr;
  WipperSnapper_I2C_Driver_SCD30 *_scd30 = nullptr;
  WipperSnapper_I2C_Driver_BH1750 *_bh1750 = nullptr;
  WipperSnapper_I2C_Driver_BME280 *_bme280 = nullptr;
  WipperSnapper_I2C_Driver_BMP280 *_bmp280 = nullptr;
  WipperSnapper_I2C_Driver_BMP3XX *_bmp3xx = nullptr;
  WipperSnapper_I2C_Driver_BME680 *_bme680 = nullptr;
  WipperSnapper_I2C_Driver_HDC302X *_hdc302x = nullptr;
  WipperSnapper_I2C_Driver_HTS221 *_hts221 = nullptr;
  WipperSnapper_I2C_Driver_HTU21D *_htu21d = nullptr;
  WipperSnapper_I2C_Driver_HTU31D *_htu31d = nullptr;
  WipperSnapper_I2C_Driver_INA219 *_ina219 = nullptr;
  WipperSnapper_I2C_Driver_LTR329_LTR303 *_ltr329 = nullptr;
  WipperSnapper_I2C_Driver_LTR390 *_ltr390 = nullptr;
  WipperSnapper_I2C_Driver_MCP3421 *_mcp3421 = nullptr;
  WipperSnapper_I2C_Driver_MCP9808 *_mcp9808 = nullptr;
  WipperSnapper_I2C_Driver_MPL115A2 *_mpl115a2 = nullptr;
  WipperSnapper_I2C_Driver_MPRLS *_mprls = nullptr;
  WipperSnapper_I2C_Driver_MS8607 *_ms8607 = nullptr;
  WipperSnapper_I2C_Driver_NAU7802 *_nau7802 = nullptr;
  WipperSnapper_I2C_Driver_TMP117 *_tmp117 = nullptr;
  WipperSnapper_I2C_Driver_TSL2591 *_tsl2591 = nullptr;
  WipperSnapper_I2C_Driver_VCNL4020 *_vcnl4020 = nullptr;
  WipperSnapper_I2C_Driver_VCNL4040 *_vcnl4040 = nullptr;
  WipperSnapper_I2C_Driver_VEML7700 *_veml7700 = nullptr;
  WipperSnapper_I2C_Driver_SCD4X *_scd40 = nullptr;
  WipperSnapper_I2C_Driver_SEN5X *_sen5x = nullptr;
  WipperSnapper_I2C_Driver_SGP30 *_sgp30 = nullptr;
  WipperSnapper_I2C_Driver_SGP40 *_sgp40 = nullptr;
  WipperSnapper_I2C_Driver_PCT2075 *_pct2075 = nullptr;
  WipperSnapper_I2C_Driver_PM25 *_pm25 = nullptr;
  WipperSnapper_I2C_Driver_SI7021 *_si7021 = nullptr;
  WipperSnapper_I2C_Driver_SHT4X *_sht4x = nullptr;
  WipperSnapper_I2C_Driver_SHT3X *_sht3x = nullptr;
  WipperSnapper_I2C_Driver_SHTC3 *_shtc3 = nullptr;
  WipperSnapper_I2C_Driver_LC709203F *_lc = nullptr;
  WipperSnapper_I2C_Driver_LPS22HB *_lps22hb = nullptr;
  WipperSnapper_I2C_Driver_LPS25HB *_lps25hb = nullptr;
  WipperSnapper_I2C_Driver_LPS3XHW *_lps3xhw = nullptr;
  WipperSnapper_I2C_Driver_STEMMA_Soil_Sensor *_ss = nullptr;
  WipperSnapper_I2C_Driver_VL53L0X *_vl53l0x = nullptr;
  WipperSnapper_I2C_Driver_VL53L1X *_vl53l1x = nullptr;
  WipperSnapper_I2C_Driver_VL53L4CD *_vl53l4cd = nullptr;
  WipperSnapper_I2C_Driver_VL53L4CX *_vl53l4cx = nullptr;
  WipperSnapper_I2C_Driver_VL6180X *_vl6180x = nullptr;
  WipperSnapper_I2C_Driver_MAX17048 *_max17048 = nullptr;
  WipperSnapper_I2C_Driver_ADT7410 *_adt7410 = nullptr;
};
extern Wippersnapper WS;

#endif // WipperSnapper_Component_I2C_H
