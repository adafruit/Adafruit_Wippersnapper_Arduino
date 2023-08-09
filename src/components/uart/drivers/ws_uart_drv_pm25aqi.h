/*!
 * @file ws_uart_drv_pm25aqi.h
 *
 * Device driver for the Adafruit PM25AQI Arduino Library
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2023 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_UART_DRV_PM25AQI_H
#define WS_UART_DRV_PM25AQI_H

#include "Wippersnapper.h"
#include "ws_uart_drv.h"
#include <Adafruit_PM25AQI.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a
            UART PM25 AQI sensor.
*/
/**************************************************************************/
class ws_uart_drv_pm25aqi : public ws_uart_drv {
public:
#ifdef USE_SW_UART
  ws_uart_drv_pm25aqi() : ws_uart_drv(SoftwareSerial * swSerial) {}
#else
  ws_uart_drv_pm25aqi(HardwareSerial *hwSerial, int32_t pollingInterval)
      : ws_uart_drv(hwSerial, pollingInterval) {
    _hwSerial = hwSerial;
    pollingInterval = pollingInterval;
  };
#endif // USE_SW_UART

  /*******************************************************************************/
  /*!
      @brief    Destructor for PM25AQI sensor.
  */
  /*******************************************************************************/
  ~ws_uart_drv_pm25aqi() {
    _aqi = nullptr;
    _hwSerial = nullptr;
  }

  bool begin() {
    _aqi = new Adafruit_PM25AQI();

#ifdef USE_SW_UART
// TODO: Add SW uart path
#else
    if (!_aqi->begin_UART(
            _hwSerial)) { // connect to the sensor over hardware serial
      return false;
    }
#endif
    // Serial.println(WS.bufSize);
    return true;
  }

  void update() {
    // Attempt to read and pack PM25AQI data
    if (!_aqi->read(&_data)) {
      Serial.println("Could not read AQI...");
      return;
    }
    // We have data packed into PM25_AQI_Data struct.
    // but, let's re-package this data into a sensor_event message for IO
    
    // Create a new response message?
    // TODO: Maybe we import the i2c response message here within the proto wrapper?
    wippersnapper_signal_v1_I2CResponse msgi2cResponse = wippersnapper_signal_v1_I2CResponse_init_zero;
    msgi2cResponse.which_payload = wippersnapper_signal_v1_I2CResponse_resp_i2c_device_event_tag;

  }

protected:
  Adafruit_PM25AQI *_aqi = nullptr;    ///< Pointer to PM25AQI sensor object
  PM25_AQI_Data _data;                 ///< PM25AQI sensor data struct.
  HardwareSerial *_hwSerial = nullptr; ///< Pointer to UART interface
};

#endif // WipperSnapper_I2C_Driver_VL53L0X