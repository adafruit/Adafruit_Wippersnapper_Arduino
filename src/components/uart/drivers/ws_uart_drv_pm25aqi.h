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
    // TODO: Make this printout more verbose, showing the units
    Serial.println("[UART, PM25] Got Data");

    // Create a new UART response message
    wippersnapper_signal_v1_UARTResponse msgUARTResponse = wippersnapper_signal_v1_UARTResponse_init_zero;
    msgUARTResponse.which_payload = wippersnapper_signal_v1_UARTResponse_resp_uart_device_event_tag;
    // We'll be sending back six sensor_events: pm10_standard, pm25_standard, pm100_standard, pm10_env, pm25_env, and pm100_env
    msgUARTResponse.payload.resp_uart_device_event.sensor_event_count = 6;


    // Pack all _data into `device_event` fields
    // pm10_std
    msgUARTResponse.payload.resp_uart_device_event.sensor_event[0].type = wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM10_STD;
    msgUARTResponse.payload.resp_uart_device_event.sensor_event[0].value = (float) _data.pm10_standard;
    // pm25_std
    msgUARTResponse.payload.resp_uart_device_event.sensor_event[1].type = wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM25_STD;
    msgUARTResponse.payload.resp_uart_device_event.sensor_event[1].value = (float) _data.pm25_standard;
    // pm100_std
    msgUARTResponse.payload.resp_uart_device_event.sensor_event[2].type = wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM100_STD;
    msgUARTResponse.payload.resp_uart_device_event.sensor_event[2].value = (float) _data.pm100_standard;
    // pm10_env
    msgUARTResponse.payload.resp_uart_device_event.sensor_event[3].type = wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM10_ENV;
    msgUARTResponse.payload.resp_uart_device_event.sensor_event[3].value = (float) _data.pm10_env;
    // pm25_env
    msgUARTResponse.payload.resp_uart_device_event.sensor_event[4].type = wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM25_ENV;
    msgUARTResponse.payload.resp_uart_device_event.sensor_event[4].value = (float) _data.pm25_env;
    // pm100_env
    msgUARTResponse.payload.resp_uart_device_event.sensor_event[5].type = wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM100_ENV;
    msgUARTResponse.payload.resp_uart_device_event.sensor_event[5].value = (float) _data.pm100_env;

    // Encode data

  }

protected:
  Adafruit_PM25AQI *_aqi = nullptr;    ///< Pointer to PM25AQI sensor object
  PM25_AQI_Data _data;                 ///< PM25AQI sensor data struct.
  HardwareSerial *_hwSerial = nullptr; ///< Pointer to UART interface
};

#endif // WipperSnapper_I2C_Driver_VL53L0X