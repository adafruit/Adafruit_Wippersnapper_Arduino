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
  /*******************************************************************************/
  /*!
      @brief    Initializes a PM25AQI UART device driver.
      @param    swSerial
                Pointer to an instance of a SoftwareSerial object.
      @param    pollingInterval
                How often the PM25AQI device will be polled, in milliseconds.
  */
  /*******************************************************************************/
  ws_uart_drv_pm25aqi() : ws_uart_drv(SoftwareSerial * swSerial) {}
#else
  /*******************************************************************************/
  /*!
      @brief    Initializes a PM25AQI UART device driver.
      @param    hwSerial
                Pointer to an instance of a HardwareSerial object.
      @param    interval
                How often the PM25AQI device will be polled, in milliseconds.
  */
  /*******************************************************************************/
  ws_uart_drv_pm25aqi(HardwareSerial *hwSerial, int32_t interval)
      : ws_uart_drv(hwSerial, pollingInterval) {
    _hwSerial = hwSerial;
    pollingInterval = (long) interval;
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

  bool begin() override {
    _aqi = new Adafruit_PM25AQI();

#ifdef USE_SW_UART
// TODO: Add SW uart path
#else
    if (!_aqi->begin_UART(
            _hwSerial)) { // connect to the sensor over hardware serial
      return false;
    }
#endif
    // Set device's ID
    setDeviceID("pms5003");
    return true;
  }

  bool data_available() override {
    if (!_aqi->read(&_data)) {
      //Serial.println("[UART, PM25] Data not available.");
      return false;
    }
    Serial.println("[UART, PM25] Read data OK");
    return true;
  }

  void update() override {
    

    Serial.println("AQI reading success");
    // TODO: Optionally print out all the results from last read
    Serial.println();
    Serial.println(F("---------------------------------------"));
    Serial.println(F("Concentration Units (standard)"));
    Serial.println(F("---------------------------------------"));
    Serial.print(F("PM 1.0: ")); Serial.print(_data.pm10_standard);
    Serial.print(F("\t\tPM 2.5: ")); Serial.print(_data.pm25_standard);
    Serial.print(F("\t\tPM 10: ")); Serial.println(_data.pm100_standard);

    // Create a new UART response message
    wippersnapper_signal_v1_UARTResponse msgUARTResponse =
        wippersnapper_signal_v1_UARTResponse_init_zero;
    msgUARTResponse.which_payload =
        wippersnapper_signal_v1_UARTResponse_resp_uart_device_event_tag;
    // We'll be sending back six sensor_events: pm10_standard, pm25_standard,
    // pm100_standard, pm10_env, pm25_env, and pm100_env
    msgUARTResponse.payload.resp_uart_device_event.sensor_event_count = 6;
    // getDeviceID();
    strcpy(msgUARTResponse.payload.resp_uart_device_event.device_id,
           getDeviceID());

    // Pack all _data into `device_event` fields
    // pm10_std
    msgUARTResponse.payload.resp_uart_device_event.sensor_event[0].type =
        wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM10_STD;
    msgUARTResponse.payload.resp_uart_device_event.sensor_event[0].value =
        (float)_data.pm10_standard;
    // pm25_std
    msgUARTResponse.payload.resp_uart_device_event.sensor_event[1].type =
        wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM25_STD;
    msgUARTResponse.payload.resp_uart_device_event.sensor_event[1].value =
        (float)_data.pm25_standard;
    // pm100_std
    msgUARTResponse.payload.resp_uart_device_event.sensor_event[2].type =
        wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM100_STD;
    msgUARTResponse.payload.resp_uart_device_event.sensor_event[2].value =
        (float)_data.pm100_standard;
    // pm10_env
    msgUARTResponse.payload.resp_uart_device_event.sensor_event[3].type =
        wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM10_ENV;
    msgUARTResponse.payload.resp_uart_device_event.sensor_event[3].value =
        (float)_data.pm10_env;
    // pm25_env
    msgUARTResponse.payload.resp_uart_device_event.sensor_event[4].type =
        wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM25_ENV;
    msgUARTResponse.payload.resp_uart_device_event.sensor_event[4].value =
        (float)_data.pm25_env;
    // pm100_env
    msgUARTResponse.payload.resp_uart_device_event.sensor_event[5].type =
        wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM100_ENV;
    msgUARTResponse.payload.resp_uart_device_event.sensor_event[5].value =
        (float)_data.pm100_env;

    // Encode message data
    uint8_t mqttBuffer[512] = {0};
    pb_ostream_t ostream =
        pb_ostream_from_buffer(mqttBuffer, sizeof(mqttBuffer));
    if (!pb_encode(&ostream, wippersnapper_signal_v1_UARTResponse_fields,
                   &msgUARTResponse)) {
      Serial.println("[ERROR, UART]: Unable to encode device response!");
      return;
    }

    // Publish message to IO
    size_t msgSz;
    pb_get_encoded_size(&msgSz, wippersnapper_signal_v1_UARTResponse_fields,
                        &msgUARTResponse);
    Serial.print("[UART] Publishing event to IO..");
    // TODO: Re-enable
    mqttClient->publish("brentrubell/wprsnpr/io-wipper-feather-esp32s3220185245/signals/device/uart", mqttBuffer, msgSz, 1);
    Serial.println("Published!");
  }

protected:
  Adafruit_PM25AQI *_aqi = nullptr;    ///< Pointer to PM25AQI sensor object
  PM25_AQI_Data _data;                 ///< PM25AQI sensor data struct.
  HardwareSerial *_hwSerial = nullptr; ///< Pointer to UART interface
};

#endif // WS_UART_DRV_PM25AQI_H