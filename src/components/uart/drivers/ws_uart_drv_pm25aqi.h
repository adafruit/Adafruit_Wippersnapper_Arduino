/*!
 * @file ws_uart_drv_pm25aqi.h
 *
 * WipperSnapper device driver for the Adafruit_PM25AQI Arduino Library
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
    @brief  Class that provides an interface for a PM25 AQI UART sensor.
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
  ws_uart_drv_pm25aqi(SoftwareSerial *swSerial, int32_t interval)
      : ws_uart_drv(swSerial, interval) {
    _swSerial = swSerial;
    pollingInterval = (unsigned long)interval;
    // Set driver ID
    setDriverID("pms5003");
  };
#else
  /*******************************************************************************/
  /*!
      @brief    Initializes the PM25AQI UART device driver.
      @param    hwSerial
                Pointer to an instance of a HardwareSerial object.
      @param    interval
                How often the PM25AQI device will be polled, in milliseconds.
  */
  /*******************************************************************************/
  ws_uart_drv_pm25aqi(HardwareSerial *hwSerial, int32_t interval)
      : ws_uart_drv(hwSerial, interval) {
    _hwSerial = hwSerial;
    pollingInterval = (unsigned long)interval;
    // Set driver ID
    setDriverID("pms5003");
  };
#endif // USE_SW_UART

  /*******************************************************************************/
  /*!
      @brief    Destructor for a PM25AQI sensor.
  */
  /*******************************************************************************/
  ~ws_uart_drv_pm25aqi() {
    delete _aqi;
#ifdef USE_SW_UART
    _swSerial = nullptr;
#else
    _hwSerial = nullptr;
#endif
  }

  /*******************************************************************************/
  /*!
      @brief   Initializes a PM25AQI sensor.
      @returns True if the PM25AQI sensor was successfully initialized,
                False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    _aqi = new Adafruit_PM25AQI();
#ifdef USE_SW_UART
    Serial.println("use sw uart init");
    if (!_aqi->begin_UART(
            _swSerial)) { // connect to the sensor over software serial
      return false;
    }
#else
    if (!_aqi->begin_UART(
            _hwSerial)) { // connect to the sensor over hardware serial
      return false;
    }
#endif
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief   Attempts to read data from the PM25AQI sensor.
      @returns True if data was successfully read, False otherwise.
  */
  /*******************************************************************************/
  bool read_data() override {
    Serial.println("[UART, PM25] Reading data...");
    // Attempt to read the PM2.5 Sensor
    if (!_aqi->read(&_data)) {
      Serial.println("[UART, PM25] Data not available.");
      delay(500);
      return false;
    }
    Serial.println("[UART, PM25] Read data OK");
    Serial.println();
    Serial.println(F("---------------------------------------"));
    Serial.println(F("Concentration Units (standard)"));
    Serial.println(F("---------------------------------------"));
    Serial.print(F("PM 1.0: "));
    Serial.print(_data.pm10_standard);
    Serial.print(F("\t\tPM 2.5: "));
    Serial.print(_data.pm25_standard);
    Serial.print(F("\t\tPM 10: "));
    Serial.println(_data.pm100_standard);
    Serial.println(F("Concentration Units (environmental)"));
    Serial.println(F("---------------------------------------"));
    Serial.print(F("PM 1.0: "));
    Serial.print(_data.pm10_env);
    Serial.print(F("\t\tPM 2.5: "));
    Serial.print(_data.pm25_env);
    Serial.print(F("\t\tPM 10: "));
    Serial.println(_data.pm100_env);
    Serial.println(F("---------------------------------------"));

    return true;
  }

  /*******************************************************************************/
  /*!
      @brief   Packs and sends the device's event data to Adafruit IO.
  */
  /*******************************************************************************/
  void send_data() override {
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

    // Pack sensor data into UART response message
    packUARTResponse(&msgUARTResponse, 0,
                     wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM10_STD,
                     (float)_data.pm10_standard);

    packUARTResponse(&msgUARTResponse, 1,
                     wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM25_STD,
                     (float)_data.pm25_standard);

    packUARTResponse(&msgUARTResponse, 2,
                     wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM100_STD,
                     (float)_data.pm100_standard);

    packUARTResponse(&msgUARTResponse, 3,
                     wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM10_ENV,
                     (float)_data.pm10_env);

    packUARTResponse(&msgUARTResponse, 4,
                     wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM25_ENV,
                     (float)_data.pm25_env);

    packUARTResponse(&msgUARTResponse, 5,
                     wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM100_ENV,
                     (float)_data.pm100_env);

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
    mqttClient->publish(uartTopic, mqttBuffer, msgSz, 1);
    Serial.println("Published!");

    setPrvPollTime(millis());
  }

protected:
  Adafruit_PM25AQI *_aqi = nullptr; ///< Pointer to PM25AQI sensor object
  PM25_AQI_Data _data;              ///< PM25AQI sensor data struct.
#ifdef USE_SW_UART
  SoftwareSerial *_swSerial = nullptr; ///< Pointer to Software UART interface
#else
  HardwareSerial *_hwSerial = nullptr; ///< Pointer to Hardware UART interface
#endif
};

#endif // WS_UART_DRV_PM25AQI_H