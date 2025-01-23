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
    // Attempt to read the PM2.5 Sensor, can be flaky see Adafruit_PM25AQI#14
    bool result = false;
    RETRY_FUNCTION_UNTIL_TIMEOUT(_aqi->read, bool, result, 
      [](bool res) -> bool { return res==true; },
      500, 100, &_data);

    if (!result) {
      WS_DEBUG_PRINTLN("[UART, PM25] Data not available.");
      return result;
    }
    WS_DEBUG_PRINTLN("[UART, PM25] Read data OK");
    WS_DEBUG_PRINTLN();
    WS_DEBUG_PRINTLN(F("---------------------------------------"));
    WS_DEBUG_PRINTLN(F("Concentration Units (standard)"));
    WS_DEBUG_PRINTLN(F("---------------------------------------"));
    WS_DEBUG_PRINT(F("PM 1.0: "));
    WS_DEBUG_PRINT(_data.pm10_standard);
    WS_DEBUG_PRINT(F("\t\tPM 2.5: "));
    WS_DEBUG_PRINT(_data.pm25_standard);
    WS_DEBUG_PRINT(F("\t\tPM 10: "));
    WS_DEBUG_PRINTLN(_data.pm100_standard);
    WS_DEBUG_PRINTLN(F("Concentration Units (environmental)"));
    WS_DEBUG_PRINTLN(F("---------------------------------------"));
    WS_DEBUG_PRINT(F("PM 1.0: "));
    WS_DEBUG_PRINT(_data.pm10_env);
    WS_DEBUG_PRINT(F("\t\tPM 2.5: "));
    WS_DEBUG_PRINT(_data.pm25_env);
    WS_DEBUG_PRINT(F("\t\tPM 10: "));
    WS_DEBUG_PRINTLN(_data.pm100_env);
    WS_DEBUG_PRINTLN(F("---------------------------------------"));
    return result;
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
    strcpy(msgUARTResponse.payload.resp_uart_device_event.device_id,
           getDriverID());

    // check if driverID is pm1006
    if (strcmp(getDriverID(), "pm1006") == 0) {
      // PM1006 returns only PM2.5_ENV readings
      msgUARTResponse.payload.resp_uart_device_event.sensor_event_count = 1;
      packUARTResponse(&msgUARTResponse, 0,
                       wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM25_ENV,
                       (float)_data.pm25_env);
    } else {
      msgUARTResponse.payload.resp_uart_device_event.sensor_event_count = 6;
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
    }

    // Encode message data
    uint8_t mqttBuffer[512] = {0};
    pb_ostream_t ostream =
        pb_ostream_from_buffer(mqttBuffer, sizeof(mqttBuffer));
    if (!ws_pb_encode(&ostream, wippersnapper_signal_v1_UARTResponse_fields,
                      &msgUARTResponse)) {
      WS_DEBUG_PRINTLN("[ERROR, UART]: Unable to encode device response!");
      return;
    }

    // Publish message to IO
    size_t msgSz;
    pb_get_encoded_size(&msgSz, wippersnapper_signal_v1_UARTResponse_fields,
                        &msgUARTResponse);
    WS_DEBUG_PRINT("[UART] Publishing event to IO..");
    mqttClient->publish(uartTopic, mqttBuffer, msgSz, 1);
    WS_DEBUG_PRINTLN("Published!");

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