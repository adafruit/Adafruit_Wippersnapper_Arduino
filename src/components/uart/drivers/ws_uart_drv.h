/*!
 * @file ws_uart_drv.h
 *
 * Base implementation for UART device drivers
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

#ifndef WS_UART_DRV_H
#define WS_UART_DRV_H
#include "Wippersnapper.h"
#include <Adafruit_Sensor.h>

// ESP8266 platform uses SoftwareSerial
// so does RP2040 (note that this has differences from the pure softwareserial
// library, see: https://arduino-pico.readthedocs.io/en/latest/piouart.html)
#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_RP2040)
#define USE_SW_UART
#include <SoftwareSerial.h>
#else
#include <HardwareSerial.h>
#endif

/**************************************************************************/
/*!
    @brief  Base class for UART Device Drivers.
*/
/**************************************************************************/
class ws_uart_drv {
public:
#ifdef USE_SW_UART
  /*******************************************************************************/
  /*!
      @brief    Initializes a UART device driver.
      @param    swSerial
                Pointer to an instance of a SoftwareSerial object.
      @param    pollingInterval
                How often the UART device will be polled, in milliseconds.
  */
  /*******************************************************************************/
  ws_uart_drv(SoftwareSerial *swSerial, int32_t pollingInterval){};
#else
  /*******************************************************************************/
  /*!
      @brief    Initializes a UART device driver.
      @param    hwSerial
                Pointer to an instance of a HardwareSerial object.
      @param    pollingInterval
                How often the UART device will be polled, in milliseconds.
  */
  /*******************************************************************************/
  ws_uart_drv(HardwareSerial *hwSerial, int32_t pollingInterval){};
#endif
  ~ws_uart_drv(void) {}

  /*******************************************************************************/
  /*!
      @brief   Checks if the UART device is ready to be polled at its time
     interval.
      @returns True if the UART device is ready to be polled, False otherwise.
  */
  /*******************************************************************************/
  bool isReady() {
    if (millis() - lastPoll >= pollingInterval) {
      return true;
    }
    return false;
  }

  /*******************************************************************************/
  /*!
      @brief   Sets the last time a UART device driver was polled
      @param   pollingInterval
               The last time a UART device was polled, in milliseconds.
  */
  /*******************************************************************************/
  void setPrvPollTime(long curTime) { lastPoll = curTime; }

  /*******************************************************************************/
  /*!
      @brief   Gets the UART device's unique identifier.
      @returns The UART device's unique identifier.
  */
  /*******************************************************************************/
  const char *getDeviceID() { return _deviceID; }

  /*******************************************************************************/
  /*!
      @brief   Sets the UART device's unique identifier.
      @param   id
               The UART device's unique identifier.
  */
  /*******************************************************************************/
  void setDeviceID(const char *id) { _deviceID = id; }

  /*******************************************************************************/
  /*!
      @brief   Sets the MQTT client used by the uart device driver for
     publishing data to Adafruit IO.
      @param   _mqtt
              Pointer to an Adafruit_MQTT object.
  */
  /*******************************************************************************/
  virtual void set_mqtt_client(Adafruit_MQTT *_mqtt) { mqttClient = _mqtt; }

  /*******************************************************************************/
  /*!
      @brief   Initializes the UART device driver.
      @returns True if UART device driver initialized successfully, False
               otherwise.
  */
  /*******************************************************************************/
  virtual bool begin() { return false; }

  /*******************************************************************************/
  /*!
      @brief   Checks if the UART device's data is ready.
      @returns True if data is available, False otherwise.
  */
  /*******************************************************************************/
  virtual bool data_available() { return false; }

  /*******************************************************************************/
  /*!
      @brief   Reads the UART device's data then packs and sends it to IO.
  */
  /*******************************************************************************/
  virtual void update(){};

  long pollingInterval; ///< UART device's polling interval, in milliseconds
  long lastPoll; ///< Last time the UART device was polled, in milliseconds
  const char *_deviceID = nullptr;     ///< UART device's ID
  Adafruit_MQTT *mqttClient = nullptr; ///< Pointer to MQTT client object
};

#endif // WS_UART_DRV_H
