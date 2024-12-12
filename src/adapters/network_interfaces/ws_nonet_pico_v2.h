/*!
 * @file ws_nonet_pico_v2.h
 *
 * This is a driver for using the Raspberry Pi Pi Pico/Pico2
 * without a network interface with Adafruit IO Wippersnapper.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2024-2025 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef WS_NONET_PICO_V2_H
#define WS_NONET_PICO_V2_H

#if defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_RASPBERRY_PI_PICO_2)

#define PICO_CONNECT_TIMEOUT_MS 20000   /*!< Connection timeout (in ms) */
#define PICO_CONNECT_RETRY_DELAY_MS 200 /*!< delay time between retries. */

#include "Wippersnapper_V2.h"
#include "Arduino.h"

extern Wippersnapper_V2 WsV2;

/****************************************************************************/
/*!
    @brief  Class for using the Raspberry Pi Pico network interface.
*/
/****************************************************************************/
class ws_nonet_pico_v2 : public Wippersnapper_V2 {

public:
  /**************************************************************************/
  /*!
  @brief  Initializes the WipperSnapper class for RPi Pico.
  */
  /**************************************************************************/
  ws_nonet_pico_v2() : Wippersnapper_V2() {
    // No-op
  }

  /**************************************************************************/
  /*!
  @brief  Destructor
  */
  /**************************************************************************/
  ~ws_nonet_pico_v2() {
    // No-op
  }

  /********************************************************/
  /*!
  @brief  Sets the WiFi client's ssid and password.
  @param  ssid
            WiFi network's SSID.
  @param  ssidPassword
            WiFi network's password.
  */
  /********************************************************/
  void set_ssid_pass(const char *ssid, const char *ssidPassword) {
    // No-op
    WS_DEBUG_PRINTLN("Code should not get here!");
  }

  /**********************************************************/
  /*!
  @brief  Sets the WiFi client's ssid and password.
  */
  /**********************************************************/
  void set_ssid_pass() {
    WS_DEBUG_PRINTLN("Code should not get here!");
  }

  /***********************************************************/
  /*!
  @brief   Performs a scan of local WiFi networks.
  @returns True if `_network_ssid` is found, False otherwise.
  */
  /***********************************************************/
  bool check_valid_ssid() {
    WS_DEBUG_PRINTLN("Code should not get here!");
    return false;
  }

  /********************************************************/
  /*!
  @brief  Sets the RPi Pico's unique client identifier
  @note   On RPi Pico, the UID is the MAC address.
  */
  /********************************************************/
  void getMacAddr() {
    WS_DEBUG_PRINTLN("Code should not get here!");
  }

  /********************************************************/
  /*!
  @brief  Gets the current network RSSI value
  @return int32_t RSSI value
  */
  /********************************************************/
  int32_t getRSSI() { 
    WS_DEBUG_PRINTLN("Code should not get here!");
    return 0; }

  /********************************************************/
  /*!
  @brief  Initializes the MQTT client
  @param  clientID
          MQTT client identifier
  */
  /********************************************************/
  void setupMQTTClient(const char *clientID) {
    // No-op
    WS_DEBUG_PRINTLN("Code should not get here!");
  }

  /********************************************************/
  /*!
  @brief  Returns the network status of an RPi Pico.
  @return ws_status_t
  */
  /********************************************************/
  ws_status_t networkStatus() {
    WS_DEBUG_PRINTLN("Code should not get here!");
    return WS_NET_DISCONNECTED;
  }

  /*******************************************************************/
  /*!
  @brief  Returns the type of network connection used by Wippersnapper
  @return Pico
  */
  /*******************************************************************/
  const char *connectionType() { 
    WS_DEBUG_PRINTLN("Code should not get here!");
    return "Pico-Nonet-v2"; }

protected:
  /**************************************************************************/
  /*!
  @brief  Establishes a connection with the wireless network.
  */
  /**************************************************************************/
  void _connect() {
    // No-op
    WS_DEBUG_PRINTLN("Code should not get here!");
  }

  /**************************************************************************/
  /*!
      @brief  Disconnects from the wireless network.
  */
  /**************************************************************************/
  void _disconnect() {
    // No-op
    WS_DEBUG_PRINTLN("Code should not get here!");
  }
};

#endif // RASPBERRY_PI_PICO_W
#endif // WS_NONET_PICO_H