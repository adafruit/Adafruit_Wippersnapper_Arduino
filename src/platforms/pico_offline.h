/*!
 * @file pico_offline.h
 *
 * This is a stub class for using the RP2040/RP2350 without a network interface
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef PICO_OFFLINE_H
#define PICO_OFFLINE_H

#if defined(ARDUINO_RASPBERRY_PI_PICO) ||                                      \
    defined(ARDUINO_RASPBERRY_PI_PICO_2) ||                                    \
    defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_ADALOGGER) ||                      \
    defined(ARDUINO_ADAFRUIT_METRO_RP2350)

#define PICO_CONNECT_TIMEOUT_MS 20000   /*!< Connection timeout (in ms) */
#define PICO_CONNECT_RETRY_DELAY_MS 200 /*!< delay time between retries. */

#include "Arduino.h"
#include "wippersnapper.h"

extern wippersnapper WsV2;

/*!
    @brief  Class for using the Raspberry Pi Pico network interface.
*/
class pico_offline : public wippersnapper {

public:
  /*!
  @brief  Initializes the WipperSnapper class for RPi Pico.
  */
  pico_offline() : wippersnapper() {
    // Do-nothing
  }

  /*!
  @brief  Destructor
  */
  ~pico_offline() {
    // Do-nothing - this class has no resources to release
  }

  /*!
  @brief  Sets the WiFi client's ssid and password.
  @param  ssid
            WiFi network's SSID.
  @param  ssidPassword
            WiFi network's password.
  */
  void set_ssid_pass(const char *ssid, const char *ssidPassword) {
    WS_DEBUG_PRINTLN("[pico_offline] Error: set_ssid_pass() is not "
                     "supported in this implementation!");
  }

  /*!
  @brief  Sets the WiFi client's ssid and password.
  */
  void set_ssid_pass() {
    WS_DEBUG_PRINTLN("[pico_offline] Error: set_ssid_pass() is not "
                     "supported in this implementation!");
  }

  /*!
  @brief   Performs a scan of local WiFi networks.
  @returns True if `_network_ssid` is found, False otherwise.
  */
  bool check_valid_ssid() {
    WS_DEBUG_PRINTLN("[pico_offline] Error: check_valid_ssid() is not "
                     "supported in this implementation!");
    return false; // return an invalid value
  }

  /*!
  @brief  Sets the RPi Pico's unique client identifier
  @note   On RPi Pico, the UID is the MAC address.
  */
  void getMacAddr() {
    // Do Nothing
  }

  /*!
  @brief  Gets the current network RSSI value
  @return int32_t RSSI value
  */
  int32_t getRSSI() {
    WS_DEBUG_PRINTLN("[pico_offline] Error: getRSSI() is not supported in "
                     "this implementation!");
    return -9999; // return an invalid value
  }

  /*!
  @brief  Initializes the MQTT client
  @param  clientID
          MQTT client identifier
  */
  void setupMQTTClient(const char *clientID) {
    WS_DEBUG_PRINTLN("[pico_offline] Error: setupMQTTClient() is not "
                     "supported in this implementation!");
  }

  /*!
  @brief  Returns the network status of an RPi Pico.
  @return ws_status_t
  */
  ws_status_t networkStatus() {
    WS_DEBUG_PRINTLN("[pico_offline] Error: networkStatus() is not "
                     "supported in this implementation!");
    return WS_NET_DISCONNECTED; // this value is valid, we are not connected to
                                // a network
  }

  /*!
  @brief  Returns the type of network connection used by Wippersnapper
  @return Pico
  */
  const char *connectionType() {
    WS_DEBUG_PRINTLN("[pico_offline] Error: connectionType() is not "
                     "supported in this implementation!");
    return "ws-offline-pico";
  }

protected:
  /*!
  @brief  Establishes a connection with the wireless network.
  */
  void _connect() {
    WS_DEBUG_PRINTLN("[pico_offline] Error: _connect() is not supported in "
                     "this implementation!");
  }

  /*!
      @brief  Disconnects from the wireless network.
  */
  void _disconnect() {
    WS_DEBUG_PRINTLN("[pico_offline] Error: _disconnect() is not supported "
                     "in this implementation!");
  }
};

#endif // RASPBERRY_PI_PICO
#endif // PICO_OFFLINE_H