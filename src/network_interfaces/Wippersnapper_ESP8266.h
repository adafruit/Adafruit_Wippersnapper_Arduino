/*!
 * @file Wippersnapper_ESP8266.h
 *
 * This is a driver for using the ESP8266's network interface
 *  with Wippersnapper.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2020-2021 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef WIPPERSNAPPER_ESP8266_H
#define WIPPERSNAPPER_ESP8266_H

#ifdef ARDUINO_ARCH_ESP8266
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "ESP8266WiFi.h"
#include "Wippersnapper.h"

/* NOTE - Projects that require "Secure MQTT" (TLS/SSL) also require a new
 * SSL certificate every year. If adding Secure MQTT to your ESP8266 project is
 * important  - please switch to using the modern ESP32 (and related models)
 * instead of the ESP8266 to avoid updating the SSL fingerprint every year.
 *
 * The commented-out fingerprint below was last updated on 07/14/2025.
 *
 * If you've read through this and still want to use "Secure MQTT" with your
 * ESP8266 project, we've left the "WiFiClientSecure" lines commented out. To
 * use them, uncomment the commented out lines within this file and re-compile
 * the library.
 */
// static const char *fingerprint PROGMEM =  "47 D2 CB 14 DF 38 97 59 C6 65 1A
// 1F 3E 00 1E 53 CC A5 17 E0";

extern Wippersnapper WS;

/******************************************************************************/
/*!
    @brief  Class for interacting with the Espressif ESP8266's network
   interface.
*/
/******************************************************************************/
class Wippersnapper_ESP8266 : public Wippersnapper {

public:
  /**************************************************************************/
  /*!
  @brief  Initializes the Adafruit IO class for ESP8266 devices.
  @param  aioUsername
          Adafruit IO username
  @param  aioKey
          Adafruit IO key
  @param  netSSID
          Wireless Network SSID
  @param  netPass
          Wireless Network password
  */
  /**************************************************************************/
  Wippersnapper_ESP8266() : Wippersnapper() {
    _ssid = 0;
    _pass = 0;
    _wifi_client = new WiFiClient;
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
  }

  /**************************************************************************/
  /*!
  @brief  Destructor for the ESP8266's network iface.
  */
  /**************************************************************************/
  ~Wippersnapper_ESP8266() {
    if (_wifi_client)
      delete _wifi_client;
    if (_mqtt)
      delete _mqtt;
  }

  /**********************************************************/
  /*!
  @brief  Sets the WiFi client's ssid and password.
  @param  ssid
            Wireless network's SSID.
  @param  ssidPassword
            Wireless network's password.
  */
  /**********************************************************/
  void set_ssid_pass(const char *ssid, const char *ssidPassword) {
    _ssid = ssid;

    // set the AP password
    // check if ssidPassword was "" in secrets.json
    if ((ssidPassword != NULL) && (strlen(ssidPassword) == 0)) {
      _pass = NULL; // Set as NULL for open networks
    } else {
      _pass = ssidPassword;
    }
  }

  /**********************************************************/
  /*!
  @brief  Sets the WiFi client's ssid and password from the
            ESP8266's LittleFS.
  */
  /**********************************************************/
  void set_ssid_pass() {
    _ssid = WS._config.network.ssid;
    _pass = WS._config.network.pass;
  }

  /***********************************************************/
  /*!
  @brief   Performs a scan of local WiFi networks.
  @returns True if `_network_ssid` is found, False otherwise.
  */
  /***********************************************************/
  bool check_valid_ssid() {
    // ESP8266 WiFi scans are memory/fragmentation heavy and can crash on
    // low-RAM boards (HUZZAH) during connect. Instead of scanning, just
    // validate we have an SSID and let the connect attempt determine success.
    if (_ssid == NULL || strlen(_ssid) == 0) {
      WS_DEBUG_PRINTLN("ERROR: SSID is empty!");
      return false;
    }

    WS_DEBUG_PRINTLN("Skipping WiFi scan on ESP8266");
    return true;
  }

  /********************************************************/
  /*!
  @brief  Gets the ESP8266's unique client identifier.
  @note   For the ESP8266, the UID is the MAC address.
  */
  /********************************************************/
  void getMacAddr() {
    uint8_t mac[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    WiFi.macAddress(mac);
    memcpy(WS._macAddr, mac, sizeof(mac));
  }

  /********************************************************/
  /*!
  @brief  Gets the current network RSSI value
  @return int32_t RSSI value
  */
  /********************************************************/
  int32_t getRSSI() { return WiFi.RSSI(); }

  /*******************************************************************/
  /*!
  @brief  Sets up an Adafruit_MQTT_Client
  @param  clientID
          MQTT client identifier
  */
  /*******************************************************************/
  void setupMQTTClient(const char *clientID) {
    // Uncomment the following lines to use MQTT/SSL. You will need to
    // re-compile after. _wifi_client->setFingerprint(fingerprint); WS._mqtt =
    // new Adafruit_MQTT_Client(_wifi_client, WS._config.aio_url,
    // WS._config.io_port, clientID, WS._config.aio_user, WS._config.aio_key);
    if (WS._config.io_port == 8883)
      WS._config.io_port = 1883;
    WS._mqtt = new Adafruit_MQTT_Client(
        _wifi_client, WS._config.aio_url, WS._config.io_port, clientID,
        WS._config.aio_user, WS._config.aio_key);
  }

  /********************************************************/
  /*!
  @brief  Returns the network status of an ESP8266 module.
  @return ws_status_t
  */
  /********************************************************/
  ws_status_t networkStatus() {
    switch (WiFi.status()) {
    case WL_CONNECTED:
      return WS_NET_CONNECTED;
    case WL_CONNECT_FAILED:
      return WS_NET_CONNECT_FAILED;
    case WL_IDLE_STATUS:
      return WS_IDLE;
    default:
      return WS_NET_DISCONNECTED;
    }
  }

  /*******************************************************************/
  /*!
  @brief  Returns the type of network connection used by Wippersnapper
  @return "ESP8266"
  */
  /*******************************************************************/
  const char *connectionType() { return "ESP8266"; }

protected:
  const char *_ssid = NULL;
  const char *_pass = NULL;
  WiFiClient *_wifi_client;

  bool _tryConnectSingle(const char *ssid, const char *pass,
                         uint32_t timeout_ms) {
    if (ssid == NULL || strlen(ssid) == 0)
      return false;

    WS_DEBUG_PRINT("WiFi.begin('");
    WS_DEBUG_PRINT(ssid);
    WS_DEBUG_PRINTLN("')");

    WiFi.begin(ssid, pass);

    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeout_ms) {
      delay(50);
      yield();
      WS.feedWDT();
    }
    WS_DEBUG_PRINT("WiFi.status(): ");
    WS_DEBUG_PRINTLN(WiFi.status());
    return (WiFi.status() == WL_CONNECTED);
  }

  /**************************************************************************/
  /*!
  @brief  Establishes a connection with the wireless network.
  */
  /**************************************************************************/
  void _connect() {

    if (WiFi.status() == WL_CONNECTED)
      return;

    if (strlen(_ssid) == 0) {
      _status = WS_SSID_INVALID;
    } else {
      WiFi.setAutoReconnect(false);
      // Attempt connection
      _disconnect();
      delay(100);
      // ESP8266 MUST be in STA mode to avoid device acting as client/server
      WiFi.mode(WIFI_STA);
      _status = WS_NET_DISCONNECTED;
      delay(100);

      WS_DEBUG_PRINT("Free heap before connect: ");
      WS_DEBUG_PRINTLN(ESP.getFreeHeap());

      bool connected = false;

      // Try primary network first.
      connected = _tryConnectSingle(_ssid, _pass, 25000);

      // If multi-network mode is enabled, try alternates sequentially.
      if (!connected && WS._isWiFiMulti) {
        for (int i = 0; i < WS_MAX_ALT_WIFI_NETWORKS && !connected; i++) {
          if (strlen(WS._multiNetworks[i].ssid) == 0)
            continue;

          _disconnect();
          delay(100);
          connected = _tryConnectSingle(WS._multiNetworks[i].ssid,
                                        WS._multiNetworks[i].pass, 25000);
        }
      }

      WS_DEBUG_PRINT("Free heap after connect: ");
      WS_DEBUG_PRINTLN(ESP.getFreeHeap());

      if (connected) {
        _status = WS_NET_CONNECTED;
      } else {
        _status = WS_NET_DISCONNECTED;
      }

      WS.feedWDT();
    }
  }

  /**************************************************************************/
  /*!
      @brief  Disconnects from the wireless network.
  */
  /**************************************************************************/
  void _disconnect() {
    WiFi.disconnect();
    delay(500);
  }
};

#endif // ARDUINO_ARCH_ESP8266
#endif // WIPPERSNAPPER_ESP8266_H