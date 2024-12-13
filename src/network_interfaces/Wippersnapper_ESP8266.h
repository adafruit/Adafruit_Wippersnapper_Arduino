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
#include "ESP8266WiFiMulti.h"
#include "Wippersnapper.h"
#include "Wippersnapper_Networking.h"

/* NOTE - Projects that require "Secure MQTT" (TLS/SSL) also require a new
 * SSL certificate every year. If adding Secure MQTT to your ESP8266 project is
 * important  - please switch to using the modern ESP32 (and related models)
 * instead of the ESP8266 to avoid updating the SSL fingerprint every year.
 *
 * If you've read through this and still want to use "Secure MQTT" with your
 * ESP8266 project, we've left the "WiFiClientSecure" lines commented out. To
 * use them, uncomment the commented out lines within this file and re-compile
 * the library.
 */
// static const char *fingerprint PROGMEM =  "4E C1 52 73 24 A8 36 D6 7A 4C 67
// C7 91 0C 0A 22 B9 2D 5B CA";

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

  /****************************************************************/
  /*!
  @brief  a structure to hold network information for sorting
  */
  /****************************************************************/
  struct WiFiNetwork {
    char ssid[33]; /*!< SSID (Max 32 characters + null terminator */
    int rssi;      /*!< Received Signal Strength Indicator */
  };

  /*******************************************************************/
  /*!
  @brief  Comparison function to sort by RSSI in descending order
  @param  a
          WiFiNetwork object
  @param  b
          WiFiNetwork object
  @returns True if a.rssi > b.rssi
  */
  /*******************************************************************/
  bool static compareByRSSI(const WiFiNetwork &a, const WiFiNetwork &b) {
    return a.rssi > b.rssi;
  }

  /***********************************************************/
  /*!
  @brief   Performs a scan of local WiFi networks.
  @returns True if `_network_ssid` is found, False otherwise.
  */
  /***********************************************************/
  bool check_valid_ssid() {
    // Set WiFi to station mode and disconnect from an AP if it was previously
    // connected
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    // Perform a network scan
    int n = WiFi.scanNetworks();
    if (n == 0) {
      WS_DEBUG_PRINTLN("ERROR: No WiFi networks found!");
      return false;
    }

    WiFiNetwork networks[WS_MAX_SORTED_NETWORKS];
    uint8_t numSavedNetworks = 0;
    // Store the scanned networks in the vector
    for (int i = 0; i < n; i++) {
      if (i < WS_MAX_SORTED_NETWORKS) {
        strncpy(networks[i].ssid, WiFi.SSID(i).c_str(),
                sizeof(networks[i].ssid));
        networks[i].ssid[sizeof(networks[i].ssid) - 1] = '\0';
        networks[i].rssi = WiFi.RSSI(i);
        numSavedNetworks++;
      } else {
        WS_DEBUG_PRINT("ERROR: Too many networks found! (>");
        WS_DEBUG_PRINT(WS_MAX_SORTED_NETWORKS);
        WS_DEBUG_PRINT(") Ignoring ");
        WS_DEBUG_PRINT(WiFi.SSID(i));
        WS_DEBUG_PRINT("(");
        WS_DEBUG_PRINT(WiFi.RSSI(i));
        WS_DEBUG_PRINTLN(")");
      }
    }

    // Sort the networks by RSSI in descending order
    std::sort(networks, networks + numSavedNetworks, compareByRSSI);

    // Was the network within secrets.json found?
    for (int i = 0; i < numSavedNetworks; i++) {
      if (strcmp(_ssid, networks[i].ssid) == 0) {
        WS_DEBUG_PRINT("SSID (");
        WS_DEBUG_PRINT(_ssid);
        WS_DEBUG_PRINT(") found! RSSI: ");
        WS_DEBUG_PRINTLN(networks[i].rssi);
        return true;
      }
      if (WS._isWiFiMulti) {
        // multi network mode
        for (int j = 0; j < WS_MAX_ALT_WIFI_NETWORKS; j++) {
          if (strcmp(WS._multiNetworks[j].ssid, networks[i].ssid) == 0) {
            WS_DEBUG_PRINT("SSID (");
            WS_DEBUG_PRINT(WS._multiNetworks[j].ssid);
            WS_DEBUG_PRINT(") found! RSSI: ");
            WS_DEBUG_PRINTLN(networks[i].rssi);
            return true;
          }
        }
      }
    }

    // User-set network not found, print scan results to serial console
    WS_DEBUG_PRINTLN("ERROR: Your WiFi network was not found!");
    WS_DEBUG_PRINTLN("WipperSnapper found these WiFi networks:");
    for (uint8_t i = 0; i < n; i++)
    {
      WS_DEBUG_PRINT(WiFi.SSID(i));
      WS_DEBUG_PRINT(" (");
      const uint8_t* BSSID = WiFi.BSSID(i);
      for (int m = 0; m < WL_MAC_ADDR_LENGTH; m++)
      {
        if (m != 0)
          WS_DEBUG_PRINT(":");
        WS_DEBUG_PRINTHEX(BSSID[m]);
      }
      WS_DEBUG_PRINT(") ");
      WS_DEBUG_PRINT(WiFi.RSSI(i));
      WS_DEBUG_PRINT("dB (ch");
      WS_DEBUG_PRINT(WiFi.channel(i))
      WS_DEBUG_PRINTLN(")");
    }

    return false;
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
  ESP8266WiFiMulti _wifiMulti;

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

      if (WS._isWiFiMulti) {
        // multi network mode
        for (int i = 0; i < WS_MAX_ALT_WIFI_NETWORKS; i++) {
          if (strlen(WS._multiNetworks[i].ssid) > 0 &&
              (_wifiMulti.existsAP(WS._multiNetworks[i].ssid) == false)) {
            // doesn't exist, add it
            _wifiMulti.addAP(WS._multiNetworks[i].ssid,
                             WS._multiNetworks[i].pass);
          }
        }
      }

      // add default network
      if (_wifiMulti.existsAP(_ssid) == false) {
        _wifiMulti.addAP(_ssid, _pass);
      }

      long startRetry = millis();
      WS_DEBUG_PRINTLN("CONNECTING");

      while (_wifiMulti.run(25000) != WL_CONNECTED &&
             millis() - startRetry < 10000) {
        // ESP8266 WDT requires yield() during a busy-loop so it doesn't bite
        yield();
      }

      if (WiFi.status() == WL_CONNECTED) {
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