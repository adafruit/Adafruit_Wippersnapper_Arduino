/*!
 * @file esp8266_wifi.h
 *
 * This is a driver for using the ESP8266's network interface
 *  with wippersnapper.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2020-2025 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef ESP8266_WIFI_H
#define ESP8266_WIFI_H

#ifdef ARDUINO_ARCH_ESP8266
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "ESP8266WiFi.h"
#include "ESP8266WiFiMulti.h"
#include "wippersnapper.h"

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

extern wippersnapper Ws;

/*!
    @brief  Class for interacting with the Espressif ESP8266's network
   interface.
*/
class esp8266_wifi : public wippersnapper {

public:
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
  esp8266_wifi() : wippersnapper() {
    _ssid = 0;
    _pass = 0;
    _wifi_client = new WiFiClient;
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
  }

  /*!
  @brief  Destructor for the ESP8266's network iface.
  */
  ~esp8266_wifi() {
    if (_wifi_client)
      delete _wifi_client;
    if (_mqttV2)
      delete _mqttV2;
  }

  /*!
  @brief  Sets the WiFi client's ssid and password.
  @param  ssid
            Wireless network's SSID.
  @param  ssidPassword
            Wireless network's password.
  */
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

  /*!
  @brief  Sets the WiFi client's ssid and password from the
            ESP8266's LittleFS.
  */
  void set_ssid_pass() {
    _ssid = Ws._configV2.network.ssid;
    _pass = Ws._configV2.network.pass;
  }

  /*!
  @brief   Performs a scan of local WiFi networks.
  @returns True if `_network_ssid` is found, False otherwise.
  */
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

    // Was the network within secrets.json found?
    for (int i = 0; i < n; ++i) {
      if (strcmp(_ssid, WiFi.SSID(i).c_str()) == 0) {
        WS_DEBUG_PRINT("SSID (");
        WS_DEBUG_PRINT(_ssid);
        WS_DEBUG_PRINT(") found! RSSI: ");
        WS_DEBUG_PRINTLN(WiFi.RSSI(i));
        return true;
      }
      if (Ws._isWiFiMultiV2) {
        // multi network mode
        for (int j = 0; j < WS_MAX_ALT_WIFI_NETWORKS; j++) {
          if (strcmp(Ws._multiNetworksV2[j].ssid, WiFi.SSID(i).c_str()) ==
              0) {
            WS_DEBUG_PRINT("SSID (");
            WS_DEBUG_PRINT(Ws._multiNetworksV2[j].ssid);
            WS_DEBUG_PRINT(") found! RSSI: ");
            WS_DEBUG_PRINTLN(WiFi.RSSI(i));
            return true;
          }
        }
      }
    }

    // User-set network not found, print scan results to serial console
    WS_DEBUG_PRINTLN("ERROR: Your requested WiFi network was not found!");
    WS_DEBUG_PRINTLN("WipperSnapper found these WiFi networks: ");
    for (int i = 0; i < n; ++i) {
      WS_DEBUG_PRINT(WiFi.SSID(i));
      WS_DEBUG_PRINT(" ");
      WS_DEBUG_PRINT(WiFi.RSSI(i));
      WS_DEBUG_PRINTLN("dB");
    }

    return false;
  }

  /*!
  @brief  Gets the ESP8266's unique client identifier.
  @note   For the ESP8266, the UID is the MAC address.
  */
  void getMacAddr() {
    uint8_t mac[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    WiFi.macAddress(mac);
    memcpy(Ws._macAddrV2, mac, sizeof(mac));
  }

  /*!
  @brief  Gets the current network RSSI value
  @return int32_t RSSI value
  */
  int32_t getRSSI() { return WiFi.RSSI(); }

  /*!
  @brief  Sets up an Adafruit_MQTT_Client
  @param  clientID
          MQTT client identifier
  */
  void setupMQTTClient(const char *clientID) {
    // Uncomment the following lines to use MQTT/SSL. You will need to
    // re-compile after. _wifi_client->setFingerprint(fingerprint); Ws._mqttV2
    // = new Adafruit_MQTT_Client(_wifi_client, Ws._configV2.aio_url,
    // Ws._configV2.io_port, clientID, Ws._configV2.aio_user,
    // Ws._configV2.aio_key);
    if (Ws._configV2.io_port == 8883)
      Ws._configV2.io_port = 1883;
    Ws._mqttV2 = new Adafruit_MQTT_Client(
        _wifi_client, Ws._configV2.aio_url, Ws._configV2.io_port, clientID,
        Ws._configV2.aio_user, Ws._configV2.aio_key);
  }

  /*!
  @brief  Returns the network status of an ESP8266 module.
  @return ws_status_t
  */
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

  /*!
  @brief  Returns the type of network connection used by wippersnapper
  @return "ESP8266"
  */
  const char *connectionType() { return "ESP8266"; }

protected:
  const char *_ssid = NULL;
  const char *_pass = NULL;
  WiFiClient *_wifi_client;
  ESP8266WiFiMulti _wifiMulti;

  /*!
  @brief  Establishes a connection with the wireless network.
  */
  void _connect() {

    if (WiFi.status() == WL_CONNECTED)
      return;

    if (strlen(_ssid) == 0) {
      _statusV2 = WS_SSID_INVALID;
    } else {
      WiFi.setAutoReconnect(false);
      // Attempt connection
      _disconnect();
      delay(100);
      // ESP8266 MUST be in STA mode to avoid device acting as client/server
      WiFi.mode(WIFI_STA);
      WiFi.begin(_ssid, _pass);
      _statusV2 = WS_NET_DISCONNECTED;
      delay(100);

      if (Ws._isWiFiMultiV2) {
        // multi network mode
        for (int i = 0; i < WS_MAX_ALT_WIFI_NETWORKS; i++) {
          if (strlen(Ws._multiNetworksV2[i].ssid) > 0 &&
              (_wifiMulti.existsAP(Ws._multiNetworksV2[i].ssid) == false)) {
            // doesn't exist, add it
            _wifiMulti.addAP(Ws._multiNetworksV2[i].ssid,
                             Ws._multiNetworksV2[i].pass);
          }
        }
        // add default network
        if (_wifiMulti.existsAP(_ssid) == false) {
          _wifiMulti.addAP(_ssid, _pass);
        }
        long startRetry = millis();
        WS_DEBUG_PRINTLN("CONNECTING");
        while (_wifiMulti.run(5000) != WL_CONNECTED &&
               millis() - startRetry < 10000) {
          // ESP8266 WDT requires yield() during a busy-loop so it doesn't bite
          yield();
        }
        if (WiFi.status() == WL_CONNECTED) {
          _statusV2 = WS_NET_CONNECTED;
        } else {
          _statusV2 = WS_NET_DISCONNECTED;
        }
      } else {
        // single network mode

        // wait for a connection to be established
        long startRetry = millis();
        WS_DEBUG_PRINTLN("CONNECTING");
        while (WiFi.status() != WL_CONNECTED && millis() - startRetry < 10000) {
          // ESP8266 WDT requires yield() during a busy-loop so it doesn't bite
          yield();
        }
        if (WiFi.status() == WL_CONNECTED) {
          _statusV2 = WS_NET_CONNECTED;
        } else {
          _statusV2 = WS_NET_DISCONNECTED;
        }
      }
      Ws._wdt->feed();
    }
  }

  /*!
      @brief  Disconnects from the wireless network.
  */
  void _disconnect() {
    WiFi.disconnect();
    delay(500);
  }
};

#endif // ARDUINO_ARCH_ESP8266
#endif // ESP8266_WIFI_H