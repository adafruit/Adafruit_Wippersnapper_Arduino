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

extern wippersnapper *Ws;

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
    disconnect();
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
    _ssid = Ws->_configV2.network.ssid;
    _pass = Ws->_configV2.network.pass;
  }

  /*!
  @brief   Performs a scan of local WiFi networks.
  @returns ws_ssid_scan_t result of the SSID scan.
  */
  ws_ssid_scan_t check_valid_ssid() {
    // Set WiFi to station mode and disconnect from an AP if it was previously
    // connected
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    // Perform a network scan, including hidden networks
    int n = WiFi.scanNetworks(false, true);
    if (n == 0) {
      WS_DEBUG_PRINTLN("ERROR: No WiFi networks found!");
      return WS_SSID_SCAN_NO_NETWORKS;
    }

    // Was the network within secrets.json found?
    bool foundHidden = false;
    for (int i = 0; i < n; ++i) {
      if (WiFi.SSID(i).length() == 0) {
        // hidden networks don't broadcast their SSID
        foundHidden = true;
        continue;
      }
      if (strcmp(_ssid, WiFi.SSID(i).c_str()) == 0) {
        WS_DEBUG_PRINT("SSID (");
        WS_DEBUG_PRINTVAR(_ssid);
        WS_DEBUG_PRINT(") found! RSSI: ");
        WS_DEBUG_PRINTLNVAR(WiFi.RSSI(i));
        return WS_SSID_SCAN_SSID_FOUND;
      }
      if (Ws->_isWiFiMultiV2) {
        // multi network mode
        for (int j = 0; j < WS_MAX_ALT_WIFI_NETWORKS; j++) {
          if (strlen(Ws->_multiNetworksV2[j].ssid) > 0 &&
              strcmp(Ws->_multiNetworksV2[j].ssid, WiFi.SSID(i).c_str()) == 0) {
            WS_DEBUG_PRINT("SSID (");
            WS_DEBUG_PRINTVAR(Ws->_multiNetworksV2[j].ssid);
            WS_DEBUG_PRINT(") found! RSSI: ");
            WS_DEBUG_PRINTLNVAR(WiFi.RSSI(i));
            return WS_SSID_SCAN_SSID_FOUND;
          }
        }
      }
    }

    // Print scan results before a directed scan replaces them
    WS_DEBUG_PRINTLN("WipperSnapper found these WiFi networks: ");
    for (int i = 0; i < n; ++i) {
      if (WiFi.SSID(i).length() == 0) {
        WS_DEBUG_PRINT("<hidden>");
      } else {
        WS_DEBUG_PRINTVAR(WiFi.SSID(i));
      }
      WS_DEBUG_PRINT(" ");
      WS_DEBUG_PRINTVAR(WiFi.RSSI(i));
      WS_DEBUG_PRINTLN("dB");
    }

    // Hidden networks only reveal their SSID in reply to a directed probe.
    // ESP8266WiFiMulti already tries hidden networks, so _connect() is as-is.
    if (foundHidden) {
      if (scan_hidden_ssid(_ssid))
        return WS_SSID_SCAN_SSID_FOUND;
      if (Ws->_isWiFiMultiV2) {
        for (int j = 0; j < WS_MAX_ALT_WIFI_NETWORKS; j++) {
          if (scan_hidden_ssid(Ws->_multiNetworksV2[j].ssid))
            return WS_SSID_SCAN_SSID_FOUND;
        }
      }
    }

    // User-set network not found
    WS_DEBUG_PRINTLN("ERROR: Your requested WiFi network was not found!");
    return WS_SSID_SCAN_SSID_NOT_FOUND;
  }

  /*!
  @brief   Performs a directed scan (probe request) for a hidden network.
  @param   ssid
           The SSID of the hidden network to scan for.
  @returns True if an access point responded to `ssid`, False otherwise.
  */
  bool scan_hidden_ssid(const char *ssid) {
    if (ssid == NULL || strlen(ssid) == 0)
      return false;

    WS_DEBUG_PRINT("Scanning for hidden WiFi network: ");
    WS_DEBUG_PRINTLNVAR(ssid);
    Ws->_wdt->feed();
    int n = WiFi.scanNetworks(false, true, 0, (uint8 *)ssid);
    Ws->_wdt->feed();
    for (int i = 0; i < n; i++) {
      // directed scans only report APs answering to `ssid`
      if (WiFi.SSID(i).length() == 0 ||
          strcmp(WiFi.SSID(i).c_str(), ssid) == 0) {
        WS_DEBUG_PRINT("Found hidden WiFi network on ch");
        WS_DEBUG_PRINTLNVAR(WiFi.channel(i));
        return true;
      }
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
    memcpy(Ws->_macAddrV2, mac, sizeof(mac));
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
    // re-compile after. _wifi_client->setFingerprint(fingerprint); Ws->_mqttV2
    // = new Adafruit_MQTT_Client(_wifi_client, Ws->_configV2.aio_url,
    // Ws->_configV2.io_port, clientID, Ws->_configV2.aio_user,
    // Ws->_configV2.aio_key);
    if (Ws->_configV2.io_port == 8883)
      Ws->_configV2.io_port = 1883;
    Ws->_mqttV2 = new Adafruit_MQTT_Client(
        _wifi_client, Ws->_configV2.aio_url, Ws->_configV2.io_port, clientID,
        Ws->_configV2.aio_user, Ws->_configV2.aio_key);
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

      if (Ws->_isWiFiMultiV2) {
        // multi network mode
        for (int i = 0; i < WS_MAX_ALT_WIFI_NETWORKS; i++) {
          if (strlen(Ws->_multiNetworksV2[i].ssid) > 0 &&
              (_wifiMulti.existsAP(Ws->_multiNetworksV2[i].ssid) == false)) {
            // doesn't exist, add it
            _wifiMulti.addAP(Ws->_multiNetworksV2[i].ssid,
                             Ws->_multiNetworksV2[i].pass);
          }
        }
        // add default network
        if (_wifiMulti.existsAP(_ssid) == false) {
          _wifiMulti.addAP(_ssid, _pass);
        }
        long startRetry = millis();
        WS_DEBUG_PRINTLN("CONNECTING");
        while (_wifiMulti.run(5 * ONE_SECOND_IN_MS) != WL_CONNECTED &&
               millis() - startRetry < TEN_SECONDS_IN_MS) {
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
        while (WiFi.status() != WL_CONNECTED &&
               millis() - startRetry < TEN_SECONDS_IN_MS) {
          // ESP8266 WDT requires yield() during a busy-loop so it doesn't bite
          yield();
        }
        if (WiFi.status() == WL_CONNECTED) {
          _statusV2 = WS_NET_CONNECTED;
        } else {
          _statusV2 = WS_NET_DISCONNECTED;
        }
      }
      Ws->_wdt->feed();
    }
  }

  /*!
      @brief  Disconnects from the wireless network.
      @param  wifi_off  Unused on this platform.
  */
  void _disconnect(bool wifi_off = true) {
    (void)wifi_off;
    WiFi.disconnect();
    delay(500);
  }
};

#endif // ARDUINO_ARCH_ESP8266
#endif // ESP8266_WIFI_H