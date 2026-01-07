/*!
 * @file esp32_wifi.h
 *
 * This is a driver for using the ESP32's network interface
 * with Adafruit IO Wippersnapper.
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

#ifndef ESP32_WIFI_H
#define ESP32_WIFI_H

#ifdef ARDUINO_ARCH_ESP32
#include "wippersnapper.h"

#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "Arduino.h"
#include "WiFi.h"
#include "WiFiMulti.h"
#include <NetworkClient.h>
#include <NetworkClientSecure.h>
extern wippersnapper Ws; ///< Wippersnapper client instance

/*!
    @brief  Class for using the ESP32 network interface.
*/
class esp32_wifi : public wippersnapper {

public:
  /*!
  @brief  Initializes the Adafruit IO class for ESP32 devices.
  */
  esp32_wifi() : wippersnapper() {
    _ssid = 0;
    _pass = 0;
  }

  /*!
  @brief  Overload for ESP32 devices without filesystem-backed provisioning.
    @param  aioUsername
                Adafruit IO username.
    @param  aioKey
                Adafruit IO key.
    @param  netSSID
                WiFi network's SSID.
    @param  netPass
                WiFi network's password.
    @param  brokerURL
                Adafruit IO broker URL.
    @param  brokerPort
                Adafruit IO broker port.
  */
  esp32_wifi(const char *aioUsername, const char *aioKey,
                const char *netSSID, const char *netPass, const char *brokerURL,
                uint16_t brokerPort)
      : wippersnapper() {
    _ssid = netSSID;
    _pass = netPass;

    // Move credentials to the config struct
    strncpy(Ws._configV2.network.ssid, _ssid,
            sizeof(Ws._configV2.network.ssid));
    strncpy(Ws._configV2.network.pass, _pass,
            sizeof(Ws._configV2.network.pass));
    strncpy(Ws._configV2.aio_key, aioKey, sizeof(Ws._configV2.aio_key));
    strncpy(Ws._configV2.aio_user, aioUsername,
            sizeof(Ws._configV2.aio_user));
    strncpy(Ws._configV2.aio_url, brokerURL, sizeof(Ws._configV2.aio_url));
    Ws._configV2.io_port = brokerPort;
  }

  /*!
  @brief  Destructor for the Adafruit IO AirLift class.
  */
  ~esp32_wifi() {
    if (_mqtt_client_secure)
      delete _mqtt_client_secure;
    if (_mqtt_client_insecure)
      delete _mqtt_client_insecure;
  }

  /*!
  @brief  Sets the WiFi client's ssid and password.
  @param  ssid
            WiFi network's SSID.
  @param  ssidPassword
            WiFi network's password.
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
  @brief  Sets the WiFi client's ssid and password.
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
        WiFi.scanDelete(); // Free the scan result memory
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
            WiFi.scanDelete(); // Free the scan result memory
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

    WiFi.scanDelete(); // Free the scan result memory

    return false;
  }

  /*!
  @brief  Sets the ESP32's unique client identifier
  @note   On ESP32, the UID is the MAC address.
  */
  void getMacAddr() {
    uint8_t mac[6] = {0};
    Network.macAddress(mac);
    memcpy(Ws._macAddrV2, mac, sizeof(mac));
  }

  /*!
  @brief  Gets the current network RSSI value
  @return int32_t RSSI value
  */
  int32_t getRSSI() { return WiFi.RSSI(); }

  /*!
  @brief  Initializes the MQTT client
  @param  clientID
          MQTT client identifier
  */
  void setupMQTTClient(const char *clientID) {
    if (strcmp(Ws._configV2.aio_url, "io.adafruit.com") == 0 ||
        strcmp(Ws._configV2.aio_url, "io.adafruit.us") == 0) {
      _mqtt_client_secure = new NetworkClientSecure();
      _mqtt_client_secure->setCACert(
          strcmp(Ws._configV2.aio_url, "io.adafruit.com") == 0
              ? _aio_root_ca_prod
              : _aio_root_ca_staging);
      Ws._mqttV2 = new Adafruit_MQTT_Client(
          _mqtt_client_secure, Ws._configV2.aio_url, Ws._configV2.io_port,
          clientID, Ws._configV2.aio_user, Ws._configV2.aio_key);
    } else {
      // Insecure connections require a NetworkClient object rather than a
      // NetworkClientSecure object
      _mqtt_client_insecure = new NetworkClient();
      Ws._mqttV2 = new Adafruit_MQTT_Client(
          _mqtt_client_insecure, Ws._configV2.aio_url, Ws._configV2.io_port,
          clientID, Ws._configV2.aio_user, Ws._configV2.aio_key);
    }
  }

  /*!
  @brief  Returns the network status of an ESP32 module.
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
  @brief  Returns the type of network connection used by Wippersnapper
  @return ESP32
  */
  const char *connectionType() { return "ESP32"; }

  /*!
  @brief  Sets the AIO key from the configuration.
  */
  void set_user_key() {
    // For ESP32, the key is already set in the constructor or from config
    // This method exists to satisfy the pure virtual function requirement
  }

protected:
  const char *_ssid; ///< WiFi SSID
  const char *_pass; ///< WiFi password
  NetworkClientSecure *_mqtt_client_secure = nullptr; ///< Pointer to a secure network client object
  NetworkClient *_mqtt_client_insecure = nullptr; ///< Pointer to an insecure network client object
  WiFiMulti _wifiMulti;       ///< WiFiMulti object for multi-network mode

  const char *_aio_root_ca_staging =
      "-----BEGIN CERTIFICATE-----\n"
      "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\n"
      "-----END CERTIFICATE-----\n"; ///< Root certificate for io.adafruit.us

  const char *_aio_root_ca_prod =
      "-----BEGIN CERTIFICATE-----\n"
      "MIIEjTCCA3WgAwIBAgIQDQd4KhM/xvmlcpbhMf/ReTANBgkqhkiG9w0BAQsFADBh\n"
      "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n"
      "d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\n"
      "MjAeFw0xNzExMDIxMjIzMzdaFw0yNzExMDIxMjIzMzdaMGAxCzAJBgNVBAYTAlVT\n"
      "MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n"
      "b20xHzAdBgNVBAMTFkdlb1RydXN0IFRMUyBSU0EgQ0EgRzEwggEiMA0GCSqGSIb3\n"
      "DQEBAQUAA4IBDwAwggEKAoIBAQC+F+jsvikKy/65LWEx/TMkCDIuWegh1Ngwvm4Q\n"
      "yISgP7oU5d79eoySG3vOhC3w/3jEMuipoH1fBtp7m0tTpsYbAhch4XA7rfuD6whU\n"
      "gajeErLVxoiWMPkC/DnUvbgi74BJmdBiuGHQSd7LwsuXpTEGG9fYXcbTVN5SATYq\n"
      "DfbexbYxTMwVJWoVb6lrBEgM3gBBqiiAiy800xu1Nq07JdCIQkBsNpFtZbIZhsDS\n"
      "fzlGWP4wEmBQ3O67c+ZXkFr2DcrXBEtHam80Gp2SNhou2U5U7UesDL/xgLK6/0d7\n"
      "6TnEVMSUVJkZ8VeZr+IUIlvoLrtjLbqugb0T3OYXW+CQU0kBAgMBAAGjggFAMIIB\n"
      "PDAdBgNVHQ4EFgQUlE/UXYvkpOKmgP792PkA76O+AlcwHwYDVR0jBBgwFoAUTiJU\n"
      "IBiV5uNu5g/6+rkS7QYXjzkwDgYDVR0PAQH/BAQDAgGGMB0GA1UdJQQWMBQGCCsG\n"
      "AQUFBwMBBggrBgEFBQcDAjASBgNVHRMBAf8ECDAGAQH/AgEAMDQGCCsGAQUFBwEB\n"
      "BCgwJjAkBggrBgEFBQcwAYYYaHR0cDovL29jc3AuZGlnaWNlcnQuY29tMEIGA1Ud\n"
      "HwQ7MDkwN6A1oDOGMWh0dHA6Ly9jcmwzLmRpZ2ljZXJ0LmNvbS9EaWdpQ2VydEds\n"
      "b2JhbFJvb3RHMi5jcmwwPQYDVR0gBDYwNDAyBgRVHSAAMCowKAYIKwYBBQUHAgEW\n"
      "HGh0dHBzOi8vd3d3LmRpZ2ljZXJ0LmNvbS9DUFMwDQYJKoZIhvcNAQELBQADggEB\n"
      "AIIcBDqC6cWpyGUSXAjjAcYwsK4iiGF7KweG97i1RJz1kwZhRoo6orU1JtBYnjzB\n"
      "c4+/sXmnHJk3mlPyL1xuIAt9sMeC7+vreRIF5wFBC0MCN5sbHwhNN1JzKbifNeP5\n"
      "ozpZdQFmkCo+neBiKR6HqIA+LMTMCMMuv2khGGuPHmtDze4GmEGZtYLyF8EQpa5Y\n"
      "jPuV6k2Cr/N3XxFpT3hRpt/3usU/Zb9wfKPtWpoznZ4/44c1p9rzFcZYrWkj3A+7\n"
      "TNBJE0GmP2fhXhP1D/XVfIW/h0yCJGEiV9Glm/uGOa3DXHlmbAcxSyCRraG+ZBkA\n"
      "7h4SeM6Y8l/7MBRpPCz6l8Y=\n"
      "-----END CERTIFICATE-----\n"; ///< Root certificate for io.adafruit.com

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
      _disconnect();
      delay(100);
      if (Ws._isWiFiMultiV2) {
        // multi network mode
        _wifiMulti.APlistClean();
        _wifiMulti.setAllowOpenAP(false);
        // add default network
        _wifiMulti.addAP(_ssid, _pass);
        // add array of alternative networks
        for (int i = 0; i < WS_MAX_ALT_WIFI_NETWORKS; i++) {
          if (strlen(Ws._multiNetworksV2[i].ssid) > 0) {
            _wifiMulti.addAP(Ws._multiNetworksV2[i].ssid,
                             Ws._multiNetworksV2[i].pass);
          }
        }
        if (_wifiMulti.run(20000) == WL_CONNECTED) {
          _statusV2 = WS_NET_CONNECTED;
        } else {
          _statusV2 = WS_NET_DISCONNECTED;
        }
      } else {
        // single network mode
        WiFi.begin(_ssid, _pass);
        _statusV2 = WS_NET_DISCONNECTED;
        Ws.feedWDTV2();
        delay(5000);
      }
      Ws.feedWDTV2();
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

#endif // ARDUINO_ARCH_ESP32_H
#endif // WS_WIFI_ESP32_H
