/*!
 * @file ws_networking_pico.h
 *
 * This is a driver for using the Raspberry Pi Pico (RP2040)
 * network interface with Adafruit IO Wippersnapper.
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

#ifndef WS_NETWORKING_PICO_H
#define WS_NETWORKING_PICO_H

#ifdef ARDUINO_ARCH_RP2040

#define PICO_CONNECT_TIMEOUT_MS 20000   /*!< Connection timeout (in ms) */
#define PICO_CONNECT_RETRY_DELAY_MS 200 /*!< delay time between retries. */

#include "Wippersnapper.h"

#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "Arduino.h"
#include <WiFiClient.h>
#include <WiFiClientSecure.h>

/****************************************************************************/
/*!
    @brief  Class for using the Raspberry Pi Pico network interface.
*/
/****************************************************************************/
class ws_networking_pico : public Wippersnapper {

public:
  /**************************************************************************/
  /*!
  @brief  Initializes the WipperSnapper class for RPi Pico.
  */
  /**************************************************************************/
  ws_networking_pico() : Wippersnapper() {
    _ssid = 0;
    _pass = 0;
  }

  /**************************************************************************/
  /*!
  @brief  Destructor
  */
  /**************************************************************************/
  ~ws_networking_pico() {
    if (_mqtt_client_secure)
      delete _mqtt_client_secure;
    if (_mqtt_client_secure)
      delete _mqtt_client_secure;
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
  @brief  Sets the WiFi client's ssid and password.
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

    bool foundNetwork = false;

    WS_DEBUG_PRINTLN("WipperSnapper found these WiFi networks:");
    for (uint8_t i = 0; i < n; i++) {
      if (!foundNetwork && strcmp(WiFi.SSID(i), _ssid) == 0) {
        foundNetwork = true;
      } else if (!foundNetwork && WS._isWiFiMulti) {
        // multi network mode
        for (int j = 0; j < WS_MAX_ALT_WIFI_NETWORKS; j++) {
          if (strcmp(WS._multiNetworks[j].ssid, WiFi.SSID(i)) == 0) {
            foundNetwork = true;
          }
        }
      }
      WS_DEBUG_PRINT(WiFi.SSID(i));
      WS_DEBUG_PRINT(" (");
      uint8_t BSSID[WL_MAC_ADDR_LENGTH];
      WiFi.BSSID(i, BSSID);
      for (int m = 0; m < WL_MAC_ADDR_LENGTH; m++) {
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

    if (!foundNetwork) {
      WS_DEBUG_PRINTLN("ERROR: Your requested WiFi network was not found!");
    }
    return foundNetwork;
  }

  /********************************************************/
  /*!
  @brief  Sets the RPi Pico's unique client identifier
  @note   On RPi Pico, the UID is the MAC address.
  */
  /********************************************************/
  void getMacAddr() {
    uint8_t mac[6] = {0};
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

  /********************************************************/
  /*!
  @brief  Initializes the MQTT client
  @param  clientID
          MQTT client identifier
  */
  /********************************************************/
  void setupMQTTClient(const char *clientID) {
    if (strcmp(WS._config.aio_url, "io.adafruit.com") == 0 ||
        strcmp(WS._config.aio_url, "io.adafruit.us") == 0) {
      _mqtt_client_secure = new WiFiClientSecure();
      _mqtt_client_secure->setCACert(
          strcmp(WS._config.aio_url, "io.adafruit.com") == 0
              ? _aio_root_ca_prod
              : _aio_root_ca_staging);
      WS._mqtt = new Adafruit_MQTT_Client(
          _mqtt_client_secure, WS._config.aio_url, WS._config.io_port, clientID,
          WS._config.aio_user, WS._config.aio_key);
    } else {
      _mqtt_client_insecure = new WiFiClient();
      WS._mqtt = new Adafruit_MQTT_Client(
          _mqtt_client_insecure, WS._config.aio_url, WS._config.io_port,
          clientID, WS._config.aio_user, WS._config.aio_key);
    }
  }

  /********************************************************/
  /*!
  @brief  Returns the network status of an RPi Pico.
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
  @return Pico
  */
  /*******************************************************************/
  const char *connectionType() { return "Pico"; }

protected:
  const char *_ssid; ///< WiFi SSID
  const char *_pass; ///< WiFi password
  WiFiClient
      *_mqtt_client_insecure; ///< Pointer to an insecure WiFi client object
  WiFiClientSecure
      *_mqtt_client_secure; ///< Pointer to a secure WiFi client object
  WiFiMulti _wifiMulti;     ///< WiFiMulti object for multi-network mode

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

  /**************************************************************************/
  /*!
  @brief  Establishes a connection with the wireless network.
  */
  /**************************************************************************/
  void _connect() {

    if (WiFi.status() == WL_CONNECTED)
      return;

    WiFi.mode(WIFI_STA);
    WS.feedWDT();
    WiFi.setTimeout(20000);
    WS.feedWDT();

    if (strlen(_ssid) == 0) {
      _status = WS_SSID_INVALID;
    } else {
      _disconnect();
      delay(5000);
      WS.feedWDT();
      if (WS._isWiFiMulti) {
        // multi network mode
        _wifiMulti.clearAPList();
        // add default network
        _wifiMulti.addAP(_ssid, _pass);
        // add array of alternative networks
        for (int i = 0; i < WS_MAX_ALT_WIFI_NETWORKS; i++) {
          _wifiMulti.addAP(WS._multiNetworks[i].ssid,
                           WS._multiNetworks[i].pass);
        }
        WS.feedWDT();
        if (_wifiMulti.run(20000) == WL_CONNECTED) {
          WS.feedWDT();
          _status = WS_NET_CONNECTED;
          return;
        }
        WS.feedWDT();
      } else {
        WiFi.begin(_ssid, _pass);

        // Use the macro to retry the status check until connected / timed out
        int lastResult;
        RETRY_FUNCTION_UNTIL_TIMEOUT(
            []() -> int { return WiFi.status(); }, // Function call each cycle
            int,                                   // return type
            lastResult, // return variable (unused here)
            [](int status) { return status == WL_CONNECTED; }, // check
            PICO_CONNECT_TIMEOUT_MS,      // timeout interval (ms)
            PICO_CONNECT_RETRY_DELAY_MS); // interval between retries

        if (lastResult == WL_CONNECTED) {
          _status = WS_NET_CONNECTED;
          // wait 2seconds for connection to stabilize
          WS_DELAY_WITH_WDT(2000);
          return;
        }
      }
      _status = WS_NET_DISCONNECTED;
    }
  }

  /**************************************************************************/
  /*!
      @brief  Disconnects from the wireless network.
  */
  /**************************************************************************/
  void _disconnect() {
    WS.feedWDT();
    WiFi.disconnect();
    delay(5000);
    WS.feedWDT();
  }
};

#endif // ARDUINO_ARCH_RP2040
#endif // WS_NETWORKING_PICO_H