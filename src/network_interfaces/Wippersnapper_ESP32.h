/*!
 * @file Wippersnapper_ESP32.h
 *
 * This is a driver for using the ESP32's network interface
 * with Adafruit IO Wippersnapper.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2020-2024 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef Wippersnapper_ESP32_H
#define Wippersnapper_ESP32_H

#ifdef ARDUINO_ARCH_ESP32
#include "Wippersnapper.h"

#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "Arduino.h"
#include "WiFi.h"
#include "WiFiMulti.h"
#include <NetworkClient.h>
#include <NetworkClientSecure.h>
extern Wippersnapper WS;

/****************************************************************************/
/*!
    @brief  Class for using the ESP32 network interface.
*/
/****************************************************************************/
class Wippersnapper_ESP32 : public Wippersnapper {

public:
  /**************************************************************************/
  /*!
  @brief  Initializes the Adafruit IO class for ESP32 devices.
  */
  /**************************************************************************/
  Wippersnapper_ESP32() : Wippersnapper() {
    _ssid = 0;
    _pass = 0;
  }

  /**************************************************************************/
  /*!
  @brief  Destructor for the Adafruit IO AirLift class.
  */
  /**************************************************************************/
  ~Wippersnapper_ESP32() {
    if (_mqtt_client_secure)
      delete _mqtt_client_secure;
    if (_mqtt_client_insecure)
      delete _mqtt_client_insecure;
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

  /****************************************************************/
  /*!
  @brief  a structure to hold network information for sorting
  */
  /****************************************************************/
  struct WiFiNetwork {
    char ssid[33]; /*!< SSID (Max 32 characters + null terminator */
    int32_t rssi;  /*!< Received Signal Strength Indicator */
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

// For boards with a "3D Antenna", we need to reduce the TX power
// to prevent flaky operation.
// NOTE: This is a known issue with the QT Py series of boards.
#ifdef ARDUINO_ADAFRUIT_QTPY_ESP32S2 ||                                        \
    ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM ||                                   \
    ARDUINO_ADAFRUIT_QTPY_ESP32S3_N4R2 || ARDUINO_ADAFRUIT_QTPY_ESP32C3 ||     \
    ARDUINO_ADAFRUIT_QTPY_ESP32_PICO
    WiFi.setTxPower(WIFI_POWER_15dBm);
#endif

    // Perform a network scan
    int n = WiFi.scanNetworks();
    if (n == 0) {
      WS_DEBUG_PRINTLN("ERROR: No WiFi networks found!");
      return false;
    }

    WiFiNetwork networks[WS_MAX_SORTED_NETWORKS];
    uint8_t numSavedNetworks = 0;
    // Store the scanned networks in the vector
    for (int i = 0; i < n; ++i) {
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
    for (int i = 0; i < numSavedNetworks; ++i) {
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
    WS_DEBUG_PRINTLN("ERROR: Your requested WiFi network was not found!");
    WS_DEBUG_PRINT("WipperSnapper found these WiFi networks");
    if (n > WS_MAX_SORTED_NETWORKS) {
      WS_DEBUG_PRINT(" (only first ");
      WS_DEBUG_PRINT(WS_MAX_SORTED_NETWORKS);
      WS_DEBUG_PRINTLN(" used):");
    } else {
      WS_DEBUG_PRINTLN(":");
    }
    for (int i = 0; i < n; ++i) {
      WS_DEBUG_PRINT(networks[i].ssid);
      WS_DEBUG_PRINT(" ");
      WS_DEBUG_PRINT(networks[i].rssi);
      WS_DEBUG_PRINTLN("dB");
    }

    return false;
  }

  /********************************************************/
  /*!
  @brief  Sets the ESP32's unique client identifier
  @note   On ESP32, the UID is the MAC address.
  */
  /********************************************************/
  void getMacAddr() {
    uint8_t mac[6] = {0};
    Network.macAddress(mac);
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
      _mqtt_client_secure = new NetworkClientSecure();
      _mqtt_client_secure->setCACert(
          strcmp(WS._config.aio_url, "io.adafruit.com") == 0
              ? _aio_root_ca_prod
              : _aio_root_ca_staging);
      WS._mqtt = new Adafruit_MQTT_Client(
          _mqtt_client_secure, WS._config.aio_url, WS._config.io_port, clientID,
          WS._config.aio_user, WS._config.aio_key);
    } else {
      // Insecure connections require a NetworkClient object rather than a
      // NetworkClientSecure object
      _mqtt_client_insecure = new NetworkClient();
      WS._mqtt = new Adafruit_MQTT_Client(
          _mqtt_client_insecure, WS._config.aio_url, WS._config.io_port,
          clientID, WS._config.aio_user, WS._config.aio_key);
    }
  }

  /********************************************************/
  /*!
  @brief  Returns the network status of an ESP32 module.
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
  @return ESP32
  */
  /*******************************************************************/
  const char *connectionType() { return "ESP32"; }

protected:
  const char *_ssid; ///< WiFi SSID
  const char *_pass; ///< WiFi password
  NetworkClientSecure
      *_mqtt_client_secure; ///< Pointer to a secure network client object
  NetworkClient
      *_mqtt_client_insecure; ///< Pointer to an insecure network client object
  WiFiMulti _wifiMulti;       ///< WiFiMulti object for multi-network mode

  const char *_aio_root_ca_staging =
      "-----BEGIN CERTIFICATE-----\n"
      "MIIEVzCCAj+gAwIBAgIRAIOPbGPOsTmMYgZigxXJ/d4wDQYJKoZIhvcNAQELBQAw\n"
      "TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n"
      "cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMjQwMzEzMDAwMDAw\n"
      "WhcNMjcwMzEyMjM1OTU5WjAyMQswCQYDVQQGEwJVUzEWMBQGA1UEChMNTGV0J3Mg\n"
      "RW5jcnlwdDELMAkGA1UEAxMCRTUwdjAQBgcqhkjOPQIBBgUrgQQAIgNiAAQNCzqK\n"
      "a2GOtu/cX1jnxkJFVKtj9mZhSAouWXW0gQI3ULc/FnncmOyhKJdyIBwsz9V8UiBO\n"
      "VHhbhBRrwJCuhezAUUE8Wod/Bk3U/mDR+mwt4X2VEIiiCFQPmRpM5uoKrNijgfgw\n"
      "gfUwDgYDVR0PAQH/BAQDAgGGMB0GA1UdJQQWMBQGCCsGAQUFBwMCBggrBgEFBQcD\n"
      "ATASBgNVHRMBAf8ECDAGAQH/AgEAMB0GA1UdDgQWBBSfK1/PPCFPnQS37SssxMZw\n"
      "i9LXDTAfBgNVHSMEGDAWgBR5tFnme7bl5AFzgAiIyBpY9umbbjAyBggrBgEFBQcB\n"
      "AQQmMCQwIgYIKwYBBQUHMAKGFmh0dHA6Ly94MS5pLmxlbmNyLm9yZy8wEwYDVR0g\n"
      "BAwwCjAIBgZngQwBAgEwJwYDVR0fBCAwHjAcoBqgGIYWaHR0cDovL3gxLmMubGVu\n"
      "Y3Iub3JnLzANBgkqhkiG9w0BAQsFAAOCAgEAH3KdNEVCQdqk0LKyuNImTKdRJY1C\n"
      "2uw2SJajuhqkyGPY8C+zzsufZ+mgnhnq1A2KVQOSykOEnUbx1cy637rBAihx97r+\n"
      "bcwbZM6sTDIaEriR/PLk6LKs9Be0uoVxgOKDcpG9svD33J+G9Lcfv1K9luDmSTgG\n"
      "6XNFIN5vfI5gs/lMPyojEMdIzK9blcl2/1vKxO8WGCcjvsQ1nJ/Pwt8LQZBfOFyV\n"
      "XP8ubAp/au3dc4EKWG9MO5zcx1qT9+NXRGdVWxGvmBFRAajciMfXME1ZuGmk3/GO\n"
      "koAM7ZkjZmleyokP1LGzmfJcUd9s7eeu1/9/eg5XlXd/55GtYjAM+C4DG5i7eaNq\n"
      "cm2F+yxYIPt6cbbtYVNJCGfHWqHEQ4FYStUyFnv8sjyqU8ypgZaNJ9aVcWSICLOI\n"
      "E1/Qv/7oKsnZCWJ926wU6RqG1OYPGOi1zuABhLw61cuPVDT28nQS/e6z95cJXq0e\n"
      "K1BcaJ6fJZsmbjRgD5p3mvEf5vdQM7MCEvU0tHbsx2I5mHHJoABHb8KVBgWp/lcX\n"
      "GWiWaeOyB7RP+OfDtvi2OsapxXiV7vNVs7fMlrRjY1joKaqmmycnBvAq14AEbtyL\n"
      "sVfOS66B8apkeFX2NY4XPEYV4ZSCe8VHPrdrERk2wILG3T/EGmSIkCYVUMSnjmJd\n"
      "VQD9F6Na/+zmXCc=\n"
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

    if (strlen(_ssid) == 0) {
      _status = WS_SSID_INVALID;
    } else {
      WiFi.setAutoReconnect(false);
      _disconnect();
      delay(100);
      if (WS._isWiFiMulti) {
        // multi network mode
        _wifiMulti.APlistClean();
        _wifiMulti.setAllowOpenAP(false);
        // add default network
        _wifiMulti.addAP(_ssid, _pass);
        // add array of alternative networks
        for (int i = 0; i < WS_MAX_ALT_WIFI_NETWORKS; i++) {
          if (strlen(WS._multiNetworks[i].ssid) > 0) {
            _wifiMulti.addAP(WS._multiNetworks[i].ssid,
                             WS._multiNetworks[i].pass);
          }
        }
        if (_wifiMulti.run(20000) == WL_CONNECTED) {
          _status = WS_NET_CONNECTED;
        } else {
          _status = WS_NET_DISCONNECTED;
        }
      } else {
        // single network mode
        WiFi.begin(_ssid, _pass);
        _status = WS_NET_DISCONNECTED;
        WS.feedWDT();
        delay(5000);
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

#endif // ARDUINO_ARCH_ESP32_H
#endif // Wippersnapper_ESP32_H