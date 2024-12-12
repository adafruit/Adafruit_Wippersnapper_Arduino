/*!
 * @file ws_networking_pico_v2.h
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

#ifndef WS_NETWORKING_PICO_V2_H
#define WS_NETWORKING_PICO_V2_H

#ifdef ARDUINO_RASPBERRY_PI_PICO_W

#define PICO_CONNECT_TIMEOUT_MS 20000   /*!< Connection timeout (in ms) */
#define PICO_CONNECT_RETRY_DELAY_MS 200 /*!< delay time between retries. */

#include "Wippersnapper_V2.h"

#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "Arduino.h"
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
extern Wippersnapper_V2 WsV2;

/****************************************************************************/
/*!
    @brief  Class for using the Raspberry Pi Pico network interface.
*/
/****************************************************************************/
class ws_networking_pico_v2 : public Wippersnapper_V2 {

public:
  /**************************************************************************/
  /*!
  @brief  Initializes the WipperSnapper class for RPi Pico.
  */
  /**************************************************************************/
  ws_networking_pico_v2() : Wippersnapper_V2() {
    _ssid = 0;
    _pass = 0;
  }

  /**************************************************************************/
  /*!
  @brief  Destructor
  */
  /**************************************************************************/
  ~ws_networking_pico_v2() {
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
    _ssid = WsV2._configV2.network.ssid;
    _pass = WsV2._configV2.network.pass;
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

    // Was the network within secrets.json found?
    for (int i = 0; i < n; ++i) {
      if (strcmp(_ssid, WiFi.SSID(i)) == 0) {
        WS_DEBUG_PRINT("SSID (");
        WS_DEBUG_PRINT(_ssid);
        WS_DEBUG_PRINT(") found! RSSI: ");
        WS_DEBUG_PRINTLN(WiFi.RSSI(i));
        return true;
      }
      if (WsV2._isWiFiMultiV2) {
        // multi network mode
        for (int j = 0; j < WS_MAX_ALT_WIFI_NETWORKS; j++) {
          if (strcmp(WsV2._multiNetworksV2[j].ssid, WiFi.SSID(i)) == 0) {
            WS_DEBUG_PRINT("SSID (");
            WS_DEBUG_PRINT(WsV2._multiNetworksV2[j].ssid);
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

  /********************************************************/
  /*!
  @brief  Sets the RPi Pico's unique client identifier
  @note   On RPi Pico, the UID is the MAC address.
  */
  /********************************************************/
  void getMacAddr() {
    uint8_t mac[6] = {0};
    WiFi.macAddress(mac);
    memcpy(WsV2._macAddrV2, mac, sizeof(mac));
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
    if (strcmp(WsV2._configV2.aio_url, "io.adafruit.com") == 0 ||
        strcmp(WsV2._configV2.aio_url, "io.adafruit.us") == 0) {
      _mqtt_client_secure = new WiFiClientSecure();
      _mqtt_client_secure->setCACert(
          strcmp(WsV2._configV2.aio_url, "io.adafruit.com") == 0
              ? _aio_root_ca_prod
              : _aio_root_ca_staging);
      WsV2._mqttV2 = new Adafruit_MQTT_Client(
          _mqtt_client_secure, WsV2._configV2.aio_url, WsV2._configV2.io_port, clientID,
          WsV2._configV2.aio_user, WsV2._configV2.aio_key);
    } else {
      _mqtt_client_insecure = new WiFiClient();
      WsV2._mqttV2 = new Adafruit_MQTT_Client(
          _mqtt_client_insecure, WsV2._configV2.aio_url, WsV2._configV2.io_port,
          clientID, WsV2._configV2.aio_user, WsV2._configV2.aio_key);
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
      "MIIEVzCCAj+gAwIBAgIRALBXPpFzlydw27SHyzpFKzgwDQYJKoZIhvcNAQELBQAw\n"
      "TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n"
      "cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMjQwMzEzMDAwMDAw\n"
      "WhcNMjcwMzEyMjM1OTU5WjAyMQswCQYDVQQGEwJVUzEWMBQGA1UEChMNTGV0J3Mg\n"
      "RW5jcnlwdDELMAkGA1UEAxMCRTYwdjAQBgcqhkjOPQIBBgUrgQQAIgNiAATZ8Z5G\n"
      "h/ghcWCoJuuj+rnq2h25EqfUJtlRFLFhfHWWvyILOR/VvtEKRqotPEoJhC6+QJVV\n"
      "6RlAN2Z17TJOdwRJ+HB7wxjnzvdxEP6sdNgA1O1tHHMWMxCcOrLqbGL0vbijgfgw\n"
      "gfUwDgYDVR0PAQH/BAQDAgGGMB0GA1UdJQQWMBQGCCsGAQUFBwMCBggrBgEFBQcD\n"
      "ATASBgNVHRMBAf8ECDAGAQH/AgEAMB0GA1UdDgQWBBSTJ0aYA6lRaI6Y1sRCSNsj\n"
      "v1iU0jAfBgNVHSMEGDAWgBR5tFnme7bl5AFzgAiIyBpY9umbbjAyBggrBgEFBQcB\n"
      "AQQmMCQwIgYIKwYBBQUHMAKGFmh0dHA6Ly94MS5pLmxlbmNyLm9yZy8wEwYDVR0g\n"
      "BAwwCjAIBgZngQwBAgEwJwYDVR0fBCAwHjAcoBqgGIYWaHR0cDovL3gxLmMubGVu\n"
      "Y3Iub3JnLzANBgkqhkiG9w0BAQsFAAOCAgEAfYt7SiA1sgWGCIpunk46r4AExIRc\n"
      "MxkKgUhNlrrv1B21hOaXN/5miE+LOTbrcmU/M9yvC6MVY730GNFoL8IhJ8j8vrOL\n"
      "pMY22OP6baS1k9YMrtDTlwJHoGby04ThTUeBDksS9RiuHvicZqBedQdIF65pZuhp\n"
      "eDcGBcLiYasQr/EO5gxxtLyTmgsHSOVSBcFOn9lgv7LECPq9i7mfH3mpxgrRKSxH\n"
      "pOoZ0KXMcB+hHuvlklHntvcI0mMMQ0mhYj6qtMFStkF1RpCG3IPdIwpVCQqu8GV7\n"
      "s8ubknRzs+3C/Bm19RFOoiPpDkwvyNfvmQ14XkyqqKK5oZ8zhD32kFRQkxa8uZSu\n"
      "h4aTImFxknu39waBxIRXE4jKxlAmQc4QjFZoq1KmQqQg0J/1JF8RlFvJas1VcjLv\n"
      "YlvUB2t6npO6oQjB3l+PNf0DpQH7iUx3Wz5AjQCi6L25FjyE06q6BZ/QlmtYdl/8\n"
      "ZYao4SRqPEs/6cAiF+Qf5zg2UkaWtDphl1LKMuTNLotvsX99HP69V2faNyegodQ0\n"
      "LyTApr/vT01YPE46vNsDLgK+4cL6TrzC/a4WcmF5SRJ938zrv/duJHLXQIku5v0+\n"
      "EwOy59Hdm0PT/Er/84dDV0CSjdR/2XuZM3kpysSKLgD1cKiDA+IRguODCxfO9cyY\n"
      "Ig46v9mFmBvyH04=\n"
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
    WsV2.feedWDTV2();
    WiFi.setTimeout(20000);
    WsV2.feedWDTV2();

    if (strlen(_ssid) == 0) {
      _statusV2 = WS_SSID_INVALID;
    } else {
      _disconnect();
      delay(5000);
      WsV2.feedWDTV2();
      if (WsV2._isWiFiMultiV2) {
        // multi network mode
        _wifiMulti.clearAPList();
        // add default network
        _wifiMulti.addAP(_ssid, _pass);
        // add array of alternative networks
        for (int i = 0; i < WS_MAX_ALT_WIFI_NETWORKS; i++) {
          _wifiMulti.addAP(WsV2._multiNetworksV2[i].ssid,
                           WsV2._multiNetworksV2[i].pass);
        }
        WsV2.feedWDTV2();
        if (_wifiMulti.run(10000) == WL_CONNECTED) {
          WsV2.feedWDTV2();
          _statusV2 = WS_NET_CONNECTED;
          return;
        }
        WsV2.feedWDTV2();
      } else {
        WiFi.begin(_ssid, _pass);

        // Use the macro to retry the status check until connected / timed out
        int lastResult;
/*         RETRY_FUNCTION_UNTIL_TIMEOUT(
            []() -> int { return WiFi.status(); }, // Function call each cycle
            int,                                   // return type
            lastResult, // return variable (unused here)
            [](int status) { return status == WL_CONNECTED; }, // check
            PICO_CONNECT_TIMEOUT_MS,      // timeout interval (ms)
            PICO_CONNECT_RETRY_DELAY_MS); // interval between retries */

        if (lastResult == WL_CONNECTED) {
          _statusV2 = WS_NET_CONNECTED;
          // wait 2seconds for connection to stabilize
          // WS_DELAY_WITH_WDT(2000);
          return;
        }
      }
      _statusV2 = WS_NET_DISCONNECTED;
    }
  }

  /**************************************************************************/
  /*!
      @brief  Disconnects from the wireless network.
  */
  /**************************************************************************/
  void _disconnect() {
    WsV2.feedWDTV2();
    WiFi.disconnect();
    delay(5000);
    WsV2.feedWDTV2();
  }
};

#endif // RASPBERRY_PI_PICO_W
#endif // WS_NETWORKING_PICO_V2_H