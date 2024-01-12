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
 * Copyright (c) Brent Rubell 2020-2021 for Adafruit Industries.
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
#include <WiFiClientSecure.h>
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
    _mqtt_client = new WiFiClientSecure;
  }

  /**************************************************************************/
  /*!
  @brief  Destructor for the Adafruit IO AirLift class.
  */
  /**************************************************************************/
  ~Wippersnapper_ESP32() {
    if (_mqtt_client)
      delete _mqtt_client;
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
    _ssid = WS._network_ssid;
    _pass = WS._network_pass;
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
      if (strcmp(_ssid, WiFi.SSID(i).c_str()) == 0)
        return true;
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
  @brief  Sets the ESP32's unique client identifier
  @note   On ESP32, the UID is the MAC address.
  */
  /********************************************************/
  void getMacAddr() {
    uint8_t mac[6] = {0};
    WiFi.macAddress(mac);
    memcpy(WS._macAddr, mac, sizeof(mac));
  }

  /********************************************************/
  /*!
  @brief  Initializes the MQTT client
  @param  clientID
          MQTT client identifier
  */
  /********************************************************/
  void setupMQTTClient(const char *clientID) {
    u_int16_t port = 8883;
    if (WS._mqttBrokerURL == nullptr) {
      WS._mqttBrokerURL = "io.adafruit.com";
      _mqtt_client->setCACert(_aio_root_ca_prod);
    } else if (WS._mqttBrokerURL == "io.adafruit.vm"){
      // Set port to 1883 for non-ssl, or set SSL cert below
      //port = 1883;
      _mqtt_client->setCACert(_aio_root_ca_vm);
    } else if (WS._mqttBrokerURL == "io.adafruit.us") {
      _mqtt_client->setCACert(_aio_root_ca_staging);
    } else {
      // Assume non-ssl IP based testing
      port = 1883;
    }
    // Construct MQTT client
    WS._mqtt = new Adafruit_MQTT_Client(_mqtt_client, WS._mqttBrokerURL, port,
                                        clientID, WS._username, WS._key);
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
  const char *_ssid;              ///< WiFi SSID
  const char *_pass;              ///< WiFi password
  const char *_mqttBrokerURL;     ///< MQTT broker URL
  WiFiClientSecure *_mqtt_client; ///< Pointer to a WiFi client object (TLS/SSL)

  const char *_aio_root_ca_staging =
      "-----BEGIN CERTIFICATE-----\n"
      "MIIEZTCCA02gAwIBAgIQQAF1BIMUpMghjISpDBbN3zANBgkqhkiG9w0BAQsFADA/\n"
      "MSQwIgYDVQQKExtEaWdpdGFsIFNpZ25hdHVyZSBUcnVzdCBDby4xFzAVBgNVBAMT\n"
      "DkRTVCBSb290IENBIFgzMB4XDTIwMTAwNzE5MjE0MFoXDTIxMDkyOTE5MjE0MFow\n"
      "MjELMAkGA1UEBhMCVVMxFjAUBgNVBAoTDUxldCdzIEVuY3J5cHQxCzAJBgNVBAMT\n"
      "AlIzMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuwIVKMz2oJTTDxLs\n"
      "jVWSw/iC8ZmmekKIp10mqrUrucVMsa+Oa/l1yKPXD0eUFFU1V4yeqKI5GfWCPEKp\n"
      "Tm71O8Mu243AsFzzWTjn7c9p8FoLG77AlCQlh/o3cbMT5xys4Zvv2+Q7RVJFlqnB\n"
      "U840yFLuta7tj95gcOKlVKu2bQ6XpUA0ayvTvGbrZjR8+muLj1cpmfgwF126cm/7\n"
      "gcWt0oZYPRfH5wm78Sv3htzB2nFd1EbjzK0lwYi8YGd1ZrPxGPeiXOZT/zqItkel\n"
      "/xMY6pgJdz+dU/nPAeX1pnAXFK9jpP+Zs5Od3FOnBv5IhR2haa4ldbsTzFID9e1R\n"
      "oYvbFQIDAQABo4IBaDCCAWQwEgYDVR0TAQH/BAgwBgEB/wIBADAOBgNVHQ8BAf8E\n"
      "BAMCAYYwSwYIKwYBBQUHAQEEPzA9MDsGCCsGAQUFBzAChi9odHRwOi8vYXBwcy5p\n"
      "ZGVudHJ1c3QuY29tL3Jvb3RzL2RzdHJvb3RjYXgzLnA3YzAfBgNVHSMEGDAWgBTE\n"
      "p7Gkeyxx+tvhS5B1/8QVYIWJEDBUBgNVHSAETTBLMAgGBmeBDAECATA/BgsrBgEE\n"
      "AYLfEwEBATAwMC4GCCsGAQUFBwIBFiJodHRwOi8vY3BzLnJvb3QteDEubGV0c2Vu\n"
      "Y3J5cHQub3JnMDwGA1UdHwQ1MDMwMaAvoC2GK2h0dHA6Ly9jcmwuaWRlbnRydXN0\n"
      "LmNvbS9EU1RST09UQ0FYM0NSTC5jcmwwHQYDVR0OBBYEFBQusxe3WFbLrlAJQOYf\n"
      "r52LFMLGMB0GA1UdJQQWMBQGCCsGAQUFBwMBBggrBgEFBQcDAjANBgkqhkiG9w0B\n"
      "AQsFAAOCAQEA2UzgyfWEiDcx27sT4rP8i2tiEmxYt0l+PAK3qB8oYevO4C5z70kH\n"
      "ejWEHx2taPDY/laBL21/WKZuNTYQHHPD5b1tXgHXbnL7KqC401dk5VvCadTQsvd8\n"
      "S8MXjohyc9z9/G2948kLjmE6Flh9dDYrVYA9x2O+hEPGOaEOa1eePynBgPayvUfL\n"
      "qjBstzLhWVQLGAkXXmNs+5ZnPBxzDJOLxhF2JIbeQAcH5H0tZrUlo5ZYyOqA7s9p\n"
      "O5b85o3AM/OJ+CktFBQtfvBhcJVd9wvlwPsk+uyOy2HI7mNxKKgsBTt375teA2Tw\n"
      "UdHkhVNcsAKX1H7GNNLOEADksd86wuoXvg==\n"
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

  const char *_aio_root_ca_vm =
      "-----BEGIN CERTIFICATE-----\n"
      "MIIEZTCCAs2gAwIBAgIQek/t0uNiCDR9fE8Rm5Np3DANBgkqhkiG9w0BAQsFADCB\n"
      "lzEeMBwGA1UEChMVbWtjZXJ0IGRldmVsb3BtZW50IENBMTYwNAYDVQQLDC10eWV0\n"
      "aEBUeWV0aHMtTWFjQm9vay1Qcm8ubG9jYWwgKFR5ZXRoIEd1bmRyeSkxPTA7BgNV\n"
      "BAMMNG1rY2VydCB0eWV0aEBUeWV0aHMtTWFjQm9vay1Qcm8ubG9jYWwgKFR5ZXRo\n"
      "IEd1bmRyeSkwHhcNMjMxMjE1MTM1MzQ1WhcNMjYwMzE1MTM1MzQ0WjBhMScwJQYD\n"
      "VQQKEx5ta2NlcnQgZGV2ZWxvcG1lbnQgY2VydGlmaWNhdGUxNjA0BgNVBAsMLXR5\n"
      "ZXRoQFR5ZXRocy1NYWNCb29rLVByby5sb2NhbCAoVHlldGggR3VuZHJ5KTCCASIw\n"
      "DQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAKppU4wb6XV8Ac1pxj91NDZ+s3Yd\n"
      "AgSq/RqLZzjamoIAObOuc/tXJaDnqpHboS9+STO0+jg9Mx5Lz9qUBC0jiBHJX2SF\n"
      "HN4Qea4Vub9KaeK2clRZO6Xmm+Hhcr0jhWDZZTXhyFg6g5/KJ61U5KjQiUr/kY7r\n"
      "tZPoRDcQvfLS5xX+qznqH6aHH7n0WNsSF6OUzbOxCcueOBpYLVFmf4aoNAfxYUEI\n"
      "K8gDVYoAszE1yp9ii0fsLZ6IT9Gko6aIZ4ovBstC07yZ9E2tfhHzUJvSbi/vTudm\n"
      "Q/aQQKT/0zSstjx8nFXrvgqnDbkJ0uwdky7roldEyC4jZHhu/FsMrT2rxkkCAwEA\n"
      "AaNiMGAwDgYDVR0PAQH/BAQDAgWgMBMGA1UdJQQMMAoGCCsGAQUFBwMBMB8GA1Ud\n"
      "IwQYMBaAFAG/zRoUGs2SLwCskpRk49ZDuG70MBgGA1UdEQQRMA+CDSouYWRhZnJ1\n"
      "aXQudm0wDQYJKoZIhvcNAQELBQADggGBAAOroe4O3IqZZ8d+cuXGMnv6LsBAmKHn\n"
      "JnwxyCi6pmWNaexDZ0KIj9d5MIAA1f48YJUVGX7Y0H9YPNUeqv3pS6JvbWUj59kl\n"
      "oUj6uAZQgConA6vPJkm/pMTktnBWYYF68gCvTZRhtH7qXuTKL21X1/6l5cTOLijs\n"
      "mVq/ERZadHiogCxA+5c3QINNbev4xsf/pmw/ap3vfAnsyHneWKKCbi63jtGhbn4g\n"
      "SCoJsvH35i4hWXr9ddbW3Kfa01JhwsONryUtRN4SGi58lIRT07e9qciLAqxH+7Ka\n"
      "EE3eNOX61QOxbVMfJAeMPEMuG8iuqEtalXb+Dheosapac3irPpHIt47leLsVL5Le\n"
      "TeTUPhirSlJj35C6Xi2OfI5jCAvM62HV7AYjyimQttunLr24qwU4LqgM3i9ZUaPj\n"
      "ovBjEvjjy1vp0wKeS4Q8nUtZvwc5LgWW4ZH7KSTHS/adFU6qWOyPHwxxTjdSxhMB\n"
      "Sl3erUxbD7UgyDQQonzUBDI6I/lBnY58eQ==\n"
      "-----END CERTIFICATE-----\n";


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
      _disconnect();
      delay(100);
      WiFi.begin(_ssid, _pass);
      _status = WS_NET_DISCONNECTED;
      delay(5000);
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