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
#include "WiFiClientSecure.h"
#include <WiFi.h>
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
    _pass = ssidPassword;
  }

  /**********************************************************/
  /*!
  @brief  Sets the WiFi client's ssid and password from the
            esp32's nvs.
  */
  /**********************************************************/
  void set_ssid_pass() {
    _ssid = WS._network_ssid;
    _pass = WS._network_pass;
  }

  /********************************************************/
  /*!
  @brief  Sets the ESP32's unique client identifier
  @note   On ESP32, the UID is the MAC address.
  */
  /********************************************************/
  void setUID() {
    WiFi.macAddress(mac);
    memcpy(WS._uid, mac, sizeof(mac));
  }

  /********************************************************/
  /*!
  @brief  Initializes the MQTT client.
  @param  clientID
          MQTT client identifier
  */
  /********************************************************/
  void setupMQTTClient(const char *clientID) {
    WS._mqtt =
        new Adafruit_MQTT_Client(_mqtt_client, WS._mqtt_broker, WS._mqtt_port,
                                 clientID, WS._username, WS._key);
    _mqtt_client->setCACert(_aio_root_ca);
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
  const char *_ssid;
  const char *_pass;
  uint8_t mac[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  WiFiClientSecure *_mqtt_client;

  // io.adafruit.com root CA
  const char *_aio_root_ca =
      "-----BEGIN CERTIFICATE-----\n"
      "MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh\n"
      "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n"
      "d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD\n"
      "QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT\n"
      "MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n"
      "b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG\n"
      "9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB\n"
      "CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97\n"
      "nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt\n"
      "43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P\n"
      "T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4\n"
      "gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO\n"
      "BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR\n"
      "TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw\n"
      "DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr\n"
      "hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg\n"
      "06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF\n"
      "PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls\n"
      "YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk\n"
      "CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=\n"
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
      delay(1000);
    }

/*     // wait for a connection to be established
    long startRetry = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startRetry < 10000) {
      // do nothing, busy loop during the timeout
    } */
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