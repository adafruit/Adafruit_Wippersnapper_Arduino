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

#ifdef ESP8266
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "Arduino.h"
#include "ESP8266WiFi.h"
#include "Wippersnapper.h"

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
  */
  /**************************************************************************/
  Wippersnapper_ESP8266() : Wippersnapper() {
    _ssid = 0;
    _pass = 0;
    _wifi_client = new WiFiClientSecure;
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
    _pass = ssidPassword;
  }

  /**********************************************************/
  /*!
  @brief  Sets the WiFi client's ssid and password from the
            ESP8266's LittleFS.
  */
  /**********************************************************/
  void set_ssid_pass() {
    _ssid = WS._network_ssid;
    _pass = WS._network_pass;
  }

  /********************************************************/
  /*!
  @brief  Gets the ESP8266's unique client identifier.
  @note   For the ESP8266, the UID is the MAC address.
  */
  /********************************************************/
  void setUID() {
    WiFi.macAddress(mac);
    memcpy(WS._uid, mac, sizeof(mac));
  }

  /*******************************************************************/
  /*!
  @brief  Sets up an Adafruit_MQTT_Client
  @param  clientID
          MQTT client identifier
  @param  useStaging
          True to use the Adafruit.io staging broker,
            False otherwise.
  */
  /*******************************************************************/
  void setupMQTTClient(const char *clientID, bool useStaging = false) {
    if (useStaging) {
      _mqttBrokerURL = "io.adafruit.us";
      _wifi_client->setFingerprint(fingerprint_staging);
    } else {
      _mqttBrokerURL = "io.adafruit.com";
    }

    WS._mqtt =
        new Adafruit_MQTT_Client(_wifi_client, _mqttBrokerURL, _mqtt_port,
                                 clientID, WS._username, WS._key);
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
  const char *_mqttBrokerURL = NULL;
  uint8_t mac[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  const char *fingerprint =
      "59 3C 48 0A B1 8B 39 4E 0D 58 50 47 9A 13 55 60 CC A0 1D AF";
  ; ///< AIO Production SSL Fingerprint
  const char
      *fingerprint_staging = "A0 A4 61 E0 D6 F8 94 FF FA C1 F2 DC 09 23 72 E1 "
                             "CC 23 CD 64"; ///< AIO Staging SSL Fingerprint
  WiFiClientSecure *_wifi_client;

  /**************************************************************************/
  /*!
  @brief  Establishes a connection with the wireless network.
  */
  /**************************************************************************/
  void _connect() {

    if (WiFi.status() == WL_CONNECTED)
      return;

    if (strlen(_ssid) == 0) {
      // TODO: The network _ssid and _password should be validated within the
      // filesystem provisioning check, not here..
      _status = WS_SSID_INVALID;
      return;
    } else {
      _disconnect();
      delay(100);
      // ESP8266 MUST be in STA mode to avoid device acting as client/server
      WiFi.mode(WIFI_STA);
      WiFi.begin(_ssid, _pass);
      _status = WS_NET_DISCONNECTED;
      delay(100);
    }

    // wait for a connection to be established
    long startRetry = millis();
    WS_DEBUG_PRINTLN("CONNECTING");
    while (WiFi.status() != WL_CONNECTED && millis() - startRetry < 10000) {
      // ESP8266 WDT requires yield() during a busy-loop so it doesn't bite
      yield();
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

#endif // ESP8266 Arduino
#endif // WIPPERSNAPPER_ESP8266_H