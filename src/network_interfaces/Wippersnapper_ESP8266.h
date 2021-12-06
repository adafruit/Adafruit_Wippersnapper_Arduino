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
#include "ESP8266WiFi.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "Arduino.h"
#include "Wippersnapper.h"

static const char *fingerprint PROGMEM = "A0 A4 61 E0 D6 F8 94 FF FA C1 F2 DC 09 23 72 E1 CC 23 CD 64";

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
    _mqtt_client = new WiFiClientSecure;
  }

  /**************************************************************************/
  /*!
  @brief  Destructor for the ESP8266's network iface.
  */
  /**************************************************************************/
  ~Wippersnapper_ESP8266() {
    if (_mqtt_client)
      delete _mqtt_client;
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

    WS_DEBUG_PRINTLN("setupMQTTClient");
    WS_DEBUG_PRINT("Using Staging? ");
    WS_DEBUG_PRINTLN(useStaging);

    if (useStaging == true) {
      _mqttBrokerURL = "io.adafruit.us";
    } else {
      _mqttBrokerURL = "io.adafruit.com";
    }
    WS_DEBUG_PRINTLN("****DUMP MQTT_CLIENT***");
    WS_DEBUG_PRINTLN(WS._username);
    WS_DEBUG_PRINTLN(WS._key);
    WS_DEBUG_PRINTLN(_mqttBrokerURL);
    WS_DEBUG_PRINTLN(WS._mqtt_port);
    WS_DEBUG_PRINTLN(clientID);
    WS_DEBUG_PRINTLN("****");
    WS._mqtt =
        new Adafruit_MQTT_Client(_mqtt_client, _mqttBrokerURL, 433,
                                 clientID, "brentrubell", "aio_QUHn80jW6oUVhJ2UtwAYxY9NYRRS");
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
  WiFiClientSecure *_mqtt_client;

  /**************************************************************************/
  /*!
  @brief  Establishes a connection with the wireless network.
  */
  /**************************************************************************/
  void _connect() {

  /* 
      Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
      would try to act as both a client and an access-point and could cause
      network-issues with your other WiFi-devices on your WiFi-network.
  */
  // We start by connecting to a WiFi network
  WS_DEBUG_PRINT("Connecting to ");
  WS_DEBUG_PRINTLN("Transit");
  WiFi.mode(WIFI_STA);
  WiFi.begin(_ssid, _pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    WS_DEBUG_PRINT(".");
  }
  WS_DEBUG_PRINT("");

  WS_DEBUG_PRINTLN("WiFi connected");
  WS_DEBUG_PRINTLN("IP address: ");
  WS_DEBUG_PRINTLN(WiFi.localIP());
  _mqtt_client->setFingerprint(fingerprint);

  }

  /**************************************************************************/
  /*!
      @brief  Disconnects from the wireless network.
  */
  /**************************************************************************/
  void _disconnect() {
    WiFi.disconnect();
    delay(300);
  }
};

#endif // ESP8266 Arduino
#endif // WIPPERSNAPPER_ESP8266_H