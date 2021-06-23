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
#include "WiFiClientSecure.h"
#include "Wippersnapper.h"

// latest fingerprint can be generated with
// echo | openssl s_client -connect io.adafruit.com:443 | openssl x509
// -fingerprint -noout
#define WS_SSL_FINGERPRINT                                                     \
  "59 3C 48 0A B1 8B 39 4E 0D 58 50 47 9A 13 55 60 CC A0 1D AF" ///< Latest
                                                                ///< Adafruit IO
                                                                ///< SSL
                                                                ///< Fingerprint
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
    _mqtt_client->setFingerprint(WS_SSL_FINGERPRINT);
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
  */
  /*******************************************************************/
  void setupMQTTClient(const char *clientID) {
    WS._mqtt =
        new Adafruit_MQTT_Client(_mqtt_client, WS._mqtt_broker, WS._mqtt_port,
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
  const char *_ssid;
  const char *_pass;
  uint8_t mac[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  WiFiClientSecure *_mqtt_client;

  /**************************************************************************/
  /*!
  @brief  Establishes a connection with the wireless network.
  */
  /**************************************************************************/
  void _connect() {
    if (strlen(_ssid) == 0) {
      _status = WS_SSID_INVALID;
    } else {
      delay(1000);
      WiFi.begin(_ssid, _pass);
      delay(2000);
      _status = WS_NET_DISCONNECTED;
    }
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
