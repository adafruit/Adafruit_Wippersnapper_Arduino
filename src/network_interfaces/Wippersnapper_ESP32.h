/*!
 * @file Wippersnapper_ESP32.h
 *
 * This is a driver for using the ESP32 with Adafruit IO Wippersnapper
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Brent Rubell for Adafruit Industries.
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


/****************************************************************************/
/*!
    @brief  Class for using the ESP32 network interface.
*/
/****************************************************************************/
class Wippersnapper_ESP32 : public Wippersnapper {

public:
  /**************************************************************************/
  /*!
  @brief  Initializes the Adafruit IO class for AirLift devices.
  @param    aio_user
            Adafruit IO username.
  @param    aio_key
            Adafruit IO active key.
  @param    ssid
            The WiFi network's SSID.
  @param    ssidPassword
            The WiFi network's password.
  */
  /**************************************************************************/
  Wippersnapper_ESP32(const char *aio_user, const char *aio_key, const char *ssid,
                        const char *ssidPassword): Wippersnapper(aio_user, aio_key) {
    _ssid = ssid;
    _pass = ssidPassword;
    _aio_user = aio_user;
    _aio_key = aio_key;
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
  @brief  Gets the AirLift interface's MAC Address.
  */
  /********************************************************/
  void setUID() {
      WiFi.macAddress(mac);
      memcpy(_uid, mac, sizeof(mac));
  }

  /********************************************************/
  /*!
  @brief  Sets up an Adafruit_MQTT_Client
  @param  clientID
          MQTT client identifier
  */
  /********************************************************/
  void setupMQTTClient(const char *clientID) {
    _mqtt = new Adafruit_MQTT_Client(_mqtt_client, _mqtt_broker, \
                _mqtt_port, clientID, _aio_user, _aio_key);
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
  const char *_aio_user;
  const char *_aio_key;
  uint8_t mac[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

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
        _disconnect();
        delay(100);
        WiFi.begin(_ssid, _pass);
        delay(100);
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
    delay(500);
  }
};

#endif // ARDUINO_ARCH_ESP32
#endif //Wippersnapper_ESP32_H