/*!
 * @file Wippersnapper_ESP8266.h
 *
 * This is a driver for using an ESP8266 with Wippersnapper.
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


#ifdef ESP8266


#include  "Wippersnapper.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "Arduino.h"
#include "ESP8266WiFi.h"
#include "WiFiClientSecure.h"

/****************************************************************************/
/*!
    @brief  Class for interacting with the Espressif ESP8266.
*/
/****************************************************************************/
class Wippersnapper_ESP8266 : public Wippersnapper {

public:
  /**************************************************************************/
  /*!
  @brief  Initializes the Adafruit IO class for ESP8266 devices.
  @param    aio_user
            Adafruit IO username.
  @param    aio_key
            Adafruit IO+ key.
  @param    ssid
            Desired WiFi network SSID.
  @param    ssidPassword
            Desired WiFi network password.
  */
  /**************************************************************************/
  Wippersnapper_ESP8266(const char *aio_user, const char *aio_key, const char *ssid,
                        const char *ssidPassword): Wippersnapper(aio_user, aio_key) {
    _ssid = ssid;
    _pass = ssidPassword;
    _aio_user = aio_user;
    _aio_key = aio_key;
    _client = new WiFiClientSecure;
    _client->setFingerprint(WS_SSL_FINGERPRINT);
    }

  /**************************************************************************/
  /*!
  @brief  Destructor for the Adafruit IO ESP8266.
  */
  /**************************************************************************/
  ~Wippersnapper_ESP8266() {
      if (_client)
        delete _client;
      if (_mqtt)
        delete _mqtt;
  }

  /********************************************************/
  /*!
  @brief  TODO: Get the ESP8266 MAC address
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
    _mqtt = new Adafruit_MQTT_Client(_mqtt_client, _mqtt_broker, _mqtt_port, _aio_user, _aio_key, clientID);
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
  @return "wifi"
  */
  /*******************************************************************/
  const char *connectionType() { return "wifi"; }

protected:
  const char *_ssid;
  const char *_pass;
  const char *_aio_user;
  const char *_aio_key;
  String _fv = "0.0.0";
  uint8_t mac[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

  WiFiClientSecure *_client;


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

#endif // ESP8266
