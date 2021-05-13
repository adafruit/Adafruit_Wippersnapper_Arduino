/*!
 * @file Wippersnapper_AIRLIFT.h
 *
 * This is a driver for using the Adafruit AirLift 
 * ESP32 Co-Processor's network interface with Wippersnapper.
 *
 * The ESP32 AirLift uses SPI to communicate. Three lines (CS, ACK, RST) are required
 * to communicate with the ESP32 AirLift.
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

#ifndef WIPPERSNAPPER_AIRLIFT_H
#define WIPPERSNAPPER_AIRLIFT_H

#include "Wippersnapper.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "Arduino.h"
#include "SPI.h"
#include "WiFiNINA.h"

#define NINAFWVER "1.6.0" /*!< min. nina-fw version compatible with this library. */

extern Wippersnapper WS;
/****************************************************************************/
/*!
    @brief  Class for using the AirLift Co-Processor network iface.
*/
/****************************************************************************/
class Wippersnapper_AIRLIFT : public Wippersnapper {

public:
  /**************************************************************************/
  /*!
  @brief  Initializes the Adafruit IO class for AirLift devices.
  */
  /**************************************************************************/
  Wippersnapper_AIRLIFT(): Wippersnapper() {
    _ssPin = 0;
    _ackPin = 0;
    _rstPin = 0;
    _wifi = 0;
    _mqtt_client = 0;
    _ssid = 0;
    _pass = 0;
    }

  /**************************************************************************/
  /*!
  @brief  Destructor for the Adafruit IO AirLift class.
  */
  /**************************************************************************/
  ~Wippersnapper_AIRLIFT() {
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
  void set_ssid_pass(char *ssid, const char *ssidPassword) {
        _ssid = ssid;
        _pass = ssidPassword;
  }

  /********************************************************/
  /*!
  @brief  Sets the WiFi client.
  @param  wifi
          Instance of SPIClass.
  */
  /********************************************************/
  void set_wifi(SPIClass *wifi) {
    _wifi = wifi;
    _mqtt_client = new WiFiSSLClient;
  }

  /********************************************************/
  /*!
  @brief  Configures ESP32 "AirLift" pins.
  @param  ssPin
            ESP32 S.S. pin.
  @param  ackPin
            ESP32 ACK pin.
  @param  rstPin
            ESP32 RST pin.
  @param  gpio0pin
            ESP32 GPIO0 pin.

  */
  /********************************************************/
  void set_airlift_pins(int ssPin, int ackPin, int rstPin, int gpio0Pin) {
    _ssPin = ssPin;
    _ackPin = ackPin;
    _rstPin = rstPin;
    _gpio0Pin = gpio0Pin;
  }

  /********************************************************/
  /*!
  @brief  Checks the version of an ESP32 module against
  NINAFWVER. Raises an error if the firmware needs to be
  upgraded.
  */
  /********************************************************/
  void firmwareCheck() {
    _fv = WiFi.firmwareVersion();
    if (_fv < NINAFWVER) {
        WS_DEBUG_PRINTLN("Please upgrade the firmware on the ESP module to the latest version.");
    }
  }

  /********************************************************/
  /*!
  @brief  Gets the ESP32's unique client identifier.
  @note   For the ESP32, the UID is the MAC address.
  */
  /********************************************************/
  void getUID() {
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
    WS._mqtt = new Adafruit_MQTT_Client(_mqtt_client, WS._mqtt_broker, \
                WS._mqtt_port, clientID, WS._username, WS._key); 

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
  @return AIRLIFT
  */
  /*******************************************************************/
  const char *connectionType() { return "AIRLIFT"; }

protected:
  const char *_ssid;
  const char *_pass;
  String _fv = "0.0.0";
  uint8_t mac[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  int _ssPin, _ackPin, _rstPin, _gpio0Pin = -1;
  WiFiSSLClient *_mqtt_client;
  SPIClass *_wifi;

  /**************************************************************************/
  /*!
  @brief  Establishes a connection with the wireless network.
  */
  /**************************************************************************/
  void _connect() {
    if (strlen(_ssid) == 0) {
      _status = WS_SSID_INVALID;
    } else {
      // setup ESP32 pins
      if (_ssPin != -1) {
        WiFi.setPins(_ssPin, _ackPin, _rstPin, _gpio0Pin, _wifi);
      }

      // validate up-to-date nina-fw version
      firmwareCheck();

      // disconnect from possible previous connection
      _disconnect();

      // check for esp32 module
      if (WiFi.status() == WL_NO_MODULE) {
        WS_DEBUG_PRINT("No ESP32 module detected!");
        return;
      }

      WiFi.begin(_ssid, _pass);
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

#endif //WIPPERSNAPPER_AIRLIFT_H