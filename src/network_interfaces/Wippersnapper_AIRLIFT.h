/*!
 * @file Wippersnapper_AIRLIFT.h
 *
 * This is a driver for using the Adafruit AirLift
 * ESP32 Co-Processor's network interface with Wippersnapper.
 *
 * The ESP32 AirLift uses SPI to communicate. Three lines (CS, ACK, RST) are
 * required to communicate with the ESP32 AirLift.
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

#ifndef WIPPERSNAPPER_AIRLIFT_H
#define WIPPERSNAPPER_AIRLIFT_H

#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "Arduino.h"
#include "SPI.h"
#include "WiFiNINA.h"
#include "Wippersnapper.h"

#define NINAFWVER                                                              \
  "1.6.0" /*!< min. nina-fw version compatible with this library. */

#define SPIWIFI SPI /*!< Instance of SPI interface used by an AirLift. */

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
  Wippersnapper_AIRLIFT() : Wippersnapper() {
    _ssPin = 10;
    _ackPin = 7;
    _rstPin = 5;
    _gpio0Pin = -1;
    _wifi = &SPIWIFI;
    _ssid = 0;
    _pass = 0;
    _mqtt_client = new WiFiSSLClient;

    // setup ESP32 co-processor pins during init.
    WiFi.setPins(_ssPin, _ackPin, _rstPin, _gpio0Pin, _wifi);
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
  void set_ssid_pass(const char *ssid, const char *ssidPassword) {
    _ssid = ssid;
    _pass = ssidPassword;
  }

  /**********************************************************/
  /*!
  @brief  Sets the WiFi client's ssid and password from the
            secrets.json provisioning file.
  */
  /**********************************************************/
  void set_ssid_pass() {
    _ssid = WS._network_ssid;
    _pass = WS._network_pass;
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
  @param  gpio0Pin
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
  @brief    Checks the version of an ESP32 module running
            nina-fw.
  @returns  True if matches min. required to run
            WipperSnapper, False otherwise.
  */
  /********************************************************/
  bool firmwareCheck() {
    _fv = WiFi.firmwareVersion();
    if (_fv < NINAFWVER)
      return false;
    return true;
  }

  /********************************************************/
  /*!
  @brief  Gets the ESP32's unique client identifier.
  @note   For the ESP32, the UID is the MAC address.
  */
  /********************************************************/
  void getMacAddr() {
    uint8_t mac[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    WiFi.macAddress(mac);
    memcpy(WS._macAddr, mac, sizeof(mac));
  }

  /********************************************************/
  /*!
  @brief  Initializes the MQTT client.
  @param  clientID
          MQTT client identifier
  */
  /********************************************************/
  void setupMQTTClient(const char *clientID) {
    if (WS._mqttBrokerURL == nullptr)
      WS._mqttBrokerURL = "io.adafruit.com";

    WS._mqtt =
        new Adafruit_MQTT_Client(_mqtt_client, WS._mqttBrokerURL, WS._mqtt_port,
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
  @return AIRLIFT
  */
  /*******************************************************************/
  const char *connectionType() { return "AIRLIFT"; }

protected:
  const char *_ssid;           /*!< Network SSID. */
  const char *_pass;           /*!< Network password. */
  const char *_mqttBrokerURL;  /*!< MQTT broker URL. */
  String _fv;                  /*!< nina-fw firmware version. */
  int _ssPin = -1;             /*!< SPI S.S. pin. */
  int _ackPin = -1;            /*!< SPI ACK pin. */
  int _rstPin = -1;            /*!< SPI RST pin. */
  int _gpio0Pin = -1;          /*!< SPI GPIO0 pin, unused. */
  WiFiSSLClient *_mqtt_client; /*!< Instance of a secure WiFi client. */
  SPIClass *_wifi; /*!< Instance of the SPI bus used by the AirLift. */

  /**************************************************************************/
  /*!
  @brief  Establishes a connection with the wireless network.
  */
  /**************************************************************************/
  void _connect() {
    if (strlen(_ssid) == 0) {
      _status = WS_SSID_INVALID;
    } else {

      // validate co-processor is physically connected connection
      if (WiFi.status() == WL_NO_MODULE) {
        WS_DEBUG_PRINT("No ESP32 module detected!");
        return;
      }

      // validate co-processor's firmware version
      if (!firmwareCheck())
        WS_DEBUG_PRINTLN("Please upgrade the firmware on the ESP module to the "
                         "latest version.");

      // disconnect from possible previous connection
      _disconnect();

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

#endif // WIPPERSNAPPER_AIRLIFT_H