/*!
 * @file BlinkaConnect_AIRLIFT.h
 *
 * This is a driver for using the Adafruit AirLift 
 * ESP32 Co-Processor with BlinkaConnect.
 *
 * The ESP32 uses SPI to communicate. Three lines (CS, ACK, RST) are required
 * to communicate with the ESP32.
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

#ifndef BLINKACONNECT_AIRLIFT_H
#define BLINKACONNECT_AIRLIFT_H

#include "BlinkaConnect.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "Arduino.h"
#include "SPI.h"
#include "WiFiNINA.h"

#define NINAFWVER "1.0.0" /*!< min. nina-fw version compatible with this library. */

/****************************************************************************/
/*!
    @brief  Class for interacting with AirLift Co-Processors.
*/
/****************************************************************************/
class BlinkaConnect_AIRLIFT : public BlinkaConnect {

public:
  /**************************************************************************/
  /*!
  @brief  Initializes the Adafruit IO class for AirLift devices.
  @param    deviceId
            The Adafruit IO+ device identifier, TODO
  @param    ssid
            The WiFi network's SSID.
  @param    ssidPassword
            The WiFi network's password.
  @param    ssPin
            The ESP32's S.S. pin.
  @param    ackPin
            The ESP32's ACK pin.
  @param    rstPin
            The ESP32's RST pin.
  @param    gpio0Pin
            The ESP32's gpio0 pin.
  @param    wifi
            a SPIClass
  */
  /**************************************************************************/
  BlinkaConnect_AIRLIFT(const char *ssid, const char *ssidPassword,
                        int ssPin, int ackPin, int rstPin,
                        int gpio0Pin, SPIClass *wifi): BlinkaConnect() {
    _wifi = wifi;
    _ssPin = ssPin;
    _ackPin = ackPin;
    _rstPin = rstPin;
    _gpio0Pin = gpio0Pin;
    _ssid = ssid;
    _pass = ssidPassword;

    _mqtt_client = new WiFiClient;
    _mqtt = new Adafruit_MQTT_Client(_mqtt_client, _mqtt_broker, _mqtt_port,
                                        _deviceId, "", "");
    }

  /**************************************************************************/
  /*!
  @brief  Destructor for the Adafruit IO AirLift class.
  */
  /**************************************************************************/
  ~BlinkaConnect_AIRLIFT() {
      if (_mqtt)
        delete _mqtt;
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
        BC_DEBUG_PRINTLN("Please upgrade the firmware on the ESP module to the latest version.");
    }
  }

  /********************************************************/
  /*!
  @brief  Returns the network status of an ESP32 module.
  @return bc_status_t
  */
  /********************************************************/
  bc_status_t networkStatus() {
    switch (WiFi.status()) {
    case WL_CONNECTED:
      return BC_NET_CONNECTED;
    case WL_CONNECT_FAILED:
      return BC_NET_CONNECT_FAILED;
    case WL_IDLE_STATUS:
      return BC_IDLE;
    default:
      return BC_NET_DISCONNECTED;
    }
  }

  /*******************************************************************/
  /*!
  @brief  Returns the type of network connection used by BlinkaConnect
  @return AIRLIFT
  */
  /*******************************************************************/
  const char *connectionType() { return "AIRLIFT"; }

protected:
  const char *_ssid;
  const char *_pass;
  String _fv = "0.0.0";
  int _ssPin, _ackPin, _rstPin, _gpio0Pin = -1;

  WiFiClient *_mqtt_client;

  SPIClass *_wifi;

  /**************************************************************************/
  /*!
  @brief  Establishes a connection with the wireless network.
  */
  /**************************************************************************/
  void _connect() {
    if (strlen(_ssid) == 0) {
      _status = BC_SSID_INVALID;
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
        BC_DEBUG_PRINT("No ESP32 module detected!");
        return;
      }

      WiFi.begin(_ssid, _pass);
      _status = BC_NET_DISCONNECTED;
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

#endif //BLINKACONNECT_AIRLIFT_H