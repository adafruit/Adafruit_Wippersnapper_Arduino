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
  "1.7.7" /*!< min. nina-fw version compatible with this library. */

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
    _ssPin = SPIWIFI_SS; // 10;
    _ackPin = SPIWIFI_ACK; //7;
    _rstPin = SPIWIFI_RESET; // 5; // should be 7 on PyPortals
    #ifdef ESP32_GPIO0
    _gpio0Pin = ESP32_GPIO0;
    #else
    _gpio0Pin = -1;
    #endif
    _wifi = &SPIWIFI;
    _ssid = 0;
    _pass = 0;
    _mqtt_client = new WiFiSSLClient;

    // setup ESP32 co-processor pins during init.
    WiFi.setPins(_ssPin, _ackPin, _rstPin, _gpio0Pin, _wifi);
    delay(1000);
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
    _ssid = WS._config.network.ssid;
    _pass = WS._config.network.pass;
  }

  /***********************************************************/
  /*!
  @brief   Performs a scan of local WiFi networks.
  @returns True if `_network_ssid` is found, False otherwise.
  */
  /***********************************************************/
  bool check_valid_ssid() {
    // Disconnect WiFi from an AP if it was previously connected
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
      if (strcmp(_ssid, WiFi.SSID(i)) == 0) {
        WS._RSSI = WiFi.RSSI(i);
        return true;
      }
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
    return compareVersions(_fv, NINAFWVER);
  }

  /********************************************************/
  /*!
  @brief  Compares two version strings.
  @param  currentVersion
          Current version string.
  @param  requiredVersion
          Required version string.
  @returns True if the current version is greater than or
          equal to the required version, False otherwise.
  */
  /********************************************************/
  bool compareVersions(const String& currentVersion, const String& requiredVersion) {
      int curMajor, curMinor, curPatch;
      int reqMajor, reqMinor, reqPatch;
      int per_major, per_minor, per_patch;
      
      sscanf(currentVersion.c_str(), "%d.%d.%d", &curMajor, &curMinor, &curPatch);
      sscanf(requiredVersion.c_str(), "%d.%d.%d", &reqMajor, &reqMinor, &reqPatch);

      if (curMajor != reqMajor) return curMajor > reqMajor;
      if (curMinor != reqMinor) return curMinor > reqMinor;
      return curPatch >= reqPatch;
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
  @brief  Gets the current network RSSI value
  */
  /********************************************************/
  void getRSSI() {
    // test if this fails when disconnected or returns something sensible
    WS._RSSI = WiFi.RSSI();
  }

  /********************************************************/
  /*!
  @brief  Initializes the MQTT client.
  @param  clientID
          MQTT client identifier
  */
  /********************************************************/
  void setupMQTTClient(const char *clientID) {
    WS._mqtt = new Adafruit_MQTT_Client(
        _mqtt_client, WS._config.aio_url, WS._config.io_port, clientID,
        WS._config.aio_user, WS._config.aio_key);
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
      _status = WS_SSID_INVALID;  // possibly unneccesary  as already checking elsewhere
    } else {
      // disconnect from possible previous connection
      _disconnect();
      delay(100);
      WiFi.end();
      _wifi->end();
      delay(100);
      _wifi->begin();
      feedWDT();
      WS_DEBUG_PRINT("Reset Pin: ");
      WS_DEBUG_PRINTLN(_rstPin);
      // reset the esp32 if possible
      if (_rstPin != -1) {
        WS_DEBUG_PRINTLN("Resetting ESP32...");
        WS_PRINTER.flush();
        // Chip select for esp32
        pinMode(_ssPin, OUTPUT);
        digitalWrite(_ssPin, HIGH); // Do we need to set SS low again?
        if (_gpio0Pin != -1) {
          pinMode(_gpio0Pin, OUTPUT);
          digitalWrite(_gpio0Pin, LOW);
        }
        pinMode(_rstPin, OUTPUT);
        digitalWrite(_rstPin, LOW);
        delay(50);
        digitalWrite(_rstPin, HIGH);
        delay(10);
        if (_gpio0Pin != -1) {
          pinMode(_gpio0Pin, INPUT);
        }
        // wait for the ESP32 to boot
        delay(2000);
      }
      feedWDT();
      WS_DEBUG_PRINT("ESP32 booted, version: ");
      WS_PRINTER.flush();
      WS_DEBUG_PRINTLN(WiFi.firmwareVersion());
      WS_PRINTER.flush();
      feedWDT();

      // // validate co-processor is physically connected connection
      // if (WiFi.status() == WL_NO_MODULE) {
      //   WS_DEBUG_PRINT("No ESP32 module detected!");
      //   WS_DEBUG_PRINT("Current Module Status:");
      //   WS_DEBUG_PRINTLN(WiFi.status());
      //   return;
      // }

      // validate co-processor's firmware version
      if (!firmwareCheck())
        WS_DEBUG_PRINTLN("Please upgrade the firmware on the ESP module to the "
                         "latest version.");

      WS_DEBUG_PRINT("Connecting to ");
      WS_DEBUG_PRINTLN(_ssid);
      WS_PRINTER.flush();
      feedWDT();
      WiFi.begin(_ssid, _pass);
      _status = WS_NET_DISCONNECTED;
      feedWDT();
      delay(5000);
      feedWDT();
      delay(5000);
      feedWDT();
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