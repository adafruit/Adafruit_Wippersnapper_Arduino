/*!
 * @file Wippersnapper_WIFININA.h
 *
 * Network interface for the ublox wifi module on the
 * Arduino MKR WiFi 1010, Arduino Nano 33 IoT and Arduino UNO WiFi Rev.2.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2021 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef WIPPERSNAPPER_WIFININA_H
#define WIPPERSNAPPER_WIFININA_H
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <Arduino.h>
#include <SPI.h>
#include <WiFiNINA.h>

#include "Wippersnapper.h"

#define SPIWIFI                                                                \
  SPI /*!< Instance of SPI interface used by an external uBlox module. */

extern Wippersnapper WS;
/****************************************************************************/
/*!
    @brief  Class for using the AirLift Co-Processor network iface.
*/
/****************************************************************************/
class Wippersnapper_WIFININA : public Wippersnapper {

public:
  /**************************************************************************/
  /*!
  @brief  Initializes the Adafruit IO class for ublox devices.
  @param  aioUsername
          Adafruit IO username
  @param  aioKey
          Adafruit IO key
  @param  netSSID
          Wireless Network SSID
  @param  netPass
          Wireless Network password
  */
  /**************************************************************************/
  Wippersnapper_WIFININA(const char *aioUsername, const char *aioKey,
                         const char *netSSID, const char *netPass)
      : Wippersnapper() {
    _ssid = netSSID;
    _pass = netPass;
    _username = aioUsername;
    _key = aioKey;

    _wifi = &SPIWIFI;
    _mqtt_client = new WiFiSSLClient;
  }

  /**************************************************************************/
  /*!
  @brief  Destructor for the Adafruit IO ublox class.
  */
  /**************************************************************************/
  ~Wippersnapper_WIFININA() {
    if (_mqtt)
      delete _mqtt;
  }

  /****************************************************************************/
  /*!
      @brief    Configures the device's Adafruit IO credentials. This method
                should be used only if filesystem-backed provisioning is
                not avaliable.
  */
  /****************************************************************************/
  void set_user_key() {
    strlcpy(WS._config.aio_user, _username, sizeof(WS._config.aio_user));
    strlcpy(WS._config.aio_key, _key, sizeof(WS._config.aio_key));
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
    strlcpy(WS._config.network.ssid, ssid, sizeof(WS._config.network.ssid));
    strlcpy(WS._config.network.pass, ssidPassword,
            sizeof(WS._config.network.pass));
  }

  /**********************************************************/
  /*!
  @brief  Sets the WiFi client's ssid and password from the
          header file's credentials.
  */
  /**********************************************************/
  void set_ssid_pass() {
    strlcpy(WS._config.network.ssid, _ssid, sizeof(WS._config.network.ssid));
    strlcpy(WS._config.network.pass, _pass, sizeof(WS._config.network.pass));
  }

  /***********************************************************/
  /*!
  @brief   Performs a scan of local WiFi networks.
  @returns True if `_network_ssid` is found, False otherwise.
  */
  /***********************************************************/
  bool check_valid_ssid() {
    // Set WiFi to station mode and disconnect from an AP if it was previously
    // connected
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
      if (strcmp(_ssid, WiFi.SSID(i)) == 0)
        WS._RSSI = WiFi.RSSI(i);
        return true;
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

  /***********************************************************/
  /*!
  @brief   Checks the nina-fw version on the module.
  @return  True if firmware on the ublox module matches
           the latest version of the library, False otherwise.
  */
  /***********************************************************/
  bool firmwareCheck() {
    String fv = WiFi.firmwareVersion();
    if (fv < WIFI_FIRMWARE_LATEST_VERSION)
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
  const char *_ssid;     /*!< Network SSID. */
  const char *_pass;     /*!< Network password. */
  const char *_username; /*!< Adafruit IO username. */
  const char *_key;      /*!< Adafruit IO key. */

  WiFiSSLClient *_mqtt_client; /*!< Instance of a secure WiFi client. */
  SPIClass *_wifi; /*!< Instance of the SPI bus used by the ublox. */

  /**************************************************************************/
  /*!
  @brief  Establishes a connection with the wireless network.
  */
  /**************************************************************************/
  void _connect() {

    // check if co-processor connected first
    if (WiFi.status() == WL_NO_MODULE)
      errorWriteHang("No WiFi Module Detected!");

    // validate the nina-fw version
    if (!firmwareCheck())
      errorWriteHang("Please upgrade the firmware on the ESP module to the "
                     "latest version.");

    if (strlen(_ssid) == 0) {
      _status = WS_SSID_INVALID;
    } else {
      // disconnect from possible previous connection
      _disconnect();

// no clear recommendation, all three defined for both boards, future-proofing
#if defined(NINA_RESETN)
#define NINA_RESET_PIN NINA_RESETN
#elif defined(SPIWIFI_RESET)
#define NINA_RESET_PIN SPIWIFI_RESET
#elif defined(ESP32_RESETN)
#define NINA_RESET_PIN ESP32_RESETN
#endif

#if defined(NINA_RESET_PIN)
      // reset the esp32 if possible, better if we didn't do this first time
      if (NINA_RESET_PIN != -1) {
        WS_DEBUG("Resetting ESP32...");
        pinMode(NINA_RESET_PIN, OUTPUT);
        digitalWrite(NINA_RESET_PIN, LOW);
        delay(10);
        digitalWrite(NINA_RESET_PIN, HIGH);
        delay(10);
      }
      // wait for the ESP32 to boot
      delay(1000);
#endif

      WS_DEBUG_PRINT("Connecting to ");
      WS_DEBUG_PRINTLN(_ssid);
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

#endif // WIPPERSNAPPER_WIFININA_H