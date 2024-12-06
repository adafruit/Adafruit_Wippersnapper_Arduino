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

  /****************************************************************/
  /*!
  @brief  a structure to hold network information for sorting
  */
  /****************************************************************/
  struct WiFiNetwork {
    char ssid[33]; /*!< SSID (Max 32 characters + null terminator */
    int rssi;      /*!< Received Signal Strength Indicator */
  };

  /*******************************************************************/
  /*!
  @brief  Comparison function to sort by RSSI in descending order
  @param  a
          WiFiNetwork object
  @param  b
          WiFiNetwork object
  @returns True if a.rssi > b.rssi
  */
  /*******************************************************************/
  bool static compareByRSSI(const WiFiNetwork &a, const WiFiNetwork &b) {
    return a.rssi > b.rssi;
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
    int8_t n = WiFi.scanNetworks();
    if (n == 0) {
      WS_DEBUG_PRINTLN("ERROR: No WiFi networks found!");
      return false;
    }

    WiFiNetwork networks[WS_MAX_SORTED_NETWORKS];
    uint8_t numSavedNetworks = 0;

    // Store the scanned networks in the vector
    for (int i = 0; i < n; ++i) {
      if (i < WS_MAX_SORTED_NETWORKS) {
        strncpy(networks[i].ssid, WiFi.SSID(i), sizeof(networks[i].ssid));
        networks[i].ssid[sizeof(networks[i].ssid) - 1] = '\0';
        networks[i].rssi = WiFi.RSSI(i);
        numSavedNetworks++;
      } else {
        WS_DEBUG_PRINT("ERROR: Too many networks found! (>");
        WS_DEBUG_PRINT(WS_MAX_SORTED_NETWORKS);
        WS_DEBUG_PRINT(") Ignoring ");
        WS_DEBUG_PRINT(WiFi.SSID(i));
        WS_DEBUG_PRINT("(");
        WS_DEBUG_PRINT(WiFi.RSSI(i));
        WS_DEBUG_PRINTLN(")");
      }
    }

    /// Sort the networks by RSSI in descending order
    std::sort(networks, networks + numSavedNetworks, compareByRSSI);

    // Was the network within secrets.json found?
    for (int i = 0; i < numSavedNetworks; ++i) {
      if (strcmp(_ssid, networks[i].ssid) == 0) {
        WS_DEBUG_PRINT("SSID (");
        WS_DEBUG_PRINT(_ssid);
        WS_DEBUG_PRINT(") found! RSSI: ");
        WS_DEBUG_PRINTLN(networks[i].rssi);
        return true;
      }
    }

    // User-set network not found, print scan results to serial console
    WS_DEBUG_PRINTLN("ERROR: Your requested WiFi network was not found!");
    WS_DEBUG_PRINT("WipperSnapper found these WiFi networks");
    if (n > WS_MAX_SORTED_NETWORKS) {
      WS_DEBUG_PRINT(" (only first ");
      WS_DEBUG_PRINT(WS_MAX_SORTED_NETWORKS);
      WS_DEBUG_PRINTLN(" used):");
    } else {
      WS_DEBUG_PRINTLN(":");
    }
    for (int i = 0; i < n; ++i) {
      WS_DEBUG_PRINT(networks[i].ssid);
      WS_DEBUG_PRINT(" ");
      WS_DEBUG_PRINT(networks[i].rssi);
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
  @return int32_t RSSI value
  */
  /********************************************************/
  int32_t getRSSI() { return WiFi.RSSI(); }

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