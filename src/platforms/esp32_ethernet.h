/*!
 * @file esp32_ethernet.h
 *
 * This is a driver for using an ESP32's onboard Ethernet (EMAC + PHY),
 * alongside its WiFi, with Adafruit IO Wippersnapper.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2026 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef ESP32_ETHERNET_H
#define ESP32_ETHERNET_H

#if defined(ARDUINO_ARCH_ESP32) && defined(USE_ETHERNET)
#include "esp32_wifi.h"
#include <ETH.h>

/// How long one _connect() call waits for an Ethernet IP before falling
/// back to WiFi (or returning, when no WiFi credentials are stored)
#define WS_ETH_WAIT_MS (5 * ONE_SECOND_IN_MS)

/*!
    @brief  Class for using an ESP32's onboard Ethernet and WiFi together.
            Ethernet is used whenever its link is up, and WiFi is used too
            when secrets.json has WiFi credentials. Missing WiFi credentials
            are not an error on these boards.
*/
class esp32_ethernet : public esp32_wifi {

public:
  using esp32_wifi::esp32_wifi;

  /*!
  @brief  Returns the network status of the Ethernet and WiFi interfaces.
  @return WS_NET_CONNECTED if either interface has an IP address.
  */
  ws_status_t networkStatus() {
    if (ethConnected())
      return WS_NET_CONNECTED;
    if (!hasWiFiCredentials())
      return WS_NET_DISCONNECTED;
    return esp32_wifi::networkStatus();
  }

  /*!
  @brief  Gets the WiFi RSSI value.
  @return The RSSI in dBm, or 0 when WiFi isn't connected (e.g. Ethernet
          only, where WiFi is never started).
  */
  int32_t getRSSI() {
    // TODO: report Ethernet/WiFi latency in telemetry too
    if (WiFi.status() != WL_CONNECTED)
      return 0;
    return WiFi.RSSI();
  }

  /*!
  @brief  Returns the type of network connection used by Wippersnapper
  @return ESP32_ETH
  */
  const char *connectionType() { return "ESP32_ETH"; }

  /*!
  @brief  Checks if WiFi credentials are stored on the device.
  @return True if a WiFi SSID is configured, False otherwise.
  */
  bool hasWiFiCredentials() {
    if (_ssid != NULL && strlen(_ssid) > 0)
      return true;
    if (!Ws->_isWiFiMultiV2)
      return false;
    for (int i = 0; i < WS_MAX_ALT_WIFI_NETWORKS; i++) {
      if (strlen(Ws->_multiNetworksV2[i].ssid) > 0)
        return true;
    }
    return false;
  }

protected:
  bool _ethStarted = false; ///< True once ETH.begin() has been called
  bool _noWiFiLogged =
      false; ///< True once the "no WiFi credentials" message was printed
  bool _wifiBackground =
      false; ///< True while WiFi auto-reconnects behind an Ethernet link

  /*!
  @brief  Prefers Ethernet as the default route whenever it has an IP.
  @note   ESP-IDF gives WiFi STA a higher route priority than Ethernet, so
          without this, traffic leaves over WiFi whenever both are up.
  @param  event
          The network event.
  @param  info
          The network event's information.
  */
  static void onNetworkEvent(arduino_event_id_t event,
                             arduino_event_info_t info) {
    (void)info;
    switch (event) {
    case ARDUINO_EVENT_ETH_START:
      WS_DEBUG_PRINTLN("[eth] Started");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      WS_DEBUG_PRINT("[eth] Link up, ");
      WS_DEBUG_PRINTVAR(ETH.linkSpeed());
      WS_DEBUG_PRINTLNVAR(ETH.fullDuplex() ? "Mbps full duplex"
                                           : "Mbps half duplex");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      WS_DEBUG_PRINT("[eth] Got IP: ");
      WS_DEBUG_PRINTLNVAR(ETH.localIP());
      ETH.setDefault();
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      // keep Ethernet as the default route while it is up
      if (ethConnected())
        ETH.setDefault();
      break;
    case ARDUINO_EVENT_ETH_LOST_IP:
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      WS_DEBUG_PRINTLNVAR(event == ARDUINO_EVENT_ETH_LOST_IP
                              ? "[eth] Lost IP"
                              : "[eth] Link down");
      if (WiFi.STA.hasIP())
        WiFi.STA.setDefault();
      break;
    case ARDUINO_EVENT_ETH_STOP:
      WS_DEBUG_PRINTLN("[eth] Stopped");
      break;
    default:
      break;
    }
  }

  /*!
  @brief  Checks if Ethernet can carry traffic.
  @note   Checks the link as well as the IP: the IP can outlive a pulled
          cable until the lost-IP timer runs out.
  @return True if the Ethernet link is up with an IP address.
  */
  static bool ethConnected() { return ETH.linkUp() && ETH.hasIP(); }

  /*!
  @brief  Starts the Ethernet interface, once.
  */
  void startEthernet() {
    if (_ethStarted)
      return;
    _ethStarted = true;
    Network.onEvent(onNetworkEvent);
    // PHY type and pins come from the board's pins_arduino.h variant
    if (!ETH.begin()) {
      WS_DEBUG_PRINTLN("ERROR: Unable to start Ethernet!");
    }
  }

  /*!
  @brief  Waits for the Ethernet interface to get an IP address.
  @param  timeout_ms
          Maximum time to wait, in milliseconds.
  @return True if Ethernet has an IP address, False otherwise.
  */
  bool waitForEthernet(uint32_t timeout_ms) {
    // covers link negotiation after ETH.begin() or a replug, then DHCP
    uint32_t start = millis();
    while (!ethConnected() && millis() - start < timeout_ms) {
      Ws->_wdt->feed();
      delay(100);
    }
    return ethConnected();
  }

  /*!
  @brief  Establishes a network connection over Ethernet and/or WiFi.
  @note   Never halts: NetworkFSM keeps calling this until one interface
          has an IP address.
  */
  void _connect() {
    startEthernet();
    bool ethUp = waitForEthernet(WS_ETH_WAIT_MS);

    if (!hasWiFiCredentials()) {
      if (!_noWiFiLogged) {
        WS_DEBUG_PRINTLN("No WiFi credentials stored, using Ethernet only");
        _noWiFiLogged = true;
      }
      if (!ethUp)
        WS_DEBUG_PRINTLN("Waiting for an Ethernet link...");
      return;
    }

    if (WiFi.status() == WL_CONNECTED)
      return;

    if (ethUp) {
      // Ethernet carries the traffic, bring WiFi up in the background and
      // let the WiFi driver reconnect it, so this never blocks on WiFi.
      // TODO: this joins one network only. Scanning alternative_networks
      // for the best one (and hidden networks) is done by the WiFi-only
      // path below, not by this background connection.
      if (!_wifiBackground) {
        const char *ssid = _ssid;
        const char *pass = _pass;
        if (ssid == NULL || strlen(ssid) == 0) {
          for (int i = 0; i < WS_MAX_ALT_WIFI_NETWORKS; i++) {
            if (strlen(Ws->_multiNetworksV2[i].ssid) > 0) {
              ssid = Ws->_multiNetworksV2[i].ssid;
              pass = Ws->_multiNetworksV2[i].pass;
              break;
            }
          }
        }
        WS_DEBUG_PRINT("Ethernet connected, joining WiFi in background: ");
        WS_DEBUG_PRINTLNVAR(ssid);
        _wifiBackground = true;
        WiFi.mode(WIFI_STA);
        WiFi.setAutoReconnect(true);
        WiFi.begin(ssid, pass);
      }
      return;
    }

    // No Ethernet: connect over WiFi, like the WiFi-only ESP32 boards.
    // esp32_wifi::_connect() restarts WiFi without auto-reconnect, so a
    // later Ethernet link starts the background connection again.
    _wifiBackground = false;

    WS_DEBUG_PRINTLN("No Ethernet link, connecting to WiFi...");
    if (check_valid_ssid() == WS_SSID_SCAN_SSID_FOUND)
      esp32_wifi::_connect();
  }
};

#endif // ARDUINO_ARCH_ESP32 && USE_ETHERNET
#endif // ESP32_ETHERNET_H
