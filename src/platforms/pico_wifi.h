/*!
 * @file pico_wifi.h
 *
 * This is a driver for using the Raspberry Pi Pico (RP2040)
 * network interface with Adafruit IO Wippersnapper.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2023 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef PICO_WIFI_H
#define PICO_WIFI_H

#if defined(ARDUINO_RASPBERRY_PI_PICO_W) ||                                    \
    defined(ARDUINO_RASPBERRY_PI_PICO_2W) || defined(PICO_CYW43_SUPPORTED)

#define PICO_CONNECT_TIMEOUT_MS 20000   /*!< Connection timeout (in ms) */
#define PICO_CONNECT_RETRY_DELAY_MS 200 /*!< delay time between retries. */

#include "wippersnapper.h"

#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "Arduino.h"
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#if defined(PICO_CYW43_SUPPORTED)
#include <pico/cyw43_arch.h>
#endif
extern wippersnapper *Ws;

/*!
    @brief  Class for using the Raspberry Pi Pico network interface.
*/
class pico_wifi : public wippersnapper {

public:
  /*!
  @brief  Initializes the WipperSnapper class for RPi Pico.
  */
  pico_wifi() : wippersnapper() {
    _ssid = 0;
    _pass = 0;
  }

  /*!
  @brief  Destructor
  */
  ~pico_wifi() {
    disconnect();
    if (_mqtt_client_secure)
      delete _mqtt_client_secure;
    if (_mqtt_client_secure)
      delete _mqtt_client_secure;
  }

  /*!
  @brief  Sets the WiFi client's ssid and password.
  @param  ssid
            WiFi network's SSID.
  @param  ssidPassword
            WiFi network's password.
  */
  void set_ssid_pass(const char *ssid, const char *ssidPassword) {
    _ssid = ssid;

    // set the AP password
    // check if ssidPassword was "" in secrets.json
    if ((ssidPassword != NULL) && (strlen(ssidPassword) == 0)) {
      _pass = NULL; // Set as NULL for open networks
    } else {
      _pass = ssidPassword;
    }
  }

  /*!
  @brief  Sets the WiFi client's ssid and password.
  */
  void set_ssid_pass() {
    _ssid = Ws->_configV2.network.ssid;
    _pass = Ws->_configV2.network.pass;
  }

  /*!
  @brief  UAT DEBUG: logs the CYW43 connection state with a timestamp.
  @param  tag
          Phase being logged.
  */
  void logWiFiState(const char *tag) {
    int32_t rssi = 0;
    cyw43_wifi_get_rssi(&cyw43_state, &rssi);
    WS_PRINTER.printf(
        "[wifi %7lu ms] %-22s status=%d link=%d tcpip=%d join=0x%04lx "
        "rssi=%ld ip=%s\n",
        (unsigned long)millis(), tag, (int)WiFi.status(),
        cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA),
        cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA),
        (unsigned long)cyw43_state.wifi_join_state, (long)rssi,
        WiFi.localIP().toString().c_str());
  }

  /*!
  @brief   Performs a scan of local WiFi networks.
  @returns ws_ssid_scan_t result of the SSID scan.
  */
  ws_ssid_scan_t check_valid_ssid() {
    // Set WiFi to station mode and disconnect from an AP if it was previously
    // connected
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    _hiddenIdx = -1;

    // Perform a network scan, CYW43 reports hidden networks with no SSID
    logWiFiState("scan start");
    int n = WiFi.scanNetworks();
    logWiFiState("scan done");
    if (n <= 0) {
      // a hidden network may still answer the directed scan below
      WS_DEBUG_PRINTLN("No visible WiFi networks found!");
    }

    // Was the network within secrets.json found?
    for (int i = 0; i < n; ++i) {
      const char *scanSsid = WiFi.SSID(i);
      if (scanSsid == NULL || scanSsid[0] == '\0') {
        // hidden networks don't broadcast their SSID
        continue;
      }
      if (strcmp(_ssid, scanSsid) == 0) {
        WS_DEBUG_PRINT("SSID (");
        WS_DEBUG_PRINTVAR(_ssid);
        WS_DEBUG_PRINT(") found! RSSI: ");
        WS_DEBUG_PRINTLNVAR(WiFi.RSSI(i));
        return WS_SSID_SCAN_SSID_FOUND;
      }
      if (Ws->_isWiFiMultiV2) {
        // multi network mode
        for (int j = 0; j < WS_MAX_ALT_WIFI_NETWORKS; j++) {
          if (strlen(Ws->_multiNetworksV2[j].ssid) > 0 &&
              strcmp(Ws->_multiNetworksV2[j].ssid, scanSsid) == 0) {
            WS_DEBUG_PRINT("SSID (");
            WS_DEBUG_PRINTVAR(Ws->_multiNetworksV2[j].ssid);
            WS_DEBUG_PRINT(") found! RSSI: ");
            WS_DEBUG_PRINTLNVAR(WiFi.RSSI(i));
            return WS_SSID_SCAN_SSID_FOUND;
          }
        }
      }
    }

    // Print scan results to serial console
    WS_DEBUG_PRINTLN("WipperSnapper found these WiFi networks: ");
    for (int i = 0; i < n; ++i) {
      const char *scanSsid = WiFi.SSID(i);
      if (scanSsid == NULL || scanSsid[0] == '\0') {
        WS_DEBUG_PRINT("<hidden>");
      } else {
        WS_DEBUG_PRINTVAR(scanSsid);
      }
      WS_DEBUG_PRINT(" ");
      WS_DEBUG_PRINTVAR(WiFi.RSSI(i));
      WS_DEBUG_PRINTLN("dB");
    }

    // Hidden networks only reveal their SSID in reply to a directed probe.
    // Not gated on seeing a hidden network above, CYW43 support is unverified.
    if (scan_hidden_ssid(_ssid)) {
      _hiddenIdx = 0;
    } else if (Ws->_isWiFiMultiV2) {
      for (int j = 0; j < WS_MAX_ALT_WIFI_NETWORKS; j++) {
        if (scan_hidden_ssid(Ws->_multiNetworksV2[j].ssid)) {
          _hiddenIdx = j + 1;
          break;
        }
      }
    }
    if (_hiddenIdx >= 0)
      return WS_SSID_SCAN_SSID_FOUND;

    if (n <= 0) {
      WS_DEBUG_PRINTLN("ERROR: No WiFi networks found!");
      return WS_SSID_SCAN_NO_NETWORKS;
    }
    // User-set network not found
    WS_DEBUG_PRINTLN("ERROR: Your requested WiFi network was not found!");
    return WS_SSID_SCAN_SSID_NOT_FOUND;
  }

  /*!
  @brief   Performs a directed scan (probe request) for a hidden network.
  @note    arduino-pico's scanNetworks() can't take an SSID, so this
           calls the CYW43 driver directly.
  @param   ssid
           The SSID of the hidden network to scan for.
  @returns True if an access point responded to `ssid`, False otherwise.
  */
  bool scan_hidden_ssid(const char *ssid) {
#if defined(PICO_CYW43_SUPPORTED)
    if (ssid == NULL || strlen(ssid) == 0)
      return false;

    WS_DEBUG_PRINT("Scanning for hidden WiFi network: ");
    WS_DEBUG_PRINTLNVAR(ssid);
    cyw43_wifi_scan_options_t scan_options;
    memset(&scan_options, 0, sizeof(scan_options));
    scan_options.ssid_len = strnlen(ssid, sizeof(scan_options.ssid));
    memcpy(scan_options.ssid, ssid, scan_options.ssid_len);
    _hiddenScan.ssid = ssid;
    _hiddenScan.ssid_len = scan_options.ssid_len;
    _hiddenScan.found = false;
    Ws->_wdt->feed();
    if (cyw43_wifi_scan(&cyw43_state, &scan_options, &_hiddenScan,
                        _hiddenScanCB) != 0) {
      return false;
    }
    uint32_t start = millis();
    while (cyw43_wifi_scan_active(&cyw43_state) &&
           millis() - start < TEN_SECONDS_IN_MS) {
      // RP2040/RP2350 hardware WDT maxes out at ~8.3s
      Ws->_wdt->feed();
      delay(10);
    }
    Ws->_wdt->feed();
    if (_hiddenScan.found) {
      WS_DEBUG_PRINT("Found hidden WiFi network on ch");
      WS_DEBUG_PRINTLNVAR(_hiddenScan.channel);
    }
    return _hiddenScan.found;
#else
    (void)ssid;
    return false;
#endif
  }

  /*!
  @brief  Sets the RPi Pico's unique client identifier
  @note   On RPi Pico, the UID is the MAC address.
  */
  void getMacAddr() {
    uint8_t mac[6] = {0};
    WiFi.macAddress(mac);
    memcpy(Ws->_macAddrV2, mac, sizeof(mac));
  }

  /*!
  @brief  Gets the current network RSSI value
  @return int32_t RSSI value
  */
  int32_t getRSSI() { return WiFi.RSSI(); }

  /*!
  @brief  Initializes the MQTT client
  @param  clientID
          MQTT client identifier
  */
  void setupMQTTClient(const char *clientID) {
    // TLS is only used on the secure port (8883); any other port is plain MQTT.
    if (Ws->_configV2.io_port == 8883) {
      _mqtt_client_secure = new WiFiClientSecure();
      if (strcmp(Ws->_configV2.aio_url, "io.adafruit.com") == 0) {
        // Production: validate against the pinned root CA
        _mqtt_client_secure->setCACert(_aio_root_ca_prod);
      } else {
        // Staging or a custom broker: TLS without certificate validation
        _mqtt_client_secure->setInsecure();
      }
      Ws->_mqttV2 = new Adafruit_MQTT_Client(
          _mqtt_client_secure, Ws->_configV2.aio_url, Ws->_configV2.io_port,
          clientID, Ws->_configV2.aio_user, Ws->_configV2.aio_key,
          WS_MQTT_BUFFER_SZ);
    } else {
      _mqtt_client_insecure = new WiFiClient();
      Ws->_mqttV2 = new Adafruit_MQTT_Client(
          _mqtt_client_insecure, Ws->_configV2.aio_url, Ws->_configV2.io_port,
          clientID, Ws->_configV2.aio_user, Ws->_configV2.aio_key,
          WS_MQTT_BUFFER_SZ);
    }
  }

  /*!
  @brief  Returns the network status of an RPi Pico.
  @return ws_status_t
  */
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

  /*!
  @brief  Returns the type of network connection used by Wippersnapper
  @return Pico
  */
  const char *connectionType() { return "PICO_CYW43"; }

protected:
  const char *_ssid; ///< WiFi SSID
  const char *_pass; ///< WiFi password
  WiFiClient
      *_mqtt_client_insecure; ///< Pointer to an insecure WiFi client object
  WiFiClientSecure
      *_mqtt_client_secure; ///< Pointer to a secure WiFi client object
  WiFiMulti _wifiMulti;     ///< WiFiMulti object for multi-network mode
  int8_t _hiddenIdx = -1;   ///< Hidden network found by a directed scan: -1
                            ///< none, 0 `_ssid`, n `_multiNetworksV2[n-1]`

#if defined(PICO_CYW43_SUPPORTED)
  /** Directed scan state, written from the CYW43 scan callback */
  struct {
    const char *ssid;          ///< SSID being scanned for
    size_t ssid_len;           ///< Length of `ssid`
    volatile bool found;       ///< True once an AP answered to `ssid`
    volatile uint16_t channel; ///< Channel of the AP that answered
  } _hiddenScan = {};

  /*!
  @brief  CYW43 scan callback, matches results against `_hiddenScan.ssid`.
  @param  env
          Pointer to `_hiddenScan`.
  @param  result
          A single scan result from the CYW43 driver.
  @return 0, to keep scanning.
  */
  static int _hiddenScanCB(void *env, const cyw43_ev_scan_result_t *result) {
    auto *scan = (decltype(_hiddenScan) *)env;
    if (result && result->ssid_len == scan->ssid_len &&
        memcmp(result->ssid, scan->ssid, scan->ssid_len) == 0) {
      scan->found = true;
      scan->channel = result->channel;
    }
    return 0;
  }
#endif

  const char *_aio_root_ca_staging =
      "-----BEGIN CERTIFICATE-----\n"
      "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\n"
      "-----END CERTIFICATE-----\n"; ///< Root certificate for io.adafruit.us

  const char *_aio_root_ca_prod =
      "-----BEGIN CERTIFICATE-----\n"
      "MIIEjTCCA3WgAwIBAgIQDQd4KhM/xvmlcpbhMf/ReTANBgkqhkiG9w0BAQsFADBh\n"
      "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n"
      "d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\n"
      "MjAeFw0xNzExMDIxMjIzMzdaFw0yNzExMDIxMjIzMzdaMGAxCzAJBgNVBAYTAlVT\n"
      "MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n"
      "b20xHzAdBgNVBAMTFkdlb1RydXN0IFRMUyBSU0EgQ0EgRzEwggEiMA0GCSqGSIb3\n"
      "DQEBAQUAA4IBDwAwggEKAoIBAQC+F+jsvikKy/65LWEx/TMkCDIuWegh1Ngwvm4Q\n"
      "yISgP7oU5d79eoySG3vOhC3w/3jEMuipoH1fBtp7m0tTpsYbAhch4XA7rfuD6whU\n"
      "gajeErLVxoiWMPkC/DnUvbgi74BJmdBiuGHQSd7LwsuXpTEGG9fYXcbTVN5SATYq\n"
      "DfbexbYxTMwVJWoVb6lrBEgM3gBBqiiAiy800xu1Nq07JdCIQkBsNpFtZbIZhsDS\n"
      "fzlGWP4wEmBQ3O67c+ZXkFr2DcrXBEtHam80Gp2SNhou2U5U7UesDL/xgLK6/0d7\n"
      "6TnEVMSUVJkZ8VeZr+IUIlvoLrtjLbqugb0T3OYXW+CQU0kBAgMBAAGjggFAMIIB\n"
      "PDAdBgNVHQ4EFgQUlE/UXYvkpOKmgP792PkA76O+AlcwHwYDVR0jBBgwFoAUTiJU\n"
      "IBiV5uNu5g/6+rkS7QYXjzkwDgYDVR0PAQH/BAQDAgGGMB0GA1UdJQQWMBQGCCsG\n"
      "AQUFBwMBBggrBgEFBQcDAjASBgNVHRMBAf8ECDAGAQH/AgEAMDQGCCsGAQUFBwEB\n"
      "BCgwJjAkBggrBgEFBQcwAYYYaHR0cDovL29jc3AuZGlnaWNlcnQuY29tMEIGA1Ud\n"
      "HwQ7MDkwN6A1oDOGMWh0dHA6Ly9jcmwzLmRpZ2ljZXJ0LmNvbS9EaWdpQ2VydEds\n"
      "b2JhbFJvb3RHMi5jcmwwPQYDVR0gBDYwNDAyBgRVHSAAMCowKAYIKwYBBQUHAgEW\n"
      "HGh0dHBzOi8vd3d3LmRpZ2ljZXJ0LmNvbS9DUFMwDQYJKoZIhvcNAQELBQADggEB\n"
      "AIIcBDqC6cWpyGUSXAjjAcYwsK4iiGF7KweG97i1RJz1kwZhRoo6orU1JtBYnjzB\n"
      "c4+/sXmnHJk3mlPyL1xuIAt9sMeC7+vreRIF5wFBC0MCN5sbHwhNN1JzKbifNeP5\n"
      "ozpZdQFmkCo+neBiKR6HqIA+LMTMCMMuv2khGGuPHmtDze4GmEGZtYLyF8EQpa5Y\n"
      "jPuV6k2Cr/N3XxFpT3hRpt/3usU/Zb9wfKPtWpoznZ4/44c1p9rzFcZYrWkj3A+7\n"
      "TNBJE0GmP2fhXhP1D/XVfIW/h0yCJGEiV9Glm/uGOa3DXHlmbAcxSyCRraG+ZBkA\n"
      "7h4SeM6Y8l/7MBRpPCz6l8Y=\n"
      "-----END CERTIFICATE-----\n"; ///< Root certificate for io.adafruit.com

  /*!
  @brief  Establishes a connection with the wireless network.
  */
  void _connect() {

    if (WiFi.status() == WL_CONNECTED)
      return;
    logWiFiState("connect start");

    WiFi.mode(WIFI_STA);
    Ws->_wdt->feed();
    // UAT DEBUG: 7 s join window, so a failing attempt returns and is logged
    // before the RP2350's ~16.8 s hardware WDT (begin() waits up to 2x this)
    WiFi.setTimeout(7 * ONE_SECOND_IN_MS);
    Ws->_wdt->feed();

    if (strlen(_ssid) == 0) {
      _statusV2 = WS_SSID_INVALID;
    } else {
      _disconnect();
      logWiFiState("after disconnect");
      delay(ONE_SECOND_IN_MS);
      Ws->_wdt->feed();
      logWiFiState("after 1s delay");
      if (Ws->_isWiFiMultiV2 && _hiddenIdx < 0) {
        // UAT DEBUG: WiFiMulti::run() steps done by hand, so the join and the
        // DHCP lease can be timed separately (it blocks through both)
        logWiFiState("multi scan start");
        int n = WiFi.scanNetworks();
        Ws->_wdt->feed();
        logWiFiState("multi scan done");
        const char *bestSsid = nullptr;
        const char *bestPass = nullptr;
        int32_t bestRssi = INT32_MIN;
        for (int i = 0; i < n; i++) {
          const char *scanSsid = WiFi.SSID(i);
          if (scanSsid == NULL || scanSsid[0] == '\0')
            continue;
          for (int k = 0; k <= WS_MAX_ALT_WIFI_NETWORKS; k++) {
            const char *ssid =
                (k == 0) ? _ssid : Ws->_multiNetworksV2[k - 1].ssid;
            if (strlen(ssid) > 0 && strcmp(ssid, scanSsid) == 0 &&
                WiFi.RSSI(i) > bestRssi) {
              bestSsid = ssid;
              bestPass = (k == 0) ? _pass : Ws->_multiNetworksV2[k - 1].pass;
              bestRssi = WiFi.RSSI(i);
            }
          }
        }
        if (bestSsid != nullptr) {
          WS_PRINTER.printf("[wifi %7lu ms] joining strongest, rssi=%ld\n",
                            (unsigned long)millis(), (long)bestRssi);
          WiFi.beginNoBlock(bestSsid, bestPass);
          Ws->_wdt->feed();
          logWiFiState("join returned");
          // Poll up to 30 s, logging every change, to see when the IP lands
          int lastStatus = -1;
          int lastLink = -99;
          uint32_t lastIp = 0;
          unsigned long t0 = millis();
          while (millis() - t0 < 30 * ONE_SECOND_IN_MS) {
            int st = WiFi.status();
            int link = cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA);
            uint32_t ip = (uint32_t)WiFi.localIP();
            if (st != lastStatus || link != lastLink || ip != lastIp) {
              lastStatus = st;
              lastLink = link;
              lastIp = ip;
              logWiFiState(ip != 0 ? "IP ARRIVED" : "poll change");
            }
            if (st == WL_CONNECTED)
              break;
            WS_DELAY_WITH_WDT(100);
          }
          logWiFiState(lastStatus == WL_CONNECTED ? "multi connected"
                                                  : "multi gave up");
          if (lastStatus == WL_CONNECTED) {
            Ws->_wdt->feed();
            _statusV2 = WS_NET_CONNECTED;
            return;
          }
        } else {
          logWiFiState("no configured network in scan");
        }
        Ws->_wdt->feed();
      } else {
        if (_hiddenIdx > 0) {
          // hidden alternative network found by check_valid_ssid(), WiFiMulti
          // can't see it
          WiFi.begin(Ws->_multiNetworksV2[_hiddenIdx - 1].ssid,
                     Ws->_multiNetworksV2[_hiddenIdx - 1].pass);
        } else {
          // single network mode, or the hidden default network
          logWiFiState("begin");
          WiFi.begin(_ssid, _pass);
        }
        logWiFiState("begin returned");
        Ws->_wdt->feed();

        // UAT DEBUG: same as RETRY_FUNCTION_UNTIL_TIMEOUT, logging changes
        int lastResult = WiFi.status();
        uint32_t lastJoin = cyw43_state.wifi_join_state;
        int lastTcpip = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
        unsigned long startTime = millis();
        while (millis() - startTime < PICO_CONNECT_TIMEOUT_MS) {
          lastResult = WiFi.status();
          int tcpip = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
          if (cyw43_state.wifi_join_state != lastJoin || tcpip != lastTcpip) {
            lastJoin = cyw43_state.wifi_join_state;
            lastTcpip = tcpip;
            logWiFiState("poll change");
          }
          if (lastResult == WL_CONNECTED)
            break;
          WS_DELAY_WITH_WDT(PICO_CONNECT_RETRY_DELAY_MS);
        }
        logWiFiState(lastResult == WL_CONNECTED ? "connected" : "poll timeout");

        if (lastResult == WL_CONNECTED) {
          _statusV2 = WS_NET_CONNECTED;
          // wait 2seconds for connection to stabilize
          WS_DELAY_WITH_WDT(2 * ONE_SECOND_IN_MS);
          return;
        }
      }
      _statusV2 = WS_NET_DISCONNECTED;
    }
  }

  /*!
      @brief  Disconnects from the wireless network.
      @param  wifi_off  Unused on this platform.
  */
  void _disconnect(bool wifi_off = true) {
    (void)wifi_off;
    Ws->_wdt->feed();
    logWiFiState("disconnect");
    WiFi.disconnect();
    delay(ONE_SECOND_IN_MS);
    Ws->_wdt->feed();
  }
};

#endif // RASPBERRY_PI_PICO_W
#endif // PICO_WIFI_H