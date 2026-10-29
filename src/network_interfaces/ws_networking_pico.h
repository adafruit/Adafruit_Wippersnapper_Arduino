/*!
 * @file ws_networking_pico.h
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

#ifndef WS_NETWORKING_PICO_H
#define WS_NETWORKING_PICO_H

#ifdef ARDUINO_ARCH_RP2040

#define PICO_CONNECT_TIMEOUT_MS 20000   /*!< Connection timeout (in ms) */
#define PICO_CONNECT_RETRY_DELAY_MS 200 /*!< delay time between retries. */

#include "Wippersnapper.h"

#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "Arduino.h"
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#if defined(PICO_CYW43_SUPPORTED)
#include <pico/cyw43_arch.h>
#endif

class ws_networking_pico;
extern ws_networking_pico WS; //!< Global instance of Wippersnapper_WiFi
/****************************************************************************/
/*!
    @brief  Class for using the Raspberry Pi Pico network interface.
*/
/****************************************************************************/
class ws_networking_pico : public Wippersnapper {

public:
  /**************************************************************************/
  /*!
  @brief  Initializes the WipperSnapper class for RPi Pico.
  */
  /**************************************************************************/
  ws_networking_pico() : Wippersnapper() {
    _ssid = 0;
    _pass = 0;
  }

  /**************************************************************************/
  /*!
  @brief  Destructor
  */
  /**************************************************************************/
  ~ws_networking_pico() {
    if (_mqtt_client_secure)
      delete _mqtt_client_secure;
    if (_mqtt_client_secure)
      delete _mqtt_client_secure;
  }

  /********************************************************/
  /*!
  @brief  Sets the WiFi client's ssid and password.
  @param  ssid
            WiFi network's SSID.
  @param  ssidPassword
            WiFi network's password.
  */
  /********************************************************/
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

  /**********************************************************/
  /*!
  @brief  Sets the WiFi client's ssid and password.
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
    // Set WiFi to station mode and disconnect from an AP if it was previously
    // connected
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    set_wifi_country();

    // Perform a network scan, CYW43 reports hidden networks with no SSID
    int n = WiFi.scanNetworks();
    if (n == 0) {
      // a hidden network may still answer the directed scan below
      WS_DEBUG_PRINTLN("No visible WiFi networks found!");
    }

    bool foundNetwork = false;

    WS_DEBUG_PRINTLN("WipperSnapper found these WiFi networks:");
    for (uint8_t i = 0; i < n; i++) {
      const char *scanSsid = WiFi.SSID(i);
      // hidden networks don't broadcast their SSID
      bool hidden = (scanSsid == NULL || scanSsid[0] == '\0');
      if (!hidden && !foundNetwork && strcmp(scanSsid, _ssid) == 0) {
        foundNetwork = true;
      } else if (!hidden && !foundNetwork && WS._isWiFiMulti) {
        // multi network mode
        for (int j = 0; j < WS_MAX_ALT_WIFI_NETWORKS; j++) {
          if (strlen(WS._multiNetworks[j].ssid) > 0 &&
              strcmp(WS._multiNetworks[j].ssid, scanSsid) == 0) {
            foundNetwork = true;
          }
        }
      }
      if (hidden) {
        WS_DEBUG_PRINT("<hidden>");
      } else {
        WS_DEBUG_PRINT(scanSsid);
      }
      WS_DEBUG_PRINT(" (");
      uint8_t BSSID[WL_MAC_ADDR_LENGTH];
      WiFi.BSSID(i, BSSID);
      for (int m = 0; m < WL_MAC_ADDR_LENGTH; m++) {
        if (m != 0)
          WS_DEBUG_PRINT(":");
        WS_DEBUG_PRINTHEX(BSSID[m]);
      }
      WS_DEBUG_PRINT(") ");
      WS_DEBUG_PRINT(WiFi.RSSI(i));
      WS_DEBUG_PRINT("dB (ch");
      WS_DEBUG_PRINT(WiFi.channel(i))
      WS_DEBUG_PRINTLN(")");
    }

    // Hidden networks only reveal their SSID in reply to a directed probe.
    // Not gated on seeing a hidden network above, CYW43 support is unverified.
    _hiddenSsid = NULL;
    _hiddenPass = NULL;
    if (!foundNetwork) {
      if (scan_hidden_ssid(_ssid)) {
        _hiddenSsid = _ssid;
        _hiddenPass = _pass;
      } else if (WS._isWiFiMulti) {
        for (int j = 0; j < WS_MAX_ALT_WIFI_NETWORKS; j++) {
          if (scan_hidden_ssid(WS._multiNetworks[j].ssid)) {
            _hiddenSsid = WS._multiNetworks[j].ssid;
            _hiddenPass = WS._multiNetworks[j].pass;
            break;
          }
        }
      }
      foundNetwork = (_hiddenSsid != NULL);
    }

    if (!foundNetwork) {
      WS_DEBUG_PRINTLN("ERROR: Your requested WiFi network was not found!");
    }
    return foundNetwork;
  }

  /***********************************************************/
  /*!
  @brief   Sets the WiFi regulatory country from secrets.json.
  @note    arduino-pico only takes a country at build time (WIFICC), so
           this sends the "country" iovar the CYW43 driver itself sends
           when the radio powers on, then reads it back. The core never
           powers the radio off, so it holds until reset. Unset keeps
           the build-time default, which is RAM-only.
  */
  /***********************************************************/
  void set_wifi_country() {
#if defined(PICO_CYW43_SUPPORTED)
    if (strlen(WS._config.network_country) != 2)
      return;
    // The iovar needs the radio on, as scanNetworks() would do
    if ((cyw43_state.itf_state & (1 << CYW43_ITF_STA)) == 0)
      cyw43_arch_enable_sta_mode();

    // Layout matches cyw43_ll_wifi_on(): abbrev, revision (-1 = any), code
    uint32_t country = CYW43_COUNTRY(WS._config.network_country[0],
                                     WS._config.network_country[1], 0);
    uint32_t rev = (uint32_t)-1;
    uint8_t buf[20] = "country";
    memcpy(buf + 8, &country, 4);
    memcpy(buf + 12, &rev, 4);
    memcpy(buf + 16, &country, 4);
    if (cyw43_ioctl(&cyw43_state, CYW43_IOCTL_SET_VAR, sizeof(buf), buf,
                    CYW43_ITF_STA) != 0) {
      WS_DEBUG_PRINT("ERROR: Unable to set WiFi country: ");
      WS_DEBUG_PRINTLNVAR(WS._config.network_country);
      return;
    }
    delay(50); // the driver waits the same after setting it

    // Read back: char abbrev[4], int32 rev, char ccode[4]
    uint8_t readback[20] = "country";
    if (cyw43_ioctl(&cyw43_state, CYW43_IOCTL_GET_VAR, sizeof(readback),
                    readback, CYW43_ITF_STA) == 0) {
      char ccode[3] = {(char)readback[8], (char)readback[9], '\0'};
      int32_t ccrev;
      memcpy(&ccrev, readback + 4, 4);
      WS_DEBUG_PRINT("WiFi country set to: ");
      WS_DEBUG_PRINTVAR(ccode);
      WS_DEBUG_PRINT(" rev ");
      WS_DEBUG_PRINTLNVAR(ccrev);
    }
#endif
  }

  /***********************************************************/
  /*!
  @brief   Performs a directed scan (probe request) for a hidden network.
  @note    arduino-pico's scanNetworks() can't take an SSID, so this
           calls the CYW43 driver directly.
  @param   ssid
           The SSID of the hidden network to scan for.
  @returns True if an access point responded to `ssid`, False otherwise.
  */
  /***********************************************************/
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
    WS.feedWDT();
    if (cyw43_wifi_scan(&cyw43_state, &scan_options, &_hiddenScan,
                        _hiddenScanCB) != 0) {
      return false;
    }
    uint32_t start = millis();
    while (cyw43_wifi_scan_active(&cyw43_state) && millis() - start < 10000) {
      // RP2040 WDT maxes out at ~8.3s, shorter than this wait
      WS.feedWDT();
      delay(10);
    }
    WS.feedWDT();
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

  /********************************************************/
  /*!
  @brief  Sets the RPi Pico's unique client identifier
  @note   On RPi Pico, the UID is the MAC address.
  */
  /********************************************************/
  void getMacAddr() {
    uint8_t mac[6] = {0};
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
  @brief  Initializes the MQTT client
  @param  clientID
          MQTT client identifier
  */
  /********************************************************/
  void setupMQTTClient(const char *clientID) {
    if (strcmp(WS._config.aio_url, "io.adafruit.com") == 0 ||
        strcmp(WS._config.aio_url, "io.adafruit.us") == 0) {
      _mqtt_client_secure = new WiFiClientSecure();
      _mqtt_client_secure->setCACert(
          strcmp(WS._config.aio_url, "io.adafruit.com") == 0
              ? _aio_root_ca_prod
              : _aio_root_ca_staging);
      WS._mqtt = new Adafruit_MQTT_Client(
          _mqtt_client_secure, WS._config.aio_url, WS._config.io_port, clientID,
          WS._config.aio_user, WS._config.aio_key);
    } else {
      _mqtt_client_insecure = new WiFiClient();
      WS._mqtt = new Adafruit_MQTT_Client(
          _mqtt_client_insecure, WS._config.aio_url, WS._config.io_port,
          clientID, WS._config.aio_user, WS._config.aio_key);
    }
  }

  /********************************************************/
  /*!
  @brief  Returns the network status of an RPi Pico.
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
  @return Pico
  */
  /*******************************************************************/
  const char *connectionType() { return "Pico"; }

protected:
  const char *_ssid; ///< WiFi SSID
  const char *_pass; ///< WiFi password
  WiFiClient
      *_mqtt_client_insecure; ///< Pointer to an insecure WiFi client object
  WiFiClientSecure
      *_mqtt_client_secure;       ///< Pointer to a secure WiFi client object
  WiFiMulti _wifiMulti;           ///< WiFiMulti object for multi-network mode
  const char *_hiddenSsid = NULL; ///< Hidden SSID found by a directed scan
  const char *_hiddenPass = NULL; ///< Password for `_hiddenSsid`

#if defined(PICO_CYW43_SUPPORTED)
  /** Directed scan state, written from the CYW43 scan callback */
  struct {
    const char *ssid;          ///< SSID being scanned for
    size_t ssid_len;           ///< Length of `ssid`
    volatile bool found;       ///< True once an AP answered to `ssid`
    volatile uint16_t channel; ///< Channel of the AP that answered
  } _hiddenScan = {};

  /**************************************************************************/
  /*!
  @brief  CYW43 scan callback, matches results against `_hiddenScan.ssid`.
  @param  env
          Pointer to `_hiddenScan`.
  @param  result
          A single scan result from the CYW43 driver.
  @return 0, to keep scanning.
  */
  /**************************************************************************/
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
      "MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw\n"
      "TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n"
      "cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4\n"
      "WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu\n"
      "ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY\n"
      "MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc\n"
      "h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+\n"
      "0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U\n"
      "A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW\n"
      "T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH\n"
      "B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC\n"
      "B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv\n"
      "KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn\n"
      "OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn\n"
      "jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw\n"
      "qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI\n"
      "rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV\n"
      "HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq\n"
      "hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL\n"
      "ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ\n"
      "3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK\n"
      "NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5\n"
      "ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur\n"
      "TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC\n"
      "jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc\n"
      "oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq\n"
      "4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA\n"
      "mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d\n"
      "emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=\n"
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

  /**************************************************************************/
  /*!
  @brief  Establishes a connection with the wireless network.
  */
  /**************************************************************************/
  void _connect() {

    if (WiFi.status() == WL_CONNECTED)
      return;

    WiFi.mode(WIFI_STA);
    WS.feedWDT();
    WiFi.setTimeout(20000);
    WS.feedWDT();

    if (strlen(_ssid) == 0) {
      _status = WS_SSID_INVALID;
    } else {
      _disconnect();
      delay(5000);
      WS.feedWDT();
      if (WS._isWiFiMulti && _hiddenSsid == NULL) {
        // multi network mode
        _wifiMulti.clearAPList();
        // add default network
        _wifiMulti.addAP(_ssid, _pass);
        // add array of alternative networks
        for (int i = 0; i < WS_MAX_ALT_WIFI_NETWORKS; i++) {
          // skip empty slots, they'd match hidden networks' empty SSIDs
          if (strlen(WS._multiNetworks[i].ssid) > 0) {
            _wifiMulti.addAP(WS._multiNetworks[i].ssid,
                             WS._multiNetworks[i].pass);
          }
        }
        WS.feedWDT();
        if (_wifiMulti.run(20000) == WL_CONNECTED) {
          WS.feedWDT();
          _status = WS_NET_CONNECTED;
          return;
        }
        WS.feedWDT();
      } else {
        if (_hiddenSsid != NULL) {
          // hidden network found by check_valid_ssid(), WiFiMulti can't see it
          WiFi.begin(_hiddenSsid, _hiddenPass);
        } else {
          WiFi.begin(_ssid, _pass);
        }

        // Use the macro to retry the status check until connected / timed out
        int lastResult;
        RETRY_FUNCTION_UNTIL_TIMEOUT(
            []() -> int { return WiFi.status(); }, // Function call each cycle
            int,                                   // return type
            lastResult, // return variable (unused here)
            [](int status) { return status == WL_CONNECTED; }, // check
            PICO_CONNECT_TIMEOUT_MS,      // timeout interval (ms)
            PICO_CONNECT_RETRY_DELAY_MS); // interval between retries

        if (lastResult == WL_CONNECTED) {
          _status = WS_NET_CONNECTED;
          // wait 2seconds for connection to stabilize
          WS_DELAY_WITH_WDT(2000);
          return;
        }
      }
      _status = WS_NET_DISCONNECTED;
    }
  }

  /**************************************************************************/
  /*!
      @brief  Disconnects from the wireless network.
  */
  /**************************************************************************/
  void _disconnect() {
    WS.feedWDT();
    WiFi.disconnect();
    delay(5000);
    WS.feedWDT();
  }
};

#endif // ARDUINO_ARCH_RP2040
#endif // WS_NETWORKING_PICO_H