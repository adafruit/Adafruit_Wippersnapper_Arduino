#ifndef WIPPERSNAPPER_MANAGER_H
#define WIPPERSNAPPER_MANAGER_H

// For compiling with ESP32
#include "adapters/wifi/ws_wifi_esp32.h"
typedef ws_wifi_esp32 ws_adapter_wifi;
#include "adapters/wifi/ws_wifi_esp32_v2.h"
typedef ws_wifi_esp32_v2 ws_adapter_wifi_v2;

// Uncomment the following lines to compile for Pico
// NOTE: When compiling for pico, you must also modify L39-L40 to reflect the
// following lines #include "adapters/offline/ws_offline_pico.h" typedef
// ws_offline_pico ws_adapter_offline; #include
// "adapters/offline/ws_offline_pico_v2.h" typedef ws_offline_pico_v2
// ws_adapter_offline_v2; typedef ws_offline_pico ws_adapter_wifi; typedef
// ws_offline_pico_v2 ws_adapter_wifi_v2;

/****************************************************************************/
/*!
    @brief  Helper class to manage the WipperSnapper API versions dynamically
            at runtime.
*/
/****************************************************************************/
class Wippersnapper_Manager {
public:
  Wippersnapper_Manager();
  ~Wippersnapper_Manager();

  // API version checks
  void checkAPIVersion(int pinNum);
  int getAPIVersion();

  // High-level functions (called from demo sketch ino)
  void provision();
  void connect();
  void run();

protected:
  ws_adapter_wifi *ws_instance;       ///< Instance of Wippersnapper API v1
  ws_adapter_wifi_v2 *ws_instance_v2; ///< Instance of Wippersnapper API v2
private:
  int _api_version;
};
#endif // WIPPERSNAPPER_MANAGER_H
