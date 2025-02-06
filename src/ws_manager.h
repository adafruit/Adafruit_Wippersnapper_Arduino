#ifndef WIPPERSNAPPER_MANAGER_H
#define WIPPERSNAPPER_MANAGER_H

//#include "adapters/wifi/ws_wifi_esp32.h"
//typedef ws_wifi_esp32 ws_adapter_wifi;
#include "adapters/wifi/ws_wifi_esp32_v2.h"
typedef ws_wifi_esp32_v2 ws_adapter_wifi_v2;

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
  //ws_adapter_wifi *ws_instance;      ///< Instance of Wippersnapper API v1
  ws_adapter_wifi_v2 *ws_instance_v2; ///< Instance of Wippersnapper API v2
private:
  int _api_version;
};
#endif // WIPPERSNAPPER_MANAGER_H
