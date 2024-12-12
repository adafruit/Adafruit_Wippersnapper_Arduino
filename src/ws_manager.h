#ifndef WIPPERSNAPPER_MANAGER_H
#define WIPPERSNAPPER_MANAGER_H

#include "network_interfaces/ws_nonet_pico.h"
typedef ws_nonet_pico Wippersnapper_WiFi;
#include "network_interfaces/ws_nonet_pico_v2.h"
typedef ws_nonet_pico_v2 Wippersnapper_WiFiV2;

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
  Wippersnapper_WiFi *ws_instance;      ///< Instance of Wippersnapper API v1
  Wippersnapper_WiFiV2 *ws_instance_v2; ///< Instance of Wippersnapper API v2
private:
  int _api_version;
};
#endif // WIPPERSNAPPER_MANAGER_H
