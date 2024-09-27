#ifndef WIPPERSNAPPER_MANAGER_H
#define WIPPERSNAPPER_MANAGER_H

#include "network_interfaces/Wippersnapper_ESP32.h"
typedef Wippersnapper_ESP32 Wippersnapper_WiFi;
#include "network_interfaces/Wippersnapper_ESP32_V2.h"
typedef Wippersnapper_ESP32V2 Wippersnapper_WiFiV2;

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
  int getAPIVersion() { return _api_version; }

  // High-level functions (called from demo sketch ino)
  void provision();
  void connect();
  void run();

protected:
  Wippersnapper_WiFi *ws_instance; ///< Instance of Wippersnapper API v1
  Wippersnapper_WiFiV2 *ws_instance_v2; ///< Instance of Wippersnapper API v2
private:
  int _api_version;
};
#endif // WIPPERSNAPPER_MANAGER_H
