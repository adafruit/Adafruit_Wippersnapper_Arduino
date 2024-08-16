#ifndef WIPPERSNAPPER_MANAGER_H
#define WIPPERSNAPPER_MANAGER_H

// #include "Wippersnapper.h"
#include "network_interfaces/Wippersnapper_ESP32.h"
typedef Wippersnapper_ESP32 Wippersnapper_WiFi;
// Wippersnapper_WiFi wipper
#include "Wippersnapper_V2.h"


class Wippersnapper_Manager {
public:
  Wippersnapper_Manager();
  ~Wippersnapper_Manager();

  // API version check
  void checkAPIVersion();
  int getAPIVersion() { return _api_version; }
  void provision();
  void connect();

protected:
  // Wippersnapper *ws_instance;
  Wippersnapper_WiFi *ws_instance;
  Wippersnapper_V2 *ws_instance_v2;

private:
  int _api_version;
};
#endif                   // WIPPERSNAPPER_MANAGER_H
