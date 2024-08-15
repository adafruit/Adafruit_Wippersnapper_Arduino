#ifndef WIPPERSNAPPER_MANAGER_H
#define WIPPERSNAPPER_MANAGER_H

#include "Wippersnapper.h"
#include "Wippersnapper_V2.h"

class Wippersnapper;
class Wippersnapper_Manager {
public:
  Wippersnapper_Manager();
  ~Wippersnapper_Manager();

  // API version check
  void checkAPIVersion();

  // TODO: Do we need this within the manager?
  // void provision();
  // TODO: Implement

protected:
  Wippersnapper *ws_instance;
  Wippersnapper_V2 *ws_instance_v2;

private:
  bool _api_version; // True if API version 2, False otherwise
};
#endif                   // WIPPERSNAPPER_MANAGER_H
