#ifndef WIPPERSNAPPER_MANAGER_H
#define WIPPERSNAPPER_MANAGER_H

#include "Wippersnapper.h"
#include "Wippersnapper_V2.h"

class Wippersnapper;
class Wippersnapper_Manager {
public:
  Wippersnapper_Manager();
  ~Wippersnapper_Manager();

  // TODO: Do we need this within the manager?
  // void provision();
  // TODO: Implement

protected:
  Wippersnapper
      *ws_instance; // Pointer to either Wippersnapper or Wippersnapper_V2

private:
  int getProvisionResult();
};
extern Wippersnapper WS; ///< Global member variable for callbacks
#endif                   // WIPPERSNAPPER_MANAGER_H
