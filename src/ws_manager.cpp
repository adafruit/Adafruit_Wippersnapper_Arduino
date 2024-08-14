#include "ws_manager.h"

Wippersnapper_Manager::Wippersnapper_Manager() : ws_instance(nullptr) {}

Wippersnapper_Manager::~Wippersnapper_Manager() {
  if (ws_instance) {
    delete ws_instance;
  }
}