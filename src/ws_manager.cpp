#include "ws_manager.h"

Wippersnapper_Manager::Wippersnapper_Manager() : ws_instance(nullptr) {}

Wippersnapper_Manager::~Wippersnapper_Manager() {
  if (ws_instance) {
    delete ws_instance;
  }
  if (ws_instance_v2) {
    delete ws_instance_v2;
  }
}

void Wippersnapper_Manager::checkAPIVersion() {
  // Check if pin D12 is high
  pinMode(0, INPUT_PULLUP);
  _api_version = digitalRead(0);
  if (_api_version) { // API version 2 if D12 is high
    ws_instance_v2 = new Wippersnapper_V2();
  } else { // API version 1 if D12 is low
    ws_instance = new Wippersnapper();
  }
}