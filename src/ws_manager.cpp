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

void Wippersnapper_Manager::connect() {
  if (_api_version == 2) {
    //ws_instance_v2->connect();
    WS_DEBUG_PRINTLN("APIv2 connect");
  } else if (_api_version == 1) {
    WS_DEBUG_PRINTLN("APIv1 connect");
    ws_instance->connect();
  } else {
    WS_DEBUG_PRINTLN("Unknown API version");
  }
}

void Wippersnapper_Manager::provision() {
  if (_api_version == 2) {
    WS_DEBUG_PRINTLN("APIv2 provisioning");
    //ws_instance_v2->provision();
  } else if (_api_version == 1) {
    WS_DEBUG_PRINTLN("APIv1 provisioning");
    ws_instance->provision();
  } else {
    WS_DEBUG_PRINTLN("Unknown API version");
  }
}

void Wippersnapper_Manager::checkAPIVersion() {
  // Check if pin D12 is high
  pinMode(0, INPUT_PULLUP);
  bool readButton = digitalRead(0);
  // NOTE: For debugging right now, we are forcing APIv1
  readButton = false;
  if (readButton) { // API version 2 if D12 is high
    ws_instance_v2 = new Wippersnapper_V2();
    _api_version = 2;
  } else { // API version 1 if D12 is low
    ws_instance = new Wippersnapper_WiFi();
    _api_version = 1;
  }
}