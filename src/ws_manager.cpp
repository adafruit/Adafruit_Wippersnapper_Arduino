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
    WS_DEBUG_PRINTLN("api v2 instance::connect()");
    ws_instance_v2->connectV2();
  } else if (_api_version == 1) {
    WS_DEBUG_PRINTLN("api v1 instance::connect()");
    ws_instance->connect();
  } else {
    WS_DEBUG_PRINTLN("Error: Could not call connect(), unknown API version!");
  }
}

void Wippersnapper_Manager::provision() {
  if (_api_version == 2) {
    WS_DEBUG_PRINTLN("api v2 instance::provision()");
    ws_instance_v2->provisionV2();
  } else if (_api_version == 1) {
    WS_DEBUG_PRINTLN("api v1 instance::provision()");
    ws_instance->provision();
  } else {
    WS_DEBUG_PRINTLN("Error: Could not call provision(), unknown API version!");
  }
}

void Wippersnapper_Manager::checkAPIVersion(int pinNum) {
  // Check if pin D12 is high
  pinMode(pinNum, INPUT_PULLUP);
  bool readButton = digitalRead(pinNum);
  // NOTE: For debugging right now, we are forcing APIv1
  readButton = false;
  if (readButton) { // API version 2 if D12 is high
    ws_instance_v2 = new Wippersnapper_WiFiV2();
    _api_version = 2;
  } else { // API version 1 if D12 is low
    ws_instance = new Wippersnapper_WiFi();
    _api_version = 1;
  }
}