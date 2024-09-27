#include "ws_manager.h"

/**************************************************************************/
/*!
    @brief    Constructor for Wippersnapper_Manager
*/
/**************************************************************************/
Wippersnapper_Manager::Wippersnapper_Manager() : ws_instance(nullptr) {}

/**************************************************************************/
/*!
    @brief    Destructor for Wippersnapper_Manager
*/
/**************************************************************************/
Wippersnapper_Manager::~Wippersnapper_Manager() {
  if (ws_instance) {
    delete ws_instance;
  }
  if (ws_instance_v2) {
    delete ws_instance_v2;
  }
}

/**************************************************************************/
/*!
    @brief    Performs the connect() function in Wippersnapper*.cpp
*/
/**************************************************************************/
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

/**************************************************************************/
/*!
    @brief    Performs the provision() function in Wippersnapper*.cpp
*/
/**************************************************************************/
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

/**************************************************************************/
/*!
    @brief    Checks the API version by reading the state of a pin
    @param    pinNum
              The pin number to read
*/
/**************************************************************************/
void Wippersnapper_Manager::checkAPIVersion(int pinNum) {
  // Check if pin D12 is high
  pinMode(pinNum, INPUT_PULLUP);
  bool readButton = digitalRead(pinNum);
  // NOTE: For debugging right now, we are forcing the API version
  readButton = true;
  if (readButton) { // API version 2 if D12 is high
    ws_instance_v2 = new Wippersnapper_WiFiV2();
    _api_version = 2;
  } else { // API version 1 if D12 is low
    ws_instance = new Wippersnapper_WiFi();
    _api_version = 1;
  }
}

/**************************************************************************/
/*!
    @brief    Performs the run() function in Wippersnapper*.cpp
*/
/**************************************************************************/
void Wippersnapper_Manager::run() {
  if (_api_version == 2) {
    ws_instance_v2->runV2();
  } else if (_api_version == 1) {
    ws_instance->run();
  } else {
    WS_DEBUG_PRINTLN("Error: Could not call run(), unknown API version!");
  }
}