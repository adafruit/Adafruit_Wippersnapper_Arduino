#ifndef WipperSnapper_I2C_Driver_MCP9808_H
#define WipperSnapper_I2C_Driver_MCP9808_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_MCP9808>

class WipperSnapper_I2C_Driver_MCP9808 : public WipperSnapper_I2C_Driver {
public:
  WipperSnapper_I2C_Driver_MCP9808(TwoWire *_i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(_i2c, sensorAddress) {
    // Called when a MCP9808 component is created
    setI2CAddress(sensorAddress); // sets the driver's I2C address
    _mcp9808 = new Adafruit_MCP9808();
    _isInitialized = _mcp9808.begin();
  }

  ~WipperSnapper_I2C_Driver_MCP9808() {
    // Called when a MCP9808 component is deleted.
    delete _mcp9808;
  }

  bool getEventAmbientTemperature(sensors_event_t *tempEvent) {
    tempEvent->temperature = _mcp9808->readTempC();
    return true;
  }

protected:
  Adafruit_MCP9808 *_mcp9808; ///< Pointer to MCP9808 temperature sensor object
};

#endif // WipperSnapper_I2C_Driver_MCP9808