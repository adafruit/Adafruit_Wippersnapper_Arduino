#ifndef WipperSnapper_I2C_Driver_SGP30_H
#define WipperSnapper_I2C_Driver_SGP30_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_SGP30.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a SGP30 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_SGP30 : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a SGP30 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_SGP30(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an SGP30 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_SGP30() {
    // Called when a SGP30 component is deleted.
    delete _sgp30;
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the SGP30 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _sgp30 = new Adafruit_SGP30();
    return _sgp30->begin(_i2c);
  }

  bool getEventECO2(sensors_event_t *senseEvent) {
    bool result = _sgp30->IAQmeasure();
    if (result) {
      senseEvent->eCO2 = _sgp30->eCO2;
    }
    return result;
  }

  bool getEventTVOC(sensors_event_t *senseEvent) {
    bool result = _sgp30->IAQmeasure();
    if (result) {
      senseEvent->tvoc = _sgp30->TVOC;
    }
    return result;
  }

protected:
  Adafruit_SGP30 *_sgp30; ///< Pointer to SGP30 temperature sensor object
};

#endif // WipperSnapper_I2C_Driver_SGP30