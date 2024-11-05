#ifndef WipperSnapper_I2C_Driver_SGP30_H
#define WipperSnapper_I2C_Driver_SGP30_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_SGP30.h>
#include <ArduinoJson.h>

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
      @param    customProperties
                String of json serialised custom properties passed to the driver
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_SGP30(TwoWire *i2c, uint16_t sensorAddress, String customProperties)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress, customProperties) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
    _customProperties = customProperties;
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


  bool processCustomProperties(){
    // decode the json for RH + Temp, ideally the dynamic baseline bytes too
    if (customProperties.length() > 0) {
      StaticJsonDocument<100> doc;
      DeserializationError error = deserializeJson(doc, customProperties);
      if (error) {
        WS_DEBUG_PRINT("deserializeJson() failed: ");
        WS_DEBUG_PRINTLN(error.c_str());
        return;
      }
      //set backing RH+Temp
      if (doc.containsKey("input-temp")){
        ref_temp = doc["input-temp"];
      }
      if (doc.containsKey("input-humidity")){
        ref_rh = doc["input-humidity"];
      }
    }
  }

  bool setBaseLineRHT(){
    if (ref_rh == 0.0) {
      return _sgp30->setHumidity(0); // turn off humidity compensation
    } // else convert Relative Humidity + Temp to absolute humidity (mg/m3)
    uint32_t absoluteHumidity = ref_rh + ref_temp; // not this!!!
    return _sgp30->setHumidity(absoluteHumidity/1000);
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the SGP30 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _sgp30 = new Adafruit_SGP30();
    bool isInit = _sgp30->begin(_i2c);
    if (isInit && processCustomProperties()) {
      // we catch the fail and avoid activating humidity compensation.
      isInit = setBaseLineRHT();
    }
    if (isInit) {
      _sgp30->IAQinit();
    }
    return isInit;
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
  float ref_temp = 20.0f;
  float ref_rh = 50.0f;
  Adafruit_SGP30 *_sgp30; ///< Pointer to SGP30 temperature sensor object
};

#endif // WipperSnapper_I2C_Driver_SGP30