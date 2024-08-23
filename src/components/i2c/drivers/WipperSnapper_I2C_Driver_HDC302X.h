/*!
 * @file WipperSnapper_I2C_Driver_HDC302X.h
 *
 * Device driver for an HDC302X Humidity and Temperature sensor.
 */

#ifndef WipperSnapper_I2C_Driver_HDC302X_H
#define WipperSnapper_I2C_Driver_HDC302X_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_HDC302x.h>

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the HDC302X humidity and
            temperature sensor. This implementation uses the 1 Hz data rate.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_HDC302X : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an HDC302X sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_HDC302X(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an HDC302X sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_HDC302X() { delete _hdc302x; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the HDC302X sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.

  */
  /*******************************************************************************/
  bool begin() {
    // attempt to initialize the HDC302X using the I2C interface
    _hdc302x = new Adafruit_HDC302x();
    if (!_hdc302x->begin(_sensorAddress, _i2c))
      return false;

    // set the HDC302X's data rate to 1 Hz lowest noise
    _hdc302x->setAutoMode(EXIT_AUTO_MODE);

    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the HDC302X's temperature and humidity data.
      @returns  True if the data was read successfully, False otherwise.
  */
  /*******************************************************************************/
  bool readSensorData() {
    uint16_t status = _hdc302x->readStatus();
    if (status & 0x0010) {
      WS_DEBUG_PRINTLN(F("Device Reset Detected"));
      return false;
    }
    if (status & 0x0001) {
      WS_DEBUG_PRINTLN(
          F("Checksum Verification Fail (incorrect checksum received)"));
      return false;
    }
    if (!_hdc302x->readTemperatureHumidityOnDemand(_temp, _humidity,
                                                   TRIGGERMODE_LP0)) {
      WS_DEBUG_PRINTLN(F("Failed to read temperature and humidity."));
      return false;
    }
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the HDC302X's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    if (readSensorData() == false)
      return false;
    return tempEvent->temperature = _temp;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the HDC302X's current humidity.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    if (readSensorData() == false)
      return false;
    return humidEvent->relative_humidity = _humidity;
  }

protected:
  Adafruit_HDC302x *_hdc302x; ///< Pointer to an HDC302X object
  double _temp = 0.0;     ///< Holds data for the HDC302X's temperature sensor
  double _humidity = 0.0; ///< Holds data for the HDC302X's humidity sensor
};

#endif // WipperSnapper_I2C_Driver_HDC302X