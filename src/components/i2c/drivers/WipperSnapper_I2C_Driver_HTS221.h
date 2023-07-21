/*!
 * @file WipperSnapper_I2C_Driver_HTS221.h
 *
 * Device driver for an HTS221 Humidity and Temperature sensor.
 */

#ifndef WipperSnapper_I2C_Driver_HTS221_H
#define WipperSnapper_I2C_Driver_HTS221_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_HTS221.h>

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the HTS221 humidity and
            temperature sensor. This implementation uses the 1 Hz data rate.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_HTS221 : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an HTS221 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_HTS221(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an HTS221 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_HTS221() { delete _hts221; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the HTS221 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.

  */
  /*******************************************************************************/
  bool begin() {
    // attempt to initialize the HTS221 using the I2C interface
    _hts221 = new Adafruit_HTS221();
    if (!_hts221->begin_I2C(_sensorAddress, _i2c))
      return false;

    // set the HTS221's data rate to 1 Hz
    _hts221->setDataRate(HTS221_RATE_1_HZ);

    // get temperature and humidity sensor
    _hts221_temp = _hts221->getTemperatureSensor();
    _hts221_humidity = _hts221->getHumiditySensor();

    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the HTS221's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    // is sensor enabled correctly?
    if (_hts221_temp == NULL)
      return false;
    // get temperature and return status
    return _hts221_temp->getEvent(tempEvent);
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the HTS221's current humidity.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    // is sensor enabled correctly?
    if (_hts221_humidity == NULL)
      return false;
    // get humidity and return status
    return _hts221_humidity->getEvent(humidEvent);
  }

protected:
  Adafruit_HTS221 *_hts221; ///< Pointer to an HTS221 object
  Adafruit_Sensor *_hts221_temp =
      NULL; ///< Holds data for the HTS221's temperature sensor
  Adafruit_Sensor *_hts221_humidity =
      NULL; ///< Holds data for the HTS221's humidity sensor
};

#endif // WipperSnapper_I2C_Driver_HTS221