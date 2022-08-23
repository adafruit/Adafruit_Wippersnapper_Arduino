#ifndef WipperSnapper_I2C_Driver_HTU31D_H
#define WipperSnapper_I2C_Driver_HTU31D_H

#include "WipperSnapper_I2C_Driver.h"

#include <Adafruit_HTU31D.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a HTU31D sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_HTU31D : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a HTU31D sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_HTU31D(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an HTU31D sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_HTU31D() {
    // Called when a HTU31D component is deleted.
    delete _htu31d;
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the HTU31D sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    // Place initialization code here!
    _htu31d = new Adafruit_HTU31D();
    bool isInit = _htu31d->begin((uint8_t)_sensorAddress, _i2c);
    return isInit;
  }

  /*******************************************************************************/
  /*!
      @brief    Enables the HTU31D's temperature sensor.
  */
  /*******************************************************************************/
  void enableSensorAmbientTemperature() {
    _htu31d_temp = _htu31d->getTemperatureSensor();
  }

  /*******************************************************************************/
  /*!
      @brief    Enables the HTU31D's humidity sensor.
  */
  /*******************************************************************************/
  void enableSensorRelativeHumidity() {
    _htu31d_humidity = _htu31d->getHumiditySensor();
  }

  /*******************************************************************************/
  /*!
      @brief    Disables the HTU31D's temperature sensor.
  */
  /*******************************************************************************/
  void disableSensorAmbientTemperature() {
    _htu31d_temp = NULL;
    _tempSensorPeriod = 0.0;
  }

  /*******************************************************************************/
  /*!
      @brief    Disables the HTU31D's humidity sensor.
  */
  /*******************************************************************************/
  void disableSensorRelativeHumidity() {
    _htu31d_humidity = NULL;
    _humidSensorPeriod = 0.0;
  }

  /*******************************************************************************/
  /*!
      @brief    Updates the properties of an ambient temperature
                  sensor, provided sensor_period.
      @param    period
                Sensor's period.
  */
  /*******************************************************************************/
  void updateSensorAmbientTemperature(float period) {
    // disable the sensor
    if (period == 0)
      disableSensorAmbientTemperature();
    // enable a previously disabled sensor
    if (period > 0 && _htu31d_temp == NULL)
      enableSensorAmbientTemperature();

    setSensorAmbientTemperaturePeriod(period);
  }

  /*******************************************************************************/
  /*!
      @brief    Updates the properties of a relative humidity sensor.
      @param    period
                The time interval at which to return new data from the humidity
                sensor.
  */
  /*******************************************************************************/
  void updateSensorRelativeHumidity(float period) {
    // disable the sensor
    if (period == 0)
      disableSensorRelativeHumidity();
    // enable a previously disabled sensor
    if (period > 0 && _htu31d_humidity == NULL)
      enableSensorRelativeHumidity();

    setSensorRelativeHumidityPeriod(period);
  }

 /*******************************************************************************/
  /*!
      @brief    Gets the HTU31D's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemperature(sensors_event_t *tempEvent) {
    if (_htu31d_temp == NULL)
      return false;
    _htu31d_temp->getEvent(tempEvent);
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the HTU31D's current relative humidity reading.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    if (_htu31d_humidity == NULL)
      return false;
    _htu31d_humidity->getEvent(humidEvent);
    return true;
  }

protected:
  Adafruit_HTU31D *_htu31d; ///< Pointer to HTU31D temperature sensor object
  Adafruit_Sensor *_htu31d_temp =
      NULL; ///< Ptr to an adafruit_sensor representing the temperature
  Adafruit_Sensor *_htu31d_humidity =
      NULL; ///< Ptr to an adafruit_sensor representing the humidity
};

#endif // WipperSnapper_I2C_Driver_HTU31D