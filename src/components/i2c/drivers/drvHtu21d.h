/*!
 * @file drvHtu21d.h
 *
 * Device driver for an HTU21D Humidity and Temperature sensor.
 */

#ifndef DRV_HTU21D_H
#define DRV_HTU21D_H

#include "drvBase.h"
#include <Adafruit_HTU21DF.h>

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the HTU21D humidity and
            temperature sensor.
*/
/**************************************************************************/
class drvHtu21d : public drvBase {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an HTU21D sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  /*******************************************************************************/
  drvHtu21d(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an HTU21D sensor.
  */
  /*******************************************************************************/
  ~drvHtu21d() { delete _htu21d; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the HTU21D sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.

  */
  /*******************************************************************************/
  bool begin() {
    // attempt to initialize the HTU21D using the I2C interface
    _htu21d = new Adafruit_HTU21DF();
    return _htu21d->begin(_i2c);
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the HTU21D's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    tempEvent->temperature = _htu21d->readTemperature();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the HTU21D's current humidity.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    humidEvent->relative_humidity = _htu21d->readHumidity();
    return true;
  }

  void ConfigureDefaultSensorTypes() override {
    _default_sensor_types_count = 3;
    _default_sensor_types[0] = wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE;
    _default_sensor_types[1] = wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE_FAHRENHEIT;
    _default_sensor_types[1] = wippersnapper_sensor_SensorType_SENSOR_TYPE_RELATIVE_HUMIDITY;
  }

protected:
  Adafruit_HTU21DF *_htu21d; ///< Pointer to an HTU21D object
};
#endif // drvHtu21d