/*!
 * @file WipperSnapper_I2C_Driver.h
 *
 * Base implementation for I2C device drivers.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef WipperSnapper_I2C_Driver_V2_H
#define WipperSnapper_I2C_Driver_V2_H

#include <Adafruit_Sensor.h>
#include <Arduino.h>

#define PERIOD_24HRS_AGO_MILLIS (millis() - (24 * 60 * 60 * 1000))
///< Used for last sensor read time, initially set 24hrs ago (max period)

/**************************************************************************/
/*!
    @brief  Base class for I2C Drivers.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Instanciates an I2C sensor.
      @param    i2c
                The I2C hardware interface, default is Wire.
      @param    address
                The I2C sensor's unique address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver(TwoWire *i2c, uint16_t address) {
    _i2c = i2c;
    _address = address;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an I2C sensor.
  */
  /*******************************************************************************/
  virtual ~WipperSnapper_I2C_Driver() {}

  /*******************************************************************************/
  /*!
      @brief    Initializes the I2C sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  // NOTE: We changed this to virtual so drivers must now reflect: bool begin() override{} 
  virtual bool begin() { }

  /*******************************************************************************/
  /*!
      @brief    Sets the sensor's period.
      @param    period The period for the sensor to return values within, in
                seconds.
  */
  /*******************************************************************************/
  void setSensorPeriod(float period) {
    long _sensor_period = (long)period * 1000;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the I2C device's address.
      @returns  The I2C device's unique i2c address.
  */
  /*******************************************************************************/
  uint16_t GetAddress() { return _address; }

  /*******************************************************************************/
  /*!
      @brief    Gets a sensor's CO2 value.
      @param    co2Event
                The CO2 value, in ppm.
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventCO2(sensors_event_t *co2Event) = 0;

  /*******************************************************************************/
  /*!
      @brief    Gets a sensor's eCO2 value.
      @param    eco2Event
                The equivalent CO2 value, in ppm.
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventECO2(sensors_event_t *eco2Event) = 0;

  /*******************************************************************************/
  /*!
      @brief    Gets a sensor's TVOC value.
      @param    tvocEvent
                The Total Volatile Organic Compounds value, in ppb.
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventTVOC(sensors_event_t *tvocEvent) = 0;

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads an ambient temperature sensor (°C).
                Expects value to return in the proper SI unit.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventAmbientTemp(sensors_event_t *tempEvent) = 0;

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a humidity sensor and converts
                the reading into the expected SI unit.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventRelativeHumidity(sensors_event_t *humidEvent) = 0;

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a pressure sensor and converts
                the reading into the expected SI unit.
      @param    pressureEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventPressure(sensors_event_t *pressureEvent) = 0;

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a Altitude sensor and converts
                the reading into the expected SI unit.
      @param    altitudeEvent
                Altitude reading, in meters.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventAltitude(sensors_event_t *altitudeEvent) = 0;

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a object temperature sensor and
                converts the reading into the expected SI unit.
      @param    objectTempEvent
                object temperature sensor reading, in meters.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventObjectTemp(sensors_event_t *objectTempEvent) = 0;

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a object light sensor and
                converts the reading into the expected SI unit.
      @param    lightEvent
                Light sensor reading, in meters.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventLight(sensors_event_t *lightEvent) = 0;

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a object pm10 std. sensor and
                converts the reading into the expected SI unit.
      @param    pm10StdEvent
                pm10 std. sensor reading, in ppm.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventPM10_STD(sensors_event_t *pm10StdEvent) = 0;

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a object pm25 std. sensor and
                converts the reading into the expected SI unit.
      @param    pm25StdEvent
                pm25 std. sensor reading, in ppm.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventPM25_STD(sensors_event_t *pm25StdEvent) = 0;

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a object pm100 std. sensor and
                converts the reading into the expected SI unit.
      @param    pm100StdEvent
                pm100 std. sensor reading, in ppm.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventPM100_STD(sensors_event_t *pm100StdEvent) = 0;

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a object unitless % std. sensor and
                converts the reading into the expected SI unit.
      @param    unitlessPercentEvent
                unitless %  sensor reading.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventUnitlessPercent(sensors_event_t *unitlessPercentEvent) = 0;

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a voltage sensor and converts the
                reading into the expected SI unit.
      @param    voltageEvent
                voltage sensor reading, in volts.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventVoltage(sensors_event_t *voltageEvent) = 0;

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a current sensor and converts the
                reading into the expected SI unit.
      @param    currentEvent
                current sensor reading, in volts.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventCurrent(sensors_event_t *currentEvent) = 0;

  /*******************************************************************************/
  /*!
      @brief    Gets a sensor's Raw value.
      @param    rawEvent
                The Raw value.
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventRaw(sensors_event_t *rawEvent) = 0;

  /*******************************************************************************/
  /*!
      @brief    Helper function to obtain a sensor's ambient temperature value
                in °F. Requires `getEventAmbientTemp()` to be fully
                implemented by a driver.
      @param    AmbientTempFEvent
                The ambient temperature value, in °F.
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventAmbientTempF(sensors_event_t *AmbientTempFEvent) {
    // obtain ambient temp. in °C
    if (!getEventAmbientTemp(AmbientTempFEvent))
      return false;
    // convert event from °C to °F
    AmbientTempFEvent->temperature =
        (AmbientTempFEvent->temperature * 9.0) / 5.0 + 32;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Helper function to obtain a sensor's object temperature value
                in °F. Requires `getEventObjectTemp()` to be fully
                implemented by a driver.
      @param    objectTempFEvent
                The object temperature value, in °F.
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventObjectTempF(sensors_event_t *objectTempFEvent) {
    // obtain ambient temp. in °C
    if (!getEventObjectTemp(objectTempFEvent))
      return false;
    // convert event from °C to °F
    objectTempFEvent->temperature =
        (objectTempFEvent->temperature * 9.0) / 5.0 + 32.0;
    return true;
  }

  /****************************** SENSOR_TYPE: Gas Resistance (ohms)
   * *******************************/

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a gas resistance sensor and converts
                the reading into the expected SI unit.
      @param    gasEvent
                gas resistance sensor reading, in ohms.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventGasResistance(sensors_event_t *gasEvent) = 0;

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a NOx Index sensor and converts
                the reading into the expected SI unit.
      @param    gasEvent
                NOx Index sensor reading, unitless.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventNOxIndex(sensors_event_t *gasEvent) = 0;

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a VOC Index sensor and converts
                the reading into the expected SI unit.
      @param    gasEvent
                VOC Index sensor reading, unitless.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventVOCIndex(sensors_event_t *gasEvent) = 0;

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a proximity sensor and
                converts the reading into the expected SI unit.
      @param    proximityEvent
                Proximity sensor reading, in millimeters.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventProximity(sensors_event_t *proximityEvent) = 0;

protected:
  TwoWire *_i2c;            ///< Pointer to the I2C bus
  uint16_t _address;        ///< The device's I2C address.
  long _sensor_period;      ///< The sensor's period, in milliseconds.
  long _sensor_period_prv;  ///< The sensor's previous period, in milliseconds.
};

#endif // WipperSnapper_I2C_Driver_H