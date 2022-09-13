/*!
 * @file WipperSnapper_I2C_Driver.h
 *
 * Base implementation for I2C device drivers.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2021-2022 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef WipperSnapper_I2C_Driver_H
#define WipperSnapper_I2C_Driver_H

#include <Adafruit_Sensor.h>
#include <Arduino.h>

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
      @param    sensorAddress
                The I2C sensor's unique address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver(TwoWire *i2c, uint16_t sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an I2C sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver() { _sensorAddress = 0; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the I2C sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() { return false; }

  // TODO: consider passing sensor type into enable/disable/set sensor period
  // calls

  void setSensorPeriod(float period,
                       wippersnapper_i2c_v1_SensorType sensorType) {
    long sensorPeriod = (long)period * 1000;

    switch (sensorType) {
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE:
      _tempSensorPeriod = sensorPeriod;
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_RELATIVE_HUMIDITY:
      _humidSensorPeriod = sensorPeriod;
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PRESSURE:
      _pressureSensorPeriod = sensorPeriod;
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_CO2:
      _CO2SensorPeriod = sensorPeriod;
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_ALTITUDE:
      _altitudeSensorPeriod = sensorPeriod;
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_OBJECT_TEMPERATURE:
      _objectTempSensorPeriod = sensorPeriod;
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_LIGHT:
      _lightSensorPeriod = sensorPeriod;
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM10_STD:
      _PM10SensorPeriod = sensorPeriod;
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM25_STD:
      _PM25SensorPeriod = sensorPeriod;
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PM100_STD:
      _PM100SensorPeriod = sensorPeriod;
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_UNITLESS_PERCENT:
      _unitlessPercentPeriod = sensorPeriod;
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_VOLTAGE:
      _voltagePeriod = sensorPeriod;
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_RAW:
      _rawSensorPeriod = sensorPeriod;
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE_FAHRENHEIT:
      _ambientTempFPeriod = sensorPeriod;
      break;
    default:
      break;
    }
  }

  /*******************************************************************************/
  /*!
      @brief    Uses an I2CDeviceInitRequest message to configure the sensors
                  belonging to the driver.
      @param    msgDeviceInitReq
                I2CDeviceInitRequest containing a list of I2C device properties.
  */
  /*******************************************************************************/
  void
  configureDriver(wippersnapper_i2c_v1_I2CDeviceInitRequest *msgDeviceInitReq) {
    int propertyIdx = 0; // contains the amount of i2c sensors in the
                         // msgDeviceInitReq to configure
    while (propertyIdx < msgDeviceInitReq->i2c_device_properties_count) {
      setSensorPeriod(
          msgDeviceInitReq->i2c_device_properties[propertyIdx].sensor_period,
          msgDeviceInitReq->i2c_device_properties[propertyIdx].sensor_type);
      ++propertyIdx;
    }
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the I2C device's address.
      @returns  The I2C device's unique i2c address.
  */
  /*******************************************************************************/
  uint16_t getI2CAddress() { return _sensorAddress; }

  /****************************** SENSOR_TYPE: CO2
   * *******************************/
  /*******************************************************************************/
  /*!
      @brief    Enables the device's CO2 sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void enableSensorCO2() { return; };

  /*******************************************************************************/
  /*!
      @brief    Disables the device's CO2 sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void disableSensorCO2() { _CO2SensorPeriod = 0.0L; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the co2 sensor's period, if
     set.
      @returns  Time when the co2 sensor should be polled, in seconds.
  */
  /*********************************************************************************/
  virtual long sensorCO2Period() { return _CO2SensorPeriod; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the previous time interval at
                    which the co2 sensor was queried last.
      @returns  Time when the co2 sensor was last queried, in seconds.
  */
  /*********************************************************************************/
  virtual long sensorCO2PeriodPrv() { return _CO2SensorPeriodPrv; }

  /*******************************************************************************/
  /*!
      @brief    Sets a timestamp for when the co2 sensor was queried.
      @param    period
                The time when the co2 sensor was queried last.
  */
  /*******************************************************************************/
  virtual void setSensorCO2PeriodPrv(long period) {
    _CO2SensorPeriodPrv = period;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets a sensor's CO2 value.
      @param    co2Event
                The CO2 value, in ppm.
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventCO2(sensors_event_t *co2Event) { return false; }

  /********************** SENSOR_TYPE: AMBIENT TEMPERATURE (°C)
   * ***********************/
  /*******************************************************************************/
  /*!
      @brief    Enables the device's ambient temperature sensor (°C), if it
     exists.
  */
  /*******************************************************************************/
  virtual void enableSensorAmbientTemperature() { return; };

  /*******************************************************************************/
  /*!
      @brief    Disables the device's ambient temperature sensor (°C), if it
     exists.
  */
  /*******************************************************************************/
  virtual void disableSensorAmbientTemperature() { _tempSensorPeriod = 0.0L; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the ambient temperature (°C)
     sensor's period, if set.
      @returns  Time when the temperature sensor should be polled, in seconds.
  */
  /*********************************************************************************/
  virtual long sensorAmbientTemperaturePeriod() { return _tempSensorPeriod; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the previous time interval at
                which the ambient temperature sensor (°C) was queried last.
      @returns  Time when the ambient temperature sensor (°C) was last queried,
                in seconds.
  */
  /*********************************************************************************/
  virtual long sensorAmbientTemperaturePeriodPrv() {
    return _tempSensorPeriodPrv;
  }

  /*******************************************************************************/
  /*!
      @brief    Sets a timestamp for when the ambient temperature sensor (°C)
                was queried.
      @param    periodPrv
                The time when the ambient temperature sensor (°C) was queried
     last.
  */
  /*******************************************************************************/
  virtual void setSensorAmbientTemperaturePeriodPrv(long periodPrv) {
    _tempSensorPeriodPrv = periodPrv;
  }

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
  virtual bool getEventAmbientTemperature(sensors_event_t *tempEvent) {
    return false;
  }

  /************************* SENSOR_TYPE: RELATIVE_HUMIDITY
   * ***********************/
  /*******************************************************************************/
  /*!
      @brief    Enables the device's relative humidity sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void enableSensorRelativeHumidity(){};

  /*******************************************************************************/
  /*!
      @brief    Disables the device's relative humidity sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void disableSensorRelativeHumidity() { _humidSensorPeriod = 0.0L; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the humidity sensor's period, if
     set.
      @returns  Time when the humidity sensor should be polled, in seconds.
  */
  /*********************************************************************************/
  virtual long sensorRelativeHumidityPeriod() { return _humidSensorPeriod; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the previous time interval at
     which the humidity sensor was queried last.
      @returns  Time when the humidity sensor was last queried, in seconds.
  */
  /*********************************************************************************/
  virtual long sensorRelativeHumidityPeriodPrv() {
    return _humidSensorPeriodPrv;
  }

  /*******************************************************************************/
  /*!
      @brief    Sets a timestamp for when the temperature sensor was queried.
      @param    periodPrv
                The time when the temperature sensor was queried last.
  */
  /*******************************************************************************/
  virtual void setSensorRelativeHumidityPeriodPrv(long periodPrv) {
    _humidSensorPeriodPrv = periodPrv;
  }

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
  virtual bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    return false;
  }

  /**************************** SENSOR_TYPE: PRESSURE
   * ****************************/
  /*******************************************************************************/
  /*!
      @brief    Enables the device's pressure sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void enableSensorPressure(){};

  /*******************************************************************************/
  /*!
      @brief    Disables the device's pressure sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void disableSensorPressure() { _pressureSensorPeriod = 0.0L; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the pressure sensor's period, if
     set.
      @returns  Time when the pressure sensor should be polled, in seconds.
  */
  /*********************************************************************************/
  virtual long sensorPressurePeriod() { return _pressureSensorPeriod; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the previous time interval at
                    which the pressure sensor was queried last.
      @returns  Time when the pressure sensor was last queried, in seconds.
  */
  /*********************************************************************************/
  virtual long sensorPressurePeriodPrv() { return _pressureSensorPeriodPrv; }

  /*******************************************************************************/
  /*!
      @brief    Sets a timestamp for when the pressure sensor was queried.
      @param    period
                The time when the pressure sensor was queried last.
  */
  /*******************************************************************************/
  virtual void setSensorPressurePeriodPrv(long period) {
    _pressureSensorPeriodPrv = period;
  }

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
  virtual bool getEventPressure(sensors_event_t *pressureEvent) {
    return false;
  }

  /**************************** SENSOR_TYPE: Altitude
   * ****************************/
  /*******************************************************************************/
  /*!
      @brief    Enables the device's Altitude sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void enableSensorAltitude(){};

  /*******************************************************************************/
  /*!
      @brief    Disables the device's Altitude sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void disableSensorAltitude() { _altitudeSensorPeriod = 0.0L; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the Altitude sensor's period, if
     set.
      @returns  Time when the Altitude sensor should be polled, in seconds.
  */
  /*********************************************************************************/
  virtual long sensorAltitudePeriod() { return _altitudeSensorPeriod; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the previous time interval at
                    which the Altitude sensor was queried last.
      @returns  Time when the Altitude sensor was last queried, in seconds.
  */
  /*********************************************************************************/
  virtual long sensorAltitudePeriodPrv() { return _altitudeSensorPeriodPrv; }

  /*******************************************************************************/
  /*!
      @brief    Sets a timestamp for when the Altitude sensor was queried.
      @param    period
                The time when the Altitude sensor was queried last.
  */
  /*******************************************************************************/
  virtual void setSensorAltitudePeriodPrv(long period) {
    _altitudeSensorPeriodPrv = period;
  }

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
  virtual bool getEventAltitude(sensors_event_t *altitudeEvent) {
    return false;
  }

  /**************************** SENSOR_TYPE: Object_Temperature
   * ****************************/
  /*******************************************************************************/
  /*!
      @brief    Enables the device's object temperature sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void enableSensorObjectTemp(){};

  /*******************************************************************************/
  /*!
      @brief    Disables the device's object temperature sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void disableSensorObjectTemp() { _objectTempSensorPeriod = 0.0L; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the object temperature sensor's
     period, if set.
      @returns  Time when the object temperature sensor should be polled, in
     seconds.
  */
  /*********************************************************************************/
  virtual long sensorObjectTempPeriod() { return _objectTempSensorPeriod; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the previous time interval at
                    which the object temperature sensor was queried last.
      @returns  Time when the object temperature sensor was last queried,
                in seconds.
  */
  /*********************************************************************************/
  virtual long sensorObjectTempPeriodPrv() {
    return _objectTempSensorPeriodPrv;
  }

  /*******************************************************************************/
  /*!
      @brief    Sets a timestamp for when the object temperature sensor
                was queried.
      @param    period
                The time when the object temperature sensor was queried last.
  */
  /*******************************************************************************/
  virtual void setSensorObjectTempPeriodPrv(long period) {
    _objectTempSensorPeriodPrv = period;
  }

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
  virtual bool getEventObjectTemp(sensors_event_t *objectTempEvent) {
    return false;
  }

  /**************************** SENSOR_TYPE: LIGHT
   * ****************************/
  /*******************************************************************************/
  /*!
      @brief    Enables the device's object light sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void enableSensorLight(){};

  /*******************************************************************************/
  /*!
      @brief    Disables the device's object light sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void disableSensorLight() { _lightSensorPeriod = 0.0L; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the object light sensor's
     period, if set.
      @returns  Time when the object light sensor should be polled, in
     seconds.
  */
  /*********************************************************************************/
  virtual long sensorLightPeriod() { return _lightSensorPeriod; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the previous time interval at
                    which the light sensor was queried last.
      @returns  Time when the light sensor was last queried,
                in seconds.
  */
  /*********************************************************************************/
  virtual long SensorLightPeriodPrv() { return _lightSensorPeriodPrv; }

  /*******************************************************************************/
  /*!
      @brief    Sets a timestamp for when the light sensor
                was queried.
      @param    period
                The time when the light sensor was queried last.
  */
  /*******************************************************************************/
  virtual void setSensorLightPeriodPrv(long period) {
    _lightSensorPeriodPrv = period;
  }

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
  virtual bool getEventLight(sensors_event_t *lightEvent) { return false; }

  /**************************** SENSOR_TYPE: PM10_STD
   * ****************************/
  /*******************************************************************************/
  /*!
      @brief    Enables sensor's pm10 standard readings, if it exists.
  */
  /*******************************************************************************/
  virtual void enableSensorPM10_STD(){};

  /*******************************************************************************/
  /*!
      @brief    Disables the sensor's pm10 standard readings, if it exists.
  */
  /*******************************************************************************/
  virtual void disableSensorPM10_STD() { _PM10SensorPeriod = 0.0L; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the object pm10 standard sensors'
     period, if set.
      @returns  Time when the object pm10 standard sensor should be polled, in
     seconds.
  */
  /*********************************************************************************/
  virtual long sensorPM10_STDPeriod() { return _PM10SensorPeriod; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the previous time interval at
                    which the pm10 std. sensor was queried last.
      @returns  Time when the pm10 std. sensor was last queried,
                in seconds.
  */
  /*********************************************************************************/
  virtual long SensorPM10_STDPeriodPrv() { return _PM10SensorPeriodPrv; }

  /*******************************************************************************/
  /*!
      @brief    Sets a timestamp for when the light sensor
                was queried.
      @param    period
                The time when the light sensor was queried last.
  */
  /*******************************************************************************/
  virtual void setSensorPM10_STDPeriodPrv(long period) {
    _PM10SensorPeriodPrv = period;
  }

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
  virtual bool getEventPM10_STD(sensors_event_t *pm10StdEvent) { return false; }

  /**************************** SENSOR_TYPE: PM25_STD
   * ****************************/
  /*******************************************************************************/
  /*!
      @brief    Enables sensor's pm25 standard readings, if it exists.
  */
  /*******************************************************************************/
  virtual void enableSensorPM25_STD(){};

  /*******************************************************************************/
  /*!
      @brief    Disables the sensor's pm25 standard readings, if it exists.
  */
  /*******************************************************************************/
  virtual void disableSensorPM25_STD() { _lightSensorPeriod = 0.0L; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the object pm25 standard sensors'
     period, if set.
      @returns  Time when the object pm25 standard sensor should be polled, in
     seconds.
  */
  /*********************************************************************************/
  virtual long sensorPM25_STDPeriod() { return _PM25SensorPeriod; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the previous time interval at
                    which the pm25 std. sensor was queried last.
      @returns  Time when the pm25 std. sensor was last queried,
                in seconds.
  */
  /*********************************************************************************/
  virtual long SensorPM25_STDPeriodPrv() { return _PM25SensorPeriodPrv; }

  /*******************************************************************************/
  /*!
      @brief    Sets a timestamp for when the light sensor
                was queried.
      @param    period
                The time when the light sensor was queried last.
  */
  /*******************************************************************************/
  virtual void setSensorPM25_STDPeriodPrv(long period) {
    _PM25SensorPeriodPrv = period;
  }

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
  virtual bool getEventPM25_STD(sensors_event_t *pm25StdEvent) { return false; }

  /**************************** SENSOR_TYPE: PM100_STD
   * ****************************/
  /*******************************************************************************/
  /*!
      @brief    Enables sensor's pm100 standard readings, if it exists.
  */
  /*******************************************************************************/
  virtual void enableSensorPM100_STD(){};

  /*******************************************************************************/
  /*!
      @brief    Disables the sensor's pm100 standard readings, if it exists.
  */
  /*******************************************************************************/
  virtual void disableSensorPM100_STD() { _PM100SensorPeriod = 0.0L; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the object pm100 standard sensors'
     period, if set.
      @returns  Time when the object pm100 standard sensor should be polled, in
     seconds.
  */
  /*********************************************************************************/
  virtual long sensorPM100_STDPeriod() { return _PM100SensorPeriod; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the previous time interval at
                    which the pm100 std. sensor was queried last.
      @returns  Time when the pm100 std. sensor was last queried,
                in seconds.
  */
  /*********************************************************************************/
  virtual long SensorPM100_STDPeriodPrv() { return _PM100SensorPeriodPrv; }

  /*******************************************************************************/
  /*!
      @brief    Sets a timestamp for when the light sensor
                was queried.
      @param    period
                The time when the light sensor was queried last.
  */
  /*******************************************************************************/
  virtual void setSensorPM100_STDPeriodPrv(long period) {
    _PM100SensorPeriodPrv = period;
  }

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
  virtual bool getEventPM100_STD(sensors_event_t *pm100StdEvent) {
    return false;
  }

  /**************************** SENSOR_TYPE: UNITLESS_PERCENT
   * ****************************/
  /*******************************************************************************/
  /*!
      @brief    Enables sensor's unitless % readings, if it exists.
  */
  /*******************************************************************************/
  virtual void enableSensorUnitlessPercent(){};

  /*******************************************************************************/
  /*!
      @brief    Disables the sensor's unitless % standard readings, if it
     exists.
  */
  /*******************************************************************************/
  virtual void disableSensorUnitlessPercent() { _unitlessPercentPeriod = 0.0L; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the object unitless % sensor
                period, if set.
      @returns  Time when the object unitless % sensor should be polled, in
                seconds.
  */
  /*********************************************************************************/
  virtual long sensorUnitlessPercentPeriod() { return _unitlessPercentPeriod; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the previous time interval at
                which the unitless % sensor was queried last.
      @returns  Time when the unitless % sensor was last queried,
                in seconds.
  */
  /*********************************************************************************/
  virtual long sensorUnitlessPercentPeriodPrv() {
    return _unitlessPercentPeriodPrv;
  }

  /*******************************************************************************/
  /*!
      @brief    Sets a timestamp for when the unitless % sensor
                was queried.
      @param    period
                The time when the unitless % sensor was queried last.
  */
  /*******************************************************************************/
  virtual void setSensorUnitlessPercentPeriodPrv(long period) {
    _unitlessPercentPeriodPrv = period;
  }

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
  virtual bool getEventUnitlessPercent(sensors_event_t *unitlessPercentEvent) {
    return false;
  }

  /**************************** SENSOR_TYPE: VOLTAGE
   * ****************************/
  /*******************************************************************************/
  /*!
      @brief    Enables sensor's voltage readings, if it exists.
  */
  /*******************************************************************************/
  virtual void enableSensorVoltage(){};

  /*******************************************************************************/
  /*!
      @brief    Disables the sensor's voltage standard readings, if it
     exists.
  */
  /*******************************************************************************/
  virtual void disableSensorVoltage() { _voltagePeriod = 0.0L; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the voltage sensor's period.
      @returns  Time when the object voltage sensor should be polled, in
     seconds.
  */
  /*********************************************************************************/
  virtual long sensorVoltagePeriod() { return _voltagePeriod; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the previous time interval at
                which the voltage sensor was queried last.
      @returns  Time when the voltage sensor was last queried, in seconds.
  */
  /*********************************************************************************/
  virtual long SensorVoltagePeriodPrv() { return _voltagePeriodPrv; }

  /*******************************************************************************/
  /*!
      @brief    Sets a timestamp for when the voltage sensor was queried.
      @param    period
                The time when the voltage sensor was queried last.
  */
  /*******************************************************************************/
  virtual void setSensorVoltagePeriodPrv(long period) {
    _voltagePeriodPrv = period;
  }

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
  virtual bool getEventVoltage(sensors_event_t *voltageEvent) { return false; }

  /****************************** SENSOR_TYPE: Raw
   * *******************************/
  /*******************************************************************************/
  /*!
      @brief    Enables the device's Raw sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void enableSensorRaw() { return; };

  /*******************************************************************************/
  /*!
      @brief    Disables the device's Raw sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void disableSensorRaw() { _rawSensorPeriod = 0.0L; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the raw sensor's period, if
     set.
      @returns  Time when the raw sensor should be polled, in seconds.
  */
  /*********************************************************************************/
  virtual long sensorRawPeriod() { return _rawSensorPeriod; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the previous time interval at
                    which the raw sensor was queried last.
      @returns  Time when the raw sensor was last queried, in seconds.
  */
  /*********************************************************************************/
  virtual long sensorRawPeriodPrv() { return _rawSensorPeriodPrv; }

  /*******************************************************************************/
  /*!
      @brief    Sets a timestamp for when the raw sensor was queried.
      @param    period
                The time when the raw sensor was queried last.
  */
  /*******************************************************************************/
  virtual void setSensorRawPeriodPrv(long period) {
    _rawSensorPeriodPrv = period;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets a sensor's Raw value.
      @param    rawEvent
                The Raw value.
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventRaw(sensors_event_t *rawEvent) { return false; }

  /****************************** SENSOR_TYPE: Ambient Temp (°F)
   * *******************************/
  /*******************************************************************************/
  /*!
      @brief    Enables the device's ambient temperature (°F) sensor, if it
     exists.
  */
  /*******************************************************************************/
  virtual void enableSensorAmbientTempF() { return; };

  /*******************************************************************************/
  /*!
      @brief    Disables the device's ambient temperature (°F) sensor, if it
     exists.
  */
  /*******************************************************************************/
  virtual void disableAmbientTempF() { _ambientTempFPeriod = 0.0L; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the ambient temperature (°F)
     sensor's period, if set.
      @returns  Time when the ambient temperature sensor should be polled,
                in seconds.
  */
  /*********************************************************************************/
  virtual long sensorAmbientTempFPeriod() { return _ambientTempFPeriod; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the previous time interval at
                which the ambient temperature sensor (°F) was queried last.
      @returns  Time when the ambient temperature sensor was last queried,
                in seconds.
  */
  /*********************************************************************************/
  virtual long sensorAmbientTempFPeriodPrv() { return _ambientTempFPeriodPrv; }

  /*******************************************************************************/
  /*!
      @brief    Sets a timestamp for when the ambient temperature sensor (°F)
                was queried.
      @param    period
                The time when the ambient temperature sensor (°F) was queried
     last.
  */
  /*******************************************************************************/
  virtual void setSensorAmbientTempFPeriodPrv(long period) {
    _ambientTempFPeriodPrv = period;
  }

  /*******************************************************************************/
  /*!
      @brief    Helper function to obtain a sensor's ambient temperature value
                in °F. Requires `getEventAmbientTemperature()` to be fully
                implemented by a driver.
      @param    AmbientTempFEvent
                The ambient temperature value, in °F.
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getAmbientTempF(sensors_event_t *AmbientTempFEvent) {
    // obtain ambient temp. in °C
    if (!getEventAmbientTemperature(AmbientTempFEvent))
      return false;
    // convert event from °C to °F
    AmbientTempFEvent->temperature =
        (AmbientTempFEvent->temperature * 9.0) / 5.0 + 32;
    return true;
  }

protected:
  TwoWire *_i2c;           ///< Pointer to the I2C driver's Wire object
  uint16_t _sensorAddress; ///< The I2C driver's unique I2C address.
  long _tempSensorPeriod =
      0L; ///< The time period between reading the temperature sensor's value.
  long _tempSensorPeriodPrv =
      0L; ///< The time when the temperature sensor was last read
  long _humidSensorPeriod =
      0L; ///< The time period between reading the humidity sensor's value.
  long _humidSensorPeriodPrv = 0L; ///< The time when the humidity sensor was
                                   ///< last read.
  long _pressureSensorPeriod =
      0L; ///< The time period between reading the pressure sensor's value.
  long _pressureSensorPeriodPrv = 0L; ///< The time when the pressure sensor
                                      ///< was last read.
  long _CO2SensorPeriod =
      0L; ///< The time period between reading the CO2 sensor's value.
  long _CO2SensorPeriodPrv = 0L; ///< The time when the CO2 sensor
                                 ///< was last read.
  long _altitudeSensorPeriod =
      0L; ///< The time period between reading the altitude sensor's value.
  long _altitudeSensorPeriodPrv = 0L;   ///< The time when the altitude sensor
                                        ///< was last read.
  long _objectTempSensorPeriod = 0L;    ///< The time period between reading the
                                        ///< object temperature sensor's value.
  long _objectTempSensorPeriodPrv = 0L; ///< The time when the object
                                        ///< temperature sensor was last read.
  long _lightSensorPeriod = 0L;         ///< The time period between reading the
                                        ///< light sensor's value.
  long _lightSensorPeriodPrv = 0L;      ///< The time when the light sensor
                                        ///< was last read.
  long _PM10SensorPeriod = 0L;          ///< The time period between reading the
                                        ///< pm25 sensor's value.
  long _PM10SensorPeriodPrv = 0L;       ///< The time when the pm25 sensor
                                        ///< was last read.
  long _PM25SensorPeriod = 0L;          ///< The time period between reading the
                                        ///< pm25 sensor's value.
  long _PM25SensorPeriodPrv = 0L;       ///< The time when the pm25 sensor
                                        ///< was last read.
  long _PM100SensorPeriod = 0L;         ///< The time period between reading the
                                        ///< pm100_std sensor's value.
  long _PM100SensorPeriodPrv = 0L;      ///< The time when the pm100_std sensor
                                        ///< was last read.
  long _unitlessPercentPeriod = 0L;     ///< The time period between reading the
                                        ///< unitless % sensor's value.
  long _unitlessPercentPeriodPrv = 0L;  ///< The time when the unitless % sensor
                                        ///< was last read.
  long _voltagePeriod = 0L;             ///< The time period between reading the
                                        ///< voltage sensor's value.
  long _voltagePeriodPrv = 0L;          ///< The time when the voltage sensor
                                        ///< was last read.
  long _rawSensorPeriod =
      0L; ///< The time period between reading the Raw sensor's value.
  long _rawSensorPeriodPrv = 0L;    ///< The time when the Raw sensor
                                    ///< was last read.
  long _ambientTempFPeriod = 0L;    ///< The time period between reading the
                                    ///< ambient temp. (degF) sensor's value.
  long _ambientTempFPeriodPrv = 0L; ///< The time when the ambient temp. (degF)
                                    ///< sensor was last read.
};

#endif // WipperSnapper_I2C_Driver_H
