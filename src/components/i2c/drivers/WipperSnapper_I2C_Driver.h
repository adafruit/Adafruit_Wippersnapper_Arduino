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
    int propertyIdx = 0;
    while (propertyIdx < msgDeviceInitReq->i2c_device_properties_count) {
      switch (
          msgDeviceInitReq->i2c_device_properties[propertyIdx].sensor_type) {
      case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE:
        enableSensorAmbientTemperature();
        setSensorAmbientTemperaturePeriod(
            msgDeviceInitReq->i2c_device_properties[propertyIdx].sensor_period);
        break;
      case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_RELATIVE_HUMIDITY:
        enableSensorRelativeHumidity();
        setSensorRelativeHumidityPeriod(
            msgDeviceInitReq->i2c_device_properties[propertyIdx].sensor_period);
        break;
      case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PRESSURE:
        enableSensorPressure();
        setSensorPressurePeriod(
            msgDeviceInitReq->i2c_device_properties[propertyIdx].sensor_period);
        break;
      case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_CO2:
        enableSensorCO2();
        setSensorCO2Period(
            msgDeviceInitReq->i2c_device_properties[propertyIdx].sensor_period);
        break;
      case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_GAS_RESISTANCE:
        enableSensorGas();
        setSensorGasPeriod(
            msgDeviceInitReq->i2c_device_properties[propertyIdx].sensor_period);
        break;
      case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_ALTITUDE:
        enableSensorAltitude();
        setSensorAltitudePeriod(
            msgDeviceInitReq->i2c_device_properties[propertyIdx].sensor_period);
        break;
      case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_OBJECT_TEMPERATURE:
        enableSensorObjectTemp();
        setSensorObjectTempPeriod(
            msgDeviceInitReq->i2c_device_properties[propertyIdx].sensor_period);
        break;
      case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_LIGHT:
        enableSensorLight();
        setSensorLightPeriod(
            msgDeviceInitReq->i2c_device_properties[propertyIdx].sensor_period);
        break;
      case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PROXIMITY:
        enableSensorProximity();
        setSensorProximityPeriod(
            msgDeviceInitReq->i2c_device_properties[propertyIdx].sensor_period);
        break;
      default:
        break;
      }
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

  /*******************************************************************************/
  /*!
      @brief    Set the co2 sensor's return frequency.
      @param    period
                The time interval at which to return new data from the
     co2 sensor.
  */
  /*******************************************************************************/
  virtual void setSensorCO2Period(float period) {
    if (period == 0.0) {
      disableSensorCO2();
      return;
    }
    // Period is in seconds, cast it to long and convert it to milliseconds
    _CO2SensorPeriod = (long)period * 1000;
  }

  /*******************************************************************************/
  /*!
      @brief    Updates the properties of a CO2 sensor.
      @param    period
                The time interval at which to return new data from the CO2
                sensor.
  */
  /*******************************************************************************/
  virtual void updateSensorCO2(float period) { setSensorCO2Period(period); }

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

  /********************** SENSOR_TYPE: AMBIENT TEMPERATURE
   * ***********************/
  /*******************************************************************************/
  /*!
      @brief    Enables the device's temperature sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void enableSensorAmbientTemperature() { return; };

  /*******************************************************************************/
  /*!
      @brief    Disables the device's temperature sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void disableSensorAmbientTemperature() { _tempSensorPeriod = 0.0L; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the humidity sensor's period, if
     set.
      @returns  Time when the temperature sensor should be polled, in seconds.
  */
  /*********************************************************************************/
  virtual long sensorAmbientTemperaturePeriod() { return _tempSensorPeriod; }

  /*******************************************************************************/
  /*!
      @brief    Set the temperature sensor's return frequency.
      @param    period
                The time interval at which to return new data from the
     temperature sensor.
  */
  /*******************************************************************************/
  virtual void setSensorAmbientTemperaturePeriod(float period) {
    if (period == 0.0) {
      disableSensorAmbientTemperature();
      return;
    }
    // Period is in seconds, cast it to long and convert it to milliseconds
    _tempSensorPeriod = (long)period * 1000;
  }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the previous time interval at
     which the temperature sensor was queried last.
      @returns  Time when the temperature sensor was last queried, in seconds.
  */
  /*********************************************************************************/
  virtual long sensorAmbientTemperaturePeriodPrv() {
    return _tempSensorPeriodPrv;
  }

  /*******************************************************************************/
  /*!
      @brief    Sets a timestamp for when the temperature sensor was queried.
      @param    periodPrv
                The time when the temperature sensor was queried last.
  */
  /*******************************************************************************/
  virtual void setSensorAmbientTemperaturePeriodPrv(long periodPrv) {
    _tempSensorPeriodPrv = periodPrv;
  }

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a temperature sensor. Expects value
                to return in the proper SI unit.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventAmbientTemperature(sensors_event_t *tempEvent) {
    return false;
  }

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a temperature sensor. Expects value
                to return in the proper SI unit.
      @param    tempEvent
                Pointer to an temperature value.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventAmbientTemperature(float tempEvent) { return false; }

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Update the properties of a temperature
                  sensor, provided sensor_period.
      @param    period
                Sensor's period.
  */
  /*******************************************************************************/
  virtual void updateSensorAmbientTemperature(float period) {
    setSensorAmbientTemperaturePeriod(period);
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

  /*******************************************************************************/
  /*!
      @brief    Set the humidity sensor's return frequency.
      @param    period
                The time interval at which to return new data from the humidity
                sensor.
  */
  /*******************************************************************************/
  virtual void setSensorRelativeHumidityPeriod(float period) {
    if (period == 0.0) {
      disableSensorRelativeHumidity();
      return;
    }
    // Period is in seconds, cast it to long and convert it to milliseconds
    _humidSensorPeriod = (long)period * 1000;
  }

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

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a humidity sensor and converts
                the reading into the expected SI unit.
      @param    humidEvent
                Pointer to a humidity value.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventRelativeHumidity(float humidEvent) { return false; }

  /*******************************************************************************/
  /*!
      @brief    Updates the properties of a relative humidity sensor.
      @param    period
                The time interval at which to return new data from the humidity
                sensor.
  */
  /*******************************************************************************/
  virtual void updateSensorRelativeHumidity(float period) {
    setSensorRelativeHumidityPeriod(period);
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

  /*******************************************************************************/
  /*!
      @brief    Set the pressure sensor's return frequency.
      @param    period
                The time interval at which to return new data from the pressure
                sensor.
  */
  /*******************************************************************************/
  virtual void setSensorPressurePeriod(float period) {
    if (period == 0.0)
      disableSensorPressure();
    // Period is in seconds, cast it to long and convert it to milliseconds
    _pressureSensorPeriod = (long)period * 1000;
  }

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

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a pressure sensor and converts
                the reading into the expected SI unit.
      @param    pressureEvent
                A pressure value
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventPressure(float pressureEvent) { return false; }

  /*******************************************************************************/
  /*!
      @brief    Updates the properties of a pressure sensor.
      @param    period
                The time interval at which to return new data from the pressure
                sensor.
  */
  /*******************************************************************************/
  virtual void updateSensorPressure(float period) {
    setSensorPressurePeriod(period);
  }

  /**************************** SENSOR_TYPE: Gas
   * ****************************/
  /*******************************************************************************/
  /*!
      @brief    Enables the device's gas sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void enableSensorGas(){};

  /*******************************************************************************/
  /*!
      @brief    Disables the device's gas sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void disableSensorGas() { _gasSensorPeriod = 0.0L; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the gas sensor's period, if set.
      @returns  Time when the Gas sensor should be polled, in seconds.
  */
  /*********************************************************************************/
  virtual long sensorGasPeriod() { return _gasSensorPeriod; }

  /*******************************************************************************/
  /*!
      @brief    Set the gas sensor's return frequency.
      @param    period
                The time interval at which to return new data from the Gas
                sensor.
  */
  /*******************************************************************************/
  virtual void setSensorGasPeriod(float period) {
    if (period == 0.0)
      disableSensorGas();
    // Period is in seconds, cast it to long and convert it to milliseconds
    _gasSensorPeriod = (long)period * 1000;
  }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the previous time interval at
                    which the gas sensor was queried last.
      @returns  Time when the gas sensor was last queried, in seconds.
  */
  /*********************************************************************************/
  virtual long sensorGasPeriodPrv() { return _gasSensorPeriodPrv; }

  /*******************************************************************************/
  /*!
      @brief    Sets a timestamp for when the gas sensor was queried.
      @param    period
                The time when the gas sensor was queried last.
  */
  /*******************************************************************************/
  virtual void setSensorGasPeriodPrv(long period) {
    _gasSensorPeriodPrv = period;
  }

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a gas sensor and converts
                the reading into the expected SI unit.
      @param    gas_resistance
                Gas resistor (ohms) reading.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventGas(uint32_t gas_resistance) { return gas_resistance; }

  /*******************************************************************************/
  /*!
      @brief    Updates the properties of a gas sensor.
      @param    period
                The time interval at which to return new data from the Gas
                sensor.
  */
  /*******************************************************************************/
  virtual void updateSensorGas(float period) { setSensorGasPeriod(period); }

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

  /*******************************************************************************/
  /*!
      @brief    Set the Altitude sensor's return frequency.
      @param    period
                The time interval at which to return new data from the Altitude
                sensor.
  */
  /*******************************************************************************/
  virtual void setSensorAltitudePeriod(float period) {
    if (period == 0)
      disableSensorAltitude();
    // Period is in seconds, cast it to long and convert it to milliseconds
    _altitudeSensorPeriod = (long)period * 1000;
  }

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

  /*******************************************************************************/
  /*!
      @brief    Updates the properties of a Altitude sensor.
      @param    period
                The time interval at which to return new data from the altitude
                sensor.
  */
  /*******************************************************************************/
  virtual void updateSensorAltitude(float period) {
    setSensorAltitudePeriod(period);
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

  /*******************************************************************************/
  /*!
      @brief    Set the object temperature sensor's return frequency.
      @param    period
                The time interval at which to return new data from the
                object temperature sensor.
  */
  /*******************************************************************************/
  virtual void setSensorObjectTempPeriod(float period) {
    if (period == 0)
      disableSensorObjectTemp();
    // Period is in seconds, cast it to long and convert it to milliseconds
    _objectTempSensorPeriod = (long)period * 1000;
  }

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

  /*******************************************************************************/
  /*!
      @brief    Updates the properties of a object temperature sensor.
      @param    period
                The time interval at which to return new data from the
                object temperature sensor.
  */
  /*******************************************************************************/
  virtual void updateSensorObjectTemp(float period) {
    setSensorObjectTempPeriod(period);
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

  /*******************************************************************************/
  /*!
      @brief    Set the object light sensor's return frequency.
      @param    period
                The time interval at which to return new data from the
                object light sensor.
  */
  /*******************************************************************************/
  virtual void setSensorLightPeriod(float period) {
    if (period == 0)
      disableSensorLight();
    // Period is in seconds, cast it to long and convert it to milliseconds
    _lightSensorPeriod = (long)period * 1000;
  }

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

  /*******************************************************************************/
  /*!
      @brief    Updates the properties of a object light sensor.
      @param    period
                The time interval at which to return new data from the
                light sensor.
  */
  /*******************************************************************************/
  virtual void updateSensorLight(float period) { setSensorLightPeriod(period); }

  /********************** SENSOR_TYPE: PROXIMITY
   * ***********************/
  /*******************************************************************************/
  /*!
      @brief    Enables the device's proximity sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void enableSensorProximity() { return; };

  /*******************************************************************************/
  /*!
      @brief    Disables the device's proximity sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void disableSensorProximity() { _proximitySensorPeriod = 0.0L; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the proximity sensor's period, if
     set.
      @returns  Time when the proximity sensor should be polled, in seconds.
  */
  /*********************************************************************************/
  virtual long sensorProximityPeriod() { return _proximitySensorPeriod; }

  /*******************************************************************************/
  /*!
      @brief    Set the proximity sensor's return frequency.
      @param    period
                The time interval at which to return new data from the
     proximity sensor.
  */
  /*******************************************************************************/
  virtual void setSensorProximityPeriod(float period) {
    if (period == 0.0) {
      disableSensorProximity();
      return;
    }
    // Period is in seconds, cast it to long and convert it to milliseconds
    _proximitySensorPeriod = (long)period * 1000;
  }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the previous time interval at
     which the proximity sensor was queried last.
      @returns  Time when the proximity sensor was last queried, in seconds.
  */
  /*********************************************************************************/
  virtual long sensorProximityPeriodPrv() {
    return _proximitySensorPeriodPrv;
  }

  /*******************************************************************************/
  /*!
      @brief    Sets a timestamp for when the proximity sensor was queried.
      @param    periodPrv
                The time when the proximity sensor was queried last.
  */
  /*******************************************************************************/
  virtual void setSensorProximityPeriodPrv(long periodPrv) {
    _proximitySensorPeriodPrv = periodPrv;
  }

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a proximity sensor. Expects value
                to return in the proper SI unit.
      @param    proximityEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventProximity(sensors_event_t *proximityEvent) {
    return false;
  }

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a proximity sensor. Expects value
                to return in the proper SI unit.
      @param    proximityEvent
                Pointer to an proximity value.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventProximity(float proximityEvent) { return false; }

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Update the properties of a proximity
                  sensor, provided sensor_period.
      @param    period
                Sensor's period.
  */
  /*******************************************************************************/
  virtual void updateSensorProximity(float period) {
    setSensorProximityPeriod(period);
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
  long _gasSensorPeriod =
      0L; ///< The time period between reading the CO2 sensor's value.
  long _gasSensorPeriodPrv = 0L; ///< The time when the CO2 sensor
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
  long _proximitySensorPeriod = 0L;     ///< The time period between reading the
                                        ///< proximity sensor's value.
  long _proximitySensorPeriodPrv = 0L;   ///< The time when the proximity sensor
                                        ///< was last read.
};

#endif // WipperSnapper_I2C_Driver_H