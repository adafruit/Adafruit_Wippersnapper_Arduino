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
  virtual ~WipperSnapper_I2C_Driver() {}

  /*******************************************************************************/
  /*!
      @brief    Initializes the I2C sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  // NOTE: We changed this to virtual so drivers must now reflect: bool begin() override{} 
  virtual bool begin() { return false; }

  /*******************************************************************************/
  /*!
      @brief    Sets the sensor's period, provided a
     wippersnapper_i2c_v1_SensorType.
      @param    period The period for the sensor to return values within, in
     seconds.
      @param    sensorType The type of sensor device.
  */
  /*******************************************************************************/
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
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_ECO2:
      _ECO2SensorPeriod = sensorPeriod;
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_TVOC:
      _TVOCSensorPeriod = sensorPeriod;
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
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_CURRENT:
      _currentPeriod = sensorPeriod;
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PROXIMITY:
      _proximitySensorPeriod = sensorPeriod;
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_RAW:
      _rawSensorPeriod = sensorPeriod;
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE_FAHRENHEIT:
      _ambientTempFPeriod = sensorPeriod;
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_OBJECT_TEMPERATURE_FAHRENHEIT:
      _objectTempFPeriod = sensorPeriod;
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_GAS_RESISTANCE:
      _gasResistancePeriod = sensorPeriod;
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_NOX_INDEX:
      _NOxIndexPeriod = sensorPeriod;
      break;
    case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_VOC_INDEX:
      _VOCIndexPeriod = sensorPeriod;
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
      @brief    Gets a sensor's CO2 value.
      @param    co2Event
                The CO2 value, in ppm.
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventCO2(sensors_event_t *co2Event) {
    (void)
        co2Event; // Parameter is intentionally unused in this virtual function.
    return false;
  }

  /****************************** SENSOR_TYPE: ECO2
   * *******************************/

  /*******************************************************************************/
  /*!
      @brief    Gets a sensor's eCO2 value.
      @param    eco2Event
                The equivalent CO2 value, in ppm.
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventECO2(sensors_event_t *eco2Event) {
    (void)eco2Event; // Parameter is intentionally unused in this virtual
                     // function.
    return false;
  }

  /****************************** SENSOR_TYPE: TVOC
   * *******************************/

  /*******************************************************************************/
  /*!
      @brief    Gets a sensor's TVOC value.
      @param    tvocEvent
                The Total Volatile Organic Compounds value, in ppb.
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventTVOC(sensors_event_t *tvocEvent) {
    (void)tvocEvent; // Parameter is intentionally unused in this virtual
                     // function.
    return false;
  }

  /********************** SENSOR_TYPE: AMBIENT TEMPERATURE (°C)
   * ***********************/

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
  virtual bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    (void)tempEvent; // Parameter is intentionally unused in this virtual
                     // function.
    return false;
  }

  /************************* SENSOR_TYPE: RELATIVE_HUMIDITY
   * ***********************/

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
    (void)humidEvent; // Parameter is intentionally unused in this virtual
                      // function.
    return false;
  }

  /**************************** SENSOR_TYPE: PRESSURE
   * ****************************/

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
    (void)pressureEvent; // Parameter is intentionally unused in this virtual
                         // function.
    return false;
  }

  /**************************** SENSOR_TYPE: Altitude
   * ****************************/

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
    (void)altitudeEvent; // Parameter is intentionally unused in this virtual
                         // function.
    return false;
  }

  /**************************** SENSOR_TYPE: Object_Temperature
   * ****************************/

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
    (void)objectTempEvent; // Parameter is intentionally unused in this virtual
                           // function.
    return false;
  }

  /**************************** SENSOR_TYPE: LIGHT
   * ****************************/

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
  virtual bool getEventLight(sensors_event_t *lightEvent) {
    (void)lightEvent; // Parameter is intentionally unused in this virtual
                      // function.
    return false;
  }

  /**************************** SENSOR_TYPE: PM10_STD
   * ****************************/

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
  virtual bool getEventPM10_STD(sensors_event_t *pm10StdEvent) {
    (void)pm10StdEvent; // Parameter is intentionally unused in this virtual
                        // function.
    return false;
  }

  /**************************** SENSOR_TYPE: PM25_STD
   * ****************************/

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
  virtual bool getEventPM25_STD(sensors_event_t *pm25StdEvent) {
    (void)pm25StdEvent; // Parameter is intentionally unused in this virtual
                        // function.
    return false;
  }

  /**************************** SENSOR_TYPE: PM100_STD
   * ****************************/

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
    (void)pm100StdEvent; // Parameter is intentionally unused in this virtual
                         // function.
    return false;
  }

  /**************************** SENSOR_TYPE: UNITLESS_PERCENT
   * ****************************/

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
    (void)unitlessPercentEvent; // Parameter is intentionally unused in this
                                // virtual function.
    return false;
  }

  /**************************** SENSOR_TYPE: VOLTAGE
   * ****************************/

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
  virtual bool getEventVoltage(sensors_event_t *voltageEvent) {
    (void)voltageEvent; // Parameter is intentionally unused in this virtual
                        // function.
    return false;
  }

  /**************************** SENSOR_TYPE: CURRENT
   * ****************************/

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
  virtual bool getEventCurrent(sensors_event_t *currentEvent) {
    (void)currentEvent; // Parameter is intentionally unused in this virtual
                        // function.
    return false;
  }

  /****************************** SENSOR_TYPE: Raw
   * *******************************/

  /*******************************************************************************/
  /*!
      @brief    Gets a sensor's Raw value.
      @param    rawEvent
                The Raw value.
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventRaw(sensors_event_t *rawEvent) {
    (void)
        rawEvent; // Parameter is intentionally unused in this virtual function.
    return false;
  }

  /****************************** SENSOR_TYPE: Ambient Temp (°F)
   * *******************************/

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

  /****************************** SENSOR_TYPE: Object Temp (°F)
   * *******************************/

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
  virtual bool getEventGasResistance(sensors_event_t *gasEvent) {
    (void)
        gasEvent; // Parameter is intentionally unused in this virtual function.
    return false;
  }

  /****************************** SENSOR_TYPE: NOx Index (index)
   * *******************************/

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
  virtual bool getEventNOxIndex(sensors_event_t *gasEvent) {
    (void)
        gasEvent; // Parameter is intentionally unused in this virtual function.
    return false;
  }

  /****************************** SENSOR_TYPE: VOC Index (index)
   * *******************************/

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
  virtual bool getEventVOCIndex(sensors_event_t *gasEvent) {
    (void)
        gasEvent; // Parameter is intentionally unused in this virtual function.
    return false;
  }

  /**************************** SENSOR_TYPE: PROXIMITY
   * ****************************/

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
  virtual bool getEventProximity(sensors_event_t *proximityEvent) {
    (void)proximityEvent; // Parameter is intentionally unused in this virtual
                          // function.
    return false;
  }

protected:
  TwoWire *_i2c;           ///< Pointer to the I2C driver's Wire object
  uint16_t _sensorAddress; ///< The I2C driver's unique I2C address.
};

#endif // WipperSnapper_I2C_Driver_H
