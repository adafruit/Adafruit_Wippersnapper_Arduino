/*!
 * @file drvBase.h
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

#ifndef DRV_BASE_H
#define DRV_BASE_H

#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <protos/i2c.pb.h>

#define NO_MUX_CH 0xFFFF; ///< No MUX channel specified

/**************************************************************************/
/*!
    @brief  Base class for I2C Drivers.
*/
/**************************************************************************/
class drvBase {

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
  drvBase(TwoWire *i2c, uint16_t address, const char *driver_name) {
    _i2c = i2c;
    _address = address;
    _i2c_mux_channel = NO_MUX_CH;
    strncpy(_name, driver_name, sizeof(_name) - 1);
    _name[sizeof(_name) - 1] = '\0';
    _has_alt_i2c_bus = false;
  }

  /*******************************************************************************/
  /*!
      @brief    Instanciates an I2C sensor.
      @param    i2c
                The I2C hardware interface, default is Wire.
      @param    address
                The I2C sensor's unique address.
      @param    mux_channel
                An optional channel number used to address a device on a I2C
     MUX.
  */
  /*******************************************************************************/
  drvBase(TwoWire *i2c, uint16_t address, uint32_t mux_channel,
          const char *driver_name) {
    _i2c = i2c;
    _address = address;
    _i2c_mux_channel = mux_channel;
    strncpy(_name, driver_name, sizeof(_name) - 1);
    _name[sizeof(_name) - 1] = '\0';
    _has_alt_i2c_bus = false;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an I2C sensor.
  */
  /*******************************************************************************/
  virtual ~drvBase() {}

  void EnableAltI2CBus() { _has_alt_i2c_bus = true; }
  bool HasAltI2CBus() { return _has_alt_i2c_bus; }

  uint32_t GetMuxChannel() { return _i2c_mux_channel; }

  const char *GetDrvName() { return _name; }

  /*******************************************************************************/
  /*!
      @brief    Configures an i2c device's sensors.
      @param    sensors
                Pointer to an array of SensorType objects.
      @param    count
                The number of active sensors on the device.
  */
  /*******************************************************************************/
  void EnableSensorReads(wippersnapper_sensor_SensorType *sensor_types,
                         size_t sensor_types_count) {
    _sensors_count = sensor_types_count;
    for (size_t i = 0; i < _sensors_count; i++) {
      _sensors[i] = sensor_types[i];
    }
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the number of enabled sensors.
      @returns  The number of enabled sensors.
  */
  /*******************************************************************************/
  size_t GetEnabledSensorCnt() { return _sensors_count; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the I2C sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  virtual bool begin() = 0;

  /*******************************************************************************/
  /*!
      @brief    Sets the sensor's period and converts from seconds to
                milliseconds.
      @param    period The period for the sensor to return values within, in
                seconds.
  */
  /*******************************************************************************/
  void SetSensorPeriod(float period) {
    if (period < 0) {
      _sensor_period = 0;
      return;
    }
    _sensor_period = (unsigned long)(period * 1000.0f);
  }

  /*******************************************************************************/
  /*!
      @brief    Sets the sensor's previous period and converts from seconds
                to milliseconds.
      @param    period The period for the sensor to return values within, in
                seconds.
  */
  /*******************************************************************************/
  void SetSensorPeriodPrv(ulong period) { _sensor_period_prv = period; }

  /*******************************************************************************/
  /*!
      @brief    Gets the sensor's period.
      @returns  The sensor's period, in milliseconds.
  */
  /*******************************************************************************/
  ulong GetSensorPeriod() { return _sensor_period; }

  /*******************************************************************************/
  /*!
      @brief    Gets the sensor's previous period.
      @returns  The sensor's previous period, in milliseconds.
  */
  /*******************************************************************************/
  ulong GetSensorPeriodPrv() { return _sensor_period_prv; }

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
  virtual bool GetEventCO2(sensors_event_t *co2Event) { return false; };

  /*******************************************************************************/
  /*!
      @brief    Gets a sensor's eCO2 value.
      @param    eco2Event
                The equivalent CO2 value, in ppm.
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool GetEventECO2(sensors_event_t *eco2Event) { return false; }

  /*******************************************************************************/
  /*!
      @brief    Gets a sensor's TVOC value.
      @param    tvocEvent
                The Total Volatile Organic Compounds value, in ppb.
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool GetEventTVOC(sensors_event_t *tvocEvent) { return false; }

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
  virtual bool GetEventAmbientTemp(sensors_event_t *tempEvent) { return false; }

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
  virtual bool GetEventRelativeHumidity(sensors_event_t *humidEvent) {
    return false;
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
  virtual bool GetEventPressure(sensors_event_t *pressureEvent) {
    return false;
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
  virtual bool GetEventAltitude(sensors_event_t *altitudeEvent) {
    return false;
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
  virtual bool GetEventObjectTemp(sensors_event_t *objectTempEvent) {
    return false;
  }

  virtual void SelectMUXChannel(uint8_t channel) { return; }

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
  virtual bool GetEventLight(sensors_event_t *lightEvent) { return false; }

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
  virtual bool GetEventPM10_STD(sensors_event_t *pm10StdEvent) { return false; }

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
  virtual bool GetEventPM25_STD(sensors_event_t *pm25StdEvent) { return false; }

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
  virtual bool GetEventPM100_STD(sensors_event_t *pm100StdEvent) {
    return false;
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
  virtual bool GetEventUnitlessPercent(sensors_event_t *unitlessPercentEvent) {
    return false;
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
  virtual bool GetEventVoltage(sensors_event_t *voltageEvent) { return false; }

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
  virtual bool GetEventCurrent(sensors_event_t *currentEvent) { return false; }

  /*******************************************************************************/
  /*!
      @brief    Gets a sensor's Raw value.
      @param    rawEvent
                The Raw value.
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool GetEventRaw(sensors_event_t *rawEvent) { return false; }

  /*******************************************************************************/
  /*!
      @brief    Helper function to obtain a sensor's ambient temperature value
                in °F. Requires `GetEventAmbientTemp()` to be fully
                implemented by a driver.
      @param    AmbientTempFEvent
                The ambient temperature value, in °F.
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool GetEventAmbientTempF(sensors_event_t *AmbientTempFEvent) {
    // obtain ambient temp. in °C
    if (!GetEventAmbientTemp(AmbientTempFEvent))
      return false;
    // convert event from °C to °F
    AmbientTempFEvent->temperature =
        (AmbientTempFEvent->temperature * 9.0) / 5.0 + 32;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Helper function to obtain a sensor's object temperature value
                in °F. Requires `GetEventObjectTemp()` to be fully
                implemented by a driver.
      @param    objectTempFEvent
                The object temperature value, in °F.
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool GetEventObjectTempF(sensors_event_t *objectTempFEvent) {
    // obtain ambient temp. in °C
    if (!GetEventObjectTemp(objectTempFEvent))
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
  virtual bool GetEventGasResistance(sensors_event_t *gasEvent) {
    return false;
  }

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
  virtual bool GetEventNOxIndex(sensors_event_t *gasEvent) { return false; }

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
  virtual bool GetEventVOCIndex(sensors_event_t *gasEvent) { return false; }

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
  virtual bool GetEventProximity(sensors_event_t *proximityEvent) {
    return false;
  }

  bool GetSensorEvent(wippersnapper_sensor_SensorType sensor_type,
                      sensors_event_t *sensors_event) {
    auto it = SensorEventHandlers.find(sensor_type);
    if (it == SensorEventHandlers.end())
      return false; // Could not find sensor_type
    return it->second(sensors_event);
  }

  // private:
  //  Lambda function type for all GetEventX() function calls
  using fnGetEvent = std::function<bool(sensors_event_t *)>;

  // Maps SensorType to function calls
  std::map<wippersnapper_sensor_SensorType, fnGetEvent> SensorEventHandlers = {
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE,
       [this](sensors_event_t *event) -> bool {
         return this->GetEventAmbientTemp(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE_FAHRENHEIT,
       [this](sensors_event_t *event) -> bool {
         return this->GetEventAmbientTempF(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_PRESSURE,
       [this](sensors_event_t *event) -> bool {
         return this->GetEventPressure(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_RELATIVE_HUMIDITY,
       [this](sensors_event_t *event) -> bool {
         return this->GetEventRelativeHumidity(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_ALTITUDE,
       [this](sensors_event_t *event) -> bool {
         return this->GetEventAltitude(event);
       }},
  };

  wippersnapper_sensor_SensorType
      _sensors[15]; ///< Sensors attached to the device.

protected:
  TwoWire *_i2c;             ///< Pointer to the I2C bus
  bool _has_alt_i2c_bus;     ///< True if the device is on an alternate I2C bus
  uint16_t _address;         ///< The device's I2C address.
  uint32_t _i2c_mux_channel; ///< The I2C MUX channel, if applicable.
  char _name[15];            ///< The device's name.
  ulong _sensor_period;      ///< The sensor's period, in milliseconds.
  ulong _sensor_period_prv;  ///< The sensor's previous period, in milliseconds.

  size_t _sensors_count; ///< Number of sensors on the device.
};
#endif // DRV_BASE_H