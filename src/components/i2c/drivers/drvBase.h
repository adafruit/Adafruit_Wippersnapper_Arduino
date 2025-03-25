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

#define NO_MUX_CH 0xFFFF;          ///< No MUX channel specified
#define DEFAULT_SENSOR_PERIOD 30.0 ///< Default sensor period, in seconds

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
      @param    mux_channel
                An optional channel number used to address a device on a I2C
     MUX.
      @param    driver_name
                The name of the driver.
  */
  /*******************************************************************************/
  drvBase(TwoWire *i2c, uint16_t address, uint32_t mux_channel,
          const char *driver_name) {
    _i2c = i2c;
    _address = address;
    _i2c_mux_addr = 0x0;
    _i2c_mux_channel = mux_channel;
    strncpy(_name, driver_name, sizeof(_name) - 1);
    _name[sizeof(_name) - 1] = '\0';
    _has_alt_i2c_bus = false;
    strcpy(_pin_scl, "default");
    strcpy(_pin_sda, "default");
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an I2C sensor.
  */
  /*******************************************************************************/
  virtual ~drvBase() {}

  /*******************************************************************************/
  /*!
      @brief    Gets the name of the driver.
      @returns  The driver's name.
  */
  /*******************************************************************************/
  const char *GetDrvName() { return _name; }

  /*******************************************************************************/
  /*!
      @brief    Gets the I2C device's address.
      @returns  The I2C device's unique i2c address.
  */
  /*******************************************************************************/
  uint16_t GetAddress() { return _address; }

  /*******************************************************************************/
  /*!
      @brief    Gets the I2C MUX address.
      @returns  The I2C MUX address.
  */
  /*******************************************************************************/
  uint32_t GetMuxAddress() { return _i2c_mux_addr; }

  /*******************************************************************************/
  /*!
      @brief    Sets the I2C MUX address.
      @param    mux_address
                The I2C MUX address.
  */
  /*******************************************************************************/
  void SetMuxAddress(uint32_t mux_address) { _i2c_mux_addr = mux_address; }

  /*******************************************************************************/
  /*!
      @brief    Set if the I2C driver has an alternative I2C bus.
      @param   scl_pin
                The SCL pin for the alternative I2C bus.
      @param   sda_pin
                The SDA pin for the alternative I2C bus.
  */
  /*******************************************************************************/
  void EnableAltI2CBus(char *scl_pin, char *sda_pin) {
    strcpy(_pin_scl, scl_pin);
    strcpy(_pin_sda, sda_pin);
    _has_alt_i2c_bus = true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SCL pin for the alternative I2C bus.
      @returns  The SCL pin for the alternative I2C bus.
  */
  /*******************************************************************************/
  const char *GetPinSCL() { return _pin_scl; }

  /*******************************************************************************/
  /*!
      @brief    Gets the SDA pin for the alternative I2C bus.
      @returns  The SDA pin for the alternative I2C bus.
  */
  /*******************************************************************************/
  const char *GetPinSDA() { return _pin_sda; }

  /*******************************************************************************/
  /*!
      @brief    Checks if the I2C driver uses an alternative I2C bus.
      @returns  True if the I2C driver uses an alternative I2C bus, False
                otherwise.
  */
  /*******************************************************************************/
  bool HasAltI2CBus() { return _has_alt_i2c_bus; }

  /*******************************************************************************/
  /*!
      @brief    Gets the I2C MUX channel connected to the I2C device.
      @returns  The desired MUX channel.
  */
  /*******************************************************************************/
  uint32_t GetMuxChannel() { return _i2c_mux_channel; }

  /*******************************************************************************/
  /*!
      @brief    Checks if the I2C driver is attached to an I2C MUX.
      @returns  True if the I2C driver uses an I2C MUX, False otherwise.
  */
  /*******************************************************************************/
  bool HasMux() { return _i2c_mux_channel != NO_MUX_CH; }

  /*******************************************************************************/
  /*!
      @brief    Configures an i2c device's sensors.
      @param    sensor_types
                Pointer to an array of SensorType objects.
      @param    sensor_types_count
                The number of active sensors to read from the device.
  */
  /*******************************************************************************/
  void SetSensorTypes(bool use_default_types = false,
                      wippersnapper_sensor_SensorType *sensor_types = nullptr,
                      size_t sensor_types_count = 0) {
    if (use_default_types) {
      // set sensor_types_count to # of elements within _default_sensor_types
      sensor_types_count =
          sizeof(_default_sensor_types) / sizeof(_default_sensor_types[0]);
    }
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
  void SetPeriod(float period = DEFAULT_SENSOR_PERIOD) {
    if (period < 0)
      _sensor_period = DEFAULT_SENSOR_PERIOD;

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
      @brief    Gets the sensor's types
      @returns  A pointer to an array of SensorTypes.
  */
  /*******************************************************************************/
  wippersnapper_sensor_SensorType *GetSensorTypes() { return _sensors; }

  /*******************************************************************************/
  /*!
      @brief    Gets the sensor's previous period.
      @returns  The sensor's previous period, in milliseconds.
  */
  /*******************************************************************************/
  ulong GetSensorPeriodPrv() { return _sensor_period_prv; }

  /*******************************************************************************/
  /*!
      @brief    Gets a sensor's CO2 value.
      @param    co2Event
                The CO2 value, in ppm.
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventCO2(sensors_event_t *co2Event) { return false; };

  /*******************************************************************************/
  /*!
      @brief    Gets a sensor's eCO2 value.
      @param    eco2Event
                The equivalent CO2 value, in ppm.
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventECO2(sensors_event_t *eco2Event) { return false; }

  /*******************************************************************************/
  /*!
      @brief    Gets a sensor's TVOC value.
      @param    tvocEvent
                The Total Volatile Organic Compounds value, in ppb.
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventTVOC(sensors_event_t *tvocEvent) { return false; }

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
  virtual bool getEventAmbientTemp(sensors_event_t *tempEvent) { return false; }

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
      @brief    Base implementation - Selects a MUX channel for use with the
                I2C device.
      @param    channel
                The MUX channel to select.
  */
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
  virtual bool getEventLight(sensors_event_t *lightEvent) { return false; }

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

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a object pm10 env. sensor and
                converts the reading into the expected SI unit.
      @param    pm10EnvEvent
                pm10 env. sensor reading, in ppm.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventPM10_Env(sensors_event_t *pm10EnvEvent) { return false; }

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

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a object pm25 env. sensor and
                converts the reading into the expected SI unit.
      @param    pm25EnvEvent
                pm25 env. sensor reading, in ppm.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventPM25_Env(sensors_event_t *pm25EnvEvent) { return false; }

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

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a object pm100 env. sensor and
                converts the reading into the expected SI unit.
      @param    pm100EnvEvent
                pm100 env. sensor reading, in ppm.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventPM100_Env(sensors_event_t *pm100EnvEvent) {
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
  virtual bool getEventUnitlessPercent(sensors_event_t *unitlessPercentEvent) {
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
  virtual bool getEventVoltage(sensors_event_t *voltageEvent) { return false; }

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
  virtual bool getEventCurrent(sensors_event_t *currentEvent) { return false; }

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
    if (!getEventAmbientTemp(AmbientTempFEvent)) {
      return false;
    }
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
  virtual bool getEventGasResistance(sensors_event_t *gasEvent) {
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
  virtual bool getEventNOxIndex(sensors_event_t *gasEvent) { return false; }

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
  virtual bool getEventVOCIndex(sensors_event_t *gasEvent) { return false; }

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
    return false;
  }

  /*******************************************************************************/
  /*!
      @brief   Reads a sensor's event from the i2c driver.
      @param   sensor_type
                The sensor type to read.
      @param    sensors_event
                Pointer to an Adafruit_Sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool GetSensorEvent(wippersnapper_sensor_SensorType sensor_type,
                      sensors_event_t *sensors_event) {
    auto it = SensorEventHandlers.find(sensor_type);
    if (it == SensorEventHandlers.end())
      return false; // Could not find sensor_type
    return it->second(sensors_event);
  }

  /*******************************************************************************/
  /*!
      @brief    Function type for sensor event handlers
      @param    sensors_event_t*
                Pointer to the sensor event structure to be filled
      @returns  True if event was successfully read, False otherwise
  */
  /*******************************************************************************/
  using fnGetEvent = std::function<bool(sensors_event_t *)>;

  // Maps SensorType to function calls
  std::map<wippersnapper_sensor_SensorType, fnGetEvent> SensorEventHandlers = {
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_UNSPECIFIED,
       [this](sensors_event_t *event) -> bool {
         return this->getEventRaw(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_LIGHT,
       [this](sensors_event_t *event) -> bool {
         return this->getEventLight(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_PRESSURE,
       [this](sensors_event_t *event) -> bool {
         return this->getEventPressure(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_PROXIMITY,
       [this](sensors_event_t *event) -> bool {
         return this->getEventProximity(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_RELATIVE_HUMIDITY,
       [this](sensors_event_t *event) -> bool {
         return this->getEventRelativeHumidity(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE,
       [this](sensors_event_t *event) -> bool {
         return this->getEventAmbientTemp(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_OBJECT_TEMPERATURE,
       [this](sensors_event_t *event) -> bool {
         return this->getEventObjectTemp(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_VOLTAGE,
       [this](sensors_event_t *event) -> bool {
         return this->getEventVoltage(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_CURRENT,
       [this](sensors_event_t *event) -> bool {
         return this->getEventCurrent(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_RAW,
       [this](sensors_event_t *event) -> bool {
         return this->getEventRaw(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_PM10_STD,
       [this](sensors_event_t *event) -> bool {
         return this->getEventPM10_STD(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_PM25_STD,
       [this](sensors_event_t *event) -> bool {
         return this->getEventPM25_STD(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_PM100_STD,
       [this](sensors_event_t *event) -> bool {
         return this->getEventPM100_STD(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_PM10_ENV,
       [this](sensors_event_t *event) -> bool {
         return this->getEventPM10_Env(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_PM25_ENV,
       [this](sensors_event_t *event) -> bool {
         return this->getEventPM25_Env(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_PM100_ENV,
       [this](sensors_event_t *event) -> bool {
         return this->getEventPM100_Env(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_CO2,
       [this](sensors_event_t *event) -> bool {
         return this->getEventCO2(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_GAS_RESISTANCE,
       [this](sensors_event_t *event) -> bool {
         return this->getEventGasResistance(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_ALTITUDE,
       [this](sensors_event_t *event) -> bool {
         return this->getEventAltitude(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_ECO2,
       [this](sensors_event_t *event) -> bool {
         return this->getEventECO2(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_UNITLESS_PERCENT,
       [this](sensors_event_t *event) -> bool {
         return this->getEventUnitlessPercent(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE_FAHRENHEIT,
       [this](sensors_event_t *event) -> bool {
         return this->getEventAmbientTempF(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_OBJECT_TEMPERATURE_FAHRENHEIT,
       [this](sensors_event_t *event) -> bool {
         return this->getEventObjectTempF(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_VOC_INDEX,
       [this](sensors_event_t *event) -> bool {
         return this->getEventVOCIndex(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_NOX_INDEX,
       [this](sensors_event_t *event) -> bool {
         return this->getEventNOxIndex(event);
       }},
      {wippersnapper_sensor_SensorType_SENSOR_TYPE_TVOC,
       [this](sensors_event_t *event) -> bool {
         return this->getEventTVOC(event);
       }}}; ///< SensorType to function call map

  wippersnapper_sensor_SensorType
      _sensors[15]; ///< Sensors attached to the device.

protected:
  TwoWire *_i2c;             ///< Pointer to the I2C bus
  bool _has_alt_i2c_bus;     ///< True if the device is on an alternate I2C bus
  uint16_t _address;         ///< The device's I2C address.
  uint32_t _i2c_mux_addr;    ///< The I2C MUX address, if applicable.
  uint32_t _i2c_mux_channel; ///< The I2C MUX channel, if applicable.
  char _name[15];            ///< The device's name.
  char _pin_scl[8];          ///< The device's SCL pin.
  char _pin_sda[8];          ///< The device's SDA pin.
  ulong _sensor_period;      ///< The sensor's period, in milliseconds.
  ulong _sensor_period_prv;  ///< The sensor's previous period, in milliseconds.
  size_t _sensors_count;     ///< Number of sensors on the device.
  wippersnapper_sensor_SensorType
      _default_sensor_types[1]; ///< Default sensor types
};
#endif // DRV_BASE_H