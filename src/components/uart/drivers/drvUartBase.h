/*!
 * @file drvUartBase.h
 *
 * Base implementation for UART device drivers.
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

#ifndef DRV_UART_BASE_H
#define DRV_UART_BASE_H
#include "../hardware.h"
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <protos/sensor.pb.h>
#include <protos/uart.pb.h>

/*!
    @brief  Base class for UART Drivers.
*/
class drvUartBase {

public:
  /*!
      @brief    Instantiates a UART device.
      @param    HardwareSerial
                Pointer to a HardwareSerial instance.
      @param    driver_name
                The name of the driver.
      @param    port_num
                The port number for the UART device corresponding to the Serial
     instance.
  */
  drvUartBase(HardwareSerial *hw_serial, const char *driver_name,
              uint32_t port_num) {
    _hw_serial = hw_serial;
    _is_software_serial = false;
    strncpy(_name, driver_name, sizeof(_name) - 1);
    _name[sizeof(_name) - 1] = '\0';
    _port_num = port_num;
  }

#if HAS_SW_SERIAL
  /*!
      @brief    Instantiates a UART device.
      @param    SoftwareSerial
                Pointer to a SoftwareSerial instance.
      @param    driver_name
                The name of the driver.
  */
  drvUartBase(SoftwareSerial *sw_serial,
              wippersnapper_uart_UartDeviceType device_type,
              const char *driver_name) {
    _sw_serial = sw_serial;
    _is_software_serial = true;
    strncpy(_name, driver_name, sizeof(_name) - 1);
    _name[sizeof(_name) - 1] = '\0';
  }
#endif // HAS_SW_SERIAL

  /*!
      @brief    Destructor for a UART device.
  */
  virtual ~drvUartBase() {
    // This is a base class, nothing to clean up!
  }

  /*!
      @brief    Configures the UART driver with device-specific settings.
      @param    cfg_device
                The configuration settings for the UART device.
  */
  void ConfigureDriver(wippersnapper_uart_UartDeviceConfig &cfg_device) {
    _device_type = cfg_device.device_type;

    switch (_device_type) {
    case wippersnapper_uart_UartDeviceType_UART_DEVICE_TYPE_GENERIC_INPUT:
      // Handle Generic Input device configuration
      // TODO!
      break;
    case wippersnapper_uart_UartDeviceType_UART_DEVICE_TYPE_GENERIC_OUTPUT:
      // Handle Generic Output device configuration
      // TODO!
      break;
    case wippersnapper_uart_UartDeviceType_UART_DEVICE_TYPE_GPS:
      // Handle GPS device configuration
      // TODO!
      break;
    case wippersnapper_uart_UartDeviceType_UART_DEVICE_TYPE_PM25AQI:
      // Handle PM2.5 AQI device configuration
      WS_DEBUG_PRINTLN("[uart] Configuring PM2.5 AQI device...");
      break;
    case wippersnapper_uart_UartDeviceType_UART_DEVICE_TYPE_TM22XX:
      // Handle TM22XX device configuration
      // TODO!
      break;
    default:
      WS_DEBUG_PRINTLN("[uart] ERROR: Unknown device type!");
      return; // bail if the device type is unknown
    }
  }

  /*!
      @brief    Gets the UART device type.
      @returns  The UART device type.
  */
  wippersnapper_uart_UartDeviceType GetDeviceType() { return _device_type; }

  /*!
      @brief    Gets the name of the UART driver.
      @returns  The name of the UART driver.
  */
  uint32_t GetPortNum() const { return _port_num; }

  /*!
      @brief    Gets the name of the UART driver.
      @returns  The name of the UART driver.
  */
  const char *GetName() const { return _name; }

  /*!
      @brief    Gets the type of Serial used by this driver.
      @returns  True if this driver uses SoftwareSerial, False if it uses
     HardwareSerial.
  */
  bool IsSoftwareSerial() const { return _is_software_serial; }

  /*!
      @brief    Initializes the UART device.
      @returns  True if initialized successfully, False otherwise.
  */
  virtual bool begin() = 0;

  /*!
      @brief    Configures an UART-input device's sensors for reading.
      @param    sensor_types
                Pointer to an array of SensorType objects.
      @param    sensor_types_count
                The number of active sensors to read from the device.
  */
  void EnableSensorEvents(wippersnapper_sensor_SensorType *sensor_types,
                          size_t sensor_types_count) {
    _sensors_count = sensor_types_count;
    for (size_t i = 0; i < _sensors_count; i++) {
      _sensors[i] = sensor_types[i];
    }
  }

  /*!
    @brief    Gets the number of enabled sensors.
    @returns  The number of enabled sensors.
  */
  size_t GetNumSensors() { return _sensors_count; }

  /*!
      @brief    Sets the sensor's period and converts from seconds to
                milliseconds.
      @param    period The period for the sensor to return values within, in
                seconds.
  */
  void SetSensorPeriod(float period) {
    if (period < 0) {
      _sensor_period = 0;
      return;
    }
    _sensor_period = (unsigned long)(period * 1000.0f);
  }

  /*!
      @brief    Sets the sensor's previous period and converts from seconds
                to milliseconds.
      @param    period The period for the sensor to return values within, in
                seconds.
  */
  void SetSensorPeriodPrv(ulong period) { _sensor_period_prv = period; }

  /*!
      @brief    Gets the sensor's period.
      @returns  The sensor's period, in milliseconds.
  */
  ulong GetSensorPeriod() { return _sensor_period; }

  /*!
      @brief    Gets the sensor's previous period.
      @returns  The sensor's previous period, in milliseconds.
  */
  ulong GetSensorPeriodPrv() { return _sensor_period_prv; }

  /*!
      @brief   Reads a sensor's event from the i2c driver.
      @param   sensor_type
                The sensor type to read.
      @param    sensors_event
                Pointer to an Adafruit_Sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  bool GetSensorEvent(wippersnapper_sensor_SensorType sensor_type,
                      sensors_event_t *sensors_event) {
    auto it = SensorEventHandlers.find(sensor_type);
    if (it == SensorEventHandlers.end())
      return false; // Could not find sensor_type
    return it->second(sensors_event);
  }

  /*!
      @brief    Base implementation - Reads a object pm10 std. sensor and
                converts the reading into the expected SI unit.
      @param    pm10StdEvent
                pm10 std. sensor reading, in ppm.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  virtual bool getEventPM10_STD(sensors_event_t *pm10StdEvent) { return false; }

  /*!
      @brief    Base implementation - Reads a object pm25 std. sensor and
                converts the reading into the expected SI unit.
      @param    pm25StdEvent
                pm25 std. sensor reading, in ppm.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  virtual bool getEventPM25_STD(sensors_event_t *pm25StdEvent) { return false; }

  /*!
      @brief    Base implementation - Reads a object pm100 std. sensor and
                converts the reading into the expected SI unit.
      @param    pm100StdEvent
                pm100 std. sensor reading, in ppm.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  virtual bool getEventPM100_STD(sensors_event_t *pm100StdEvent) {
    return false;
  }

  /*!
      @brief    Function type for sensor event handlers
      @param    sensors_event_t*
                Pointer to the sensor event structure to be filled
      @returns  True if event was successfully read, False otherwise
  */
  using fnGetEvent = std::function<bool(sensors_event_t *)>;

  // Maps SensorType to function calls
  std::map<wippersnapper_sensor_SensorType, fnGetEvent> SensorEventHandlers = {
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
       }}}; ///< SensorType to function call map

  wippersnapper_sensor_SensorType
      _sensors[15]; ///< The sensors attached to the device.
protected:
  HardwareSerial *_hw_serial; ///< Pointer to a HardwareSerial instance
#if HAS_SW_SERIAL
  SoftwareSerial *_sw_serial; ///< Pointer to a SoftwareSerial instance
#endif                        // HAS_SW_SERIAL
  wippersnapper_uart_UartDeviceType _device_type; ///< The UART device type
  char _name[15];                                 ///< The device's name
  uint32_t _port_num = 0; ///< The port number for the UART device
  bool _is_software_serial =
      false; ///< Indicates if this driver uses SoftwareSerial
  // Sensor API - UART INPUT
  // TODO: This is OK here for testing but should be moved to a base class and
  // refactored for drvBase and drvUartBase

  size_t _sensors_count;    ///< Number of sensors on the device
  ulong _sensor_period;     ///< The sensor's period, in milliseconds.
  ulong _sensor_period_prv; ///< The sensor's previous period, in milliseconds.
};
#endif // DRV_UART_BASE_H