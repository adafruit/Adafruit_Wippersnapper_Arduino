/*!
 * @file drvUartPm25.h
 *
 * Interface for the Adafruit_PM25AQI UART driver.
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

#ifndef DRV_UART_PM25_H
#define DRV_UART_PM25_H
#include "Adafruit_PM25AQI.h"
#include "drvUartBase.h"

/*!
    @brief  Provides an interface for the Adafruit_PM25AQI library over
            UART.
*/
class drvUartPm25 : public drvUartBase {

public:
  /*!
      @brief    Instantiates a UART device.
      @param    hw_serial
                Pointer to a HardwareSerial instance.
      @param    driver_name
                The name of the driver.
      @param    port_name
                The name of the RX pin for the UART device corresponding to
     the Serial instance.
  */
  drvUartPm25(HardwareSerial *hw_serial, const char *driver_name,
              const char *port_name)
      : drvUartBase(hw_serial, driver_name, port_name) {
    // Handled by drvUartBase constructor
  }

#if HAS_SW_SERIAL
  /*!
    @brief    Instantiates a UART device.
    @param    sw_serial
              Pointer to a SoftwareSerial instance.
    @param    driver_name
              The name of the driver.
    @param   port_name
              The name of the RX pin for the UART device corresponding to
    the Serial instance.
*/
  drvUartPm25(SoftwareSerial *sw_serial, const char *driver_name,
              const char *port_name)
      : drvUartBase(sw_serial, driver_name, port_name) {
    // Handled by drvUartBase constructor
  }
#endif // HAS_SW_SERIAL

  /*!
      @brief    Destructor for a UART device.
  */
  ~drvUartPm25() {
    if (_pm25) {
      delete _pm25; // Clean up the Adafruit_PM25AQI instance
      _pm25 = nullptr;
    }
  }

  /*!
      @brief    Initializes the Adafruit_PM25AQI instance.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    _pm25 = new Adafruit_PM25AQI();
    if (!_pm25)
      return false;
    delay(3 * ONE_SECOND_IN_MS); // Wait for the sensor to boot up
                                 /*     if (IsSoftwareSerial)
                                       return _pm25->begin_UART(_sw_serial); */
    return _pm25->begin_UART(_hw_serial);
  }

  /*!
      @brief    Gets the PM25 sensor's PM1.0 STD reading.
      @param    pm10StdEvent
                  Adafruit Sensor event for PM1.0
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  bool getEventPM10_STD(sensors_event_t *pm10StdEvent) {
    if (!refreshData()) {
      // TODO: Debug - remove for production PR
      WS_DEBUG_PRINTLN("Failed to read PM10STD data");
      return false; // couldn't read data
    }

    pm10StdEvent->pm10_std = (float)_cached_data.pm10_standard;
    // TODO: Debug - remove for production PR
    WS_DEBUG_PRINT("PM10STD: ");
    WS_DEBUG_PRINTLNVAR(pm10StdEvent->pm10_std);
    return true;
  }

  /*!
      @brief    Gets the PM25 sensor's PM2.5 STD reading.
      @param    pm25StdEvent
                  Adafruit Sensor event for PM2.5
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  bool getEventPM25_STD(sensors_event_t *pm25StdEvent) {
    if (!refreshData()) {
      // TODO: Debug - remove for production PR
      WS_DEBUG_PRINTLN("Failed to read PM25STD data");
      return false; // couldn't read data
    }
    pm25StdEvent->pm25_std = (float)_cached_data.pm25_standard;
    // TODO: Debug - remove for production PR
    WS_DEBUG_PRINT("PM25STD: ");
    WS_DEBUG_PRINTLNVAR(pm25StdEvent->pm25_std);
    return true;
  }

  /*!
      @brief    Gets the PM25 sensor's PM10.0 STD reading.
      @param    pm100StdEvent
                  Adafruit Sensor event for PM10.0
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  bool getEventPM100_STD(sensors_event_t *pm100StdEvent) {
    if (!refreshData()) {
      // TODO: Debug - remove for production PR
      WS_DEBUG_PRINTLN("Failed to read PM100STD data");
      return false; // couldn't read data
    }

    pm100StdEvent->pm100_std = (float)_cached_data.pm100_standard;
    // TODO: Debug - remove for production PR
    WS_DEBUG_PRINT("PM100STD: ");
    WS_DEBUG_PRINTLNVAR(pm100StdEvent->pm100_std);
    return true;
  }

  /*!
      @brief    Gets the PM25 sensor's PM1.0 ENV reading.
      @param    pm10EnvEvent
                  Adafruit Sensor event for environmental PM1.0
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  bool getEventPM10_ENV(sensors_event_t *pm10EnvEvent) {
    if (!refreshData()) {
      WS_DEBUG_PRINTLN("Failed to read PM10ENV data");
      return false; // couldn't read data
    }
    pm10EnvEvent->pm10_env = (float)_cached_data.pm10_env;
    return true;
  }

  /*!
      @brief    Gets the PM25 sensor's PM2.5 ENV reading.
      @param    pm25EnvEvent
                  Adafruit Sensor event for environmental PM2.5
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  bool getEventPM25_ENV(sensors_event_t *pm25EnvEvent) {
    if (!refreshData()) {
      WS_DEBUG_PRINTLN("Failed to read PM25ENV data");
      return false; // couldn't read data
    }
    pm25EnvEvent->pm25_env = (float)_cached_data.pm25_env;
    return true;
  }

  /*!
      @brief    Gets the PM25 sensor's PM10.0 ENV reading.
      @param    pm100EnvEvent
                  Adafruit Sensor event for environmental PM10.0
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  bool getEventPM100_ENV(sensors_event_t *pm100EnvEvent) {
    if (!refreshData()) {
      WS_DEBUG_PRINTLN("Failed to read PM100ENV data");
      return false; // couldn't read data
    }
    pm100EnvEvent->pm100_env = (float)_cached_data.pm100_env;
    return true;
  }

protected:
  /*!
      @brief    Reads at most one fresh PMS5003 frame per cache window so that
                all SI sub-sensors (std + env) report from a single sensor
                read per update cycle. The sensor only emits a frame roughly
                once per second, so calling read() once per sub-sensor would
                exhaust the buffer and fail every read after the first.
      @returns  True if cached data is valid (freshly read or within the
                cache window), False if a fresh read was needed but failed.
  */
  bool refreshData() {
    const unsigned long READ_CACHE_MS = 1000;
    if (_has_cached_data && (millis() - _cached_data_ms) < READ_CACHE_MS)
      return true;
    if (!_pm25->read(&_cached_data))
      return false;
    _cached_data_ms = millis();
    _has_cached_data = true;
    return true;
  }

  Adafruit_PM25AQI *_pm25 = nullptr; ///< Instance of the Adafruit_PM25AQI class
  PM25_AQI_Data _cached_data;        ///< Last decoded sensor frame
  unsigned long _cached_data_ms = 0; ///< millis() of last successful read
  bool _has_cached_data = false;     ///< True once a frame has been cached
};
#endif // DRV_UART_PM25_H