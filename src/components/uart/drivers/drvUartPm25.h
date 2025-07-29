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
      @param    port_num
                The port number for the UART device corresponding to the Serial
     instance.
  */
  drvUartPm25(HardwareSerial *hw_serial, const char *driver_name,
              uint32_t port_num)
      : drvUartBase(hw_serial, driver_name, port_num) {
    // Handled by drvUartBase constructor
  }

#if HAS_SW_SERIAL
  /*!
    @brief    Instantiates a UART device.
    @param    sw_serial
              Pointer to a SoftwareSerial instance.
    @param    device_type
              The type of device connected to the UART port.
    @param    driver_name
              The name of the driver.
    @param   port_num
              The port number for the UART device corresponding to the Serial
    instance.
*/
  drvUartPm25(SoftwareSerial *sw_serial,
              const char *driver_name, uint32_t port_num)
      : drvUartBase(sw_serial, driver_name, port_num) {
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
    if (_hw_serial == nullptr)
      return false;

    _pm25 = new Adafruit_PM25AQI();
    if (_pm25 == nullptr)
        return false;

    delay(1000); // Wait for the sensor to boot
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
    PM25_AQI_Data data;
    if (!_pm25->read(&data))
      return false; // couldn't read data

    pm10StdEvent->pm10_std = (float)data.pm10_standard;
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
    PM25_AQI_Data data;
    if (!_pm25->read(&data)) {
      return false; // couldn't read data

    pm25StdEvent->pm25_std = (float)data.pm25_standard;
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
    PM25_AQI_Data data;
    if (!_pm25->read(&data))
      return false; // couldn't read data

    pm100StdEvent->pm100_std = (float)data.pm100_standard;
    return true;
  }

protected:
  Adafruit_PM25AQI *_pm25 = nullptr; ///< Instance of the Adafruit_PM25AQI class
};
#endif // DRV_UART_PM25_H