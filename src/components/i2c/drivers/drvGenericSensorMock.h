/*!
 * @file drvGenericSensorMock.h
 *
 * Mock I2C sensor driver for testing. Initializes successfully but
 * always fails sensor reads, useful for testing watchdog timeout behavior.
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
#ifndef DRV_GENERIC_SENSOR_MOCK_H
#define DRV_GENERIC_SENSOR_MOCK_H

#include "drvBase.h"

/*!
    @brief  Mock I2C sensor driver that initializes successfully but
            always fails sensor reads.
*/
class drvGenericSensorMock : public drvBase {
public:
  /*!
      @brief    Constructor for a mock sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvGenericSensorMock(TwoWire *i2c, uint16_t sensorAddress,
                       uint32_t mux_channel, const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {}

  /*!
      @brief    Destructor for the mock sensor.
  */
  ~drvGenericSensorMock() {}

  /*!
      @brief    Initializes the mock sensor - always succeeds.
      @returns  True always.
  */
  bool begin() override { return true; }

  /*!
      @brief    Attempts to read the mock sensor - always fails.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  False always.
  */
  bool getEventAmbientTemp(sensors_event_t *tempEvent) { return false; }

  /*!
      @brief    Attempts to read the mock sensor - always fails.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  False always.
  */
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) { return false; }

  /*!
      @brief    Attempts to read the mock sensor - always fails.
      @param    pressureEvent
                Pointer to an Adafruit_Sensor event.
      @returns  False always.
  */
  bool getEventPressure(sensors_event_t *pressureEvent) { return false; }
};

#endif // DRV_GENERIC_SENSOR_MOCK_H
