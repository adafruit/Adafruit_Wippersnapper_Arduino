/*!
 * @file drvUartUs100.h
 *
 * UART driver for the US-100 Ultrasonic Distance Sensor
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

#ifndef DRV_UART_US100_H
#define DRV_UART_US100_H
#include "Adafruit_GenericDevice.h"
#include "drvUartBase.h"

#define CMD_US100_REQ_TEMP 0x50     ///< Temperature request command
#define CMD_US100_REQ_DISTANCE 0x55 ///< Distance request command
#define US_100_DELAY_TIME 100       ///< Delay time for US-100, in milliseconds

/**
 * Basic UART device class that demonstrates using GenericDevice with a Stream
 * interface. This example shows how to wrap a Stream (like HardwareSerial or
 * SoftwareSerial) with read/write callbacks that can be used by BusIO's
 * register functions.
 */

class UARTDevice {
public:
  UARTDevice(Stream *serial) { _serial_dev = serial; }
  ~UARTDevice() { /* no-op destructor */ }

  bool CreateDevice() {
    if (_generic_dev != nullptr) {
      return false; // already initialized
    }
    _generic_dev = new Adafruit_GenericDevice(this, uart_read, uart_write);
    return _generic_dev->begin();
  }

  static bool uart_write(void *thiz, const uint8_t *buffer, size_t len) {
    UARTDevice *dev = (UARTDevice *)thiz;
    dev->_serial_dev->write(buffer, len);
    return true;
  }

  static bool uart_read(void *thiz, uint8_t *buffer, size_t len) {
    UARTDevice *dev = (UARTDevice *)thiz;
    uint16_t timeout = 100;
    while (dev->_serial_dev->available() < len && timeout--) {
      delay(1);
    }
    if (timeout == 0) {
      return false;
    }
    for (size_t i = 0; i < len; i++) {
      buffer[i] = dev->_serial_dev->read();
    }
    return true;
  }

  Adafruit_GenericDevice *getGenericDevice() { return _generic_dev; }

private:
  Adafruit_GenericDevice *_generic_dev = nullptr;
  Stream *_serial_dev = nullptr;
};

/*!
    @brief  Provides an interface for the US-100 Ultrasonic Distance Sensor over
            UART.
*/
class drvUartUs100 : public drvUartBase {

public:
  /*!
      @brief    Instantiates a US-100 UART device.
      @param    hw_serial
                Pointer to a HardwareSerial instance.
      @param    driver_name
                The name of the driver.
      @param    port_num
                The port number for the UART device corresponding to the Serial
     instance.
  */
  drvUartUs100(HardwareSerial *hw_serial, const char *driver_name,
               uint32_t port_num)
      : drvUartBase(hw_serial, driver_name, port_num) {
    // Handled by drvUartBase constructor
  }

#if HAS_SW_SERIAL
  /*!
    @brief    Instantiates a US-100 UART device.
    @param    sw_serial
              Pointer to a SoftwareSerial instance.
    @param    driver_name
              The name of the driver.
    @param   port_num
              The port number for the UART device corresponding to the Serial
    instance.
*/
  drvUartUs100(SoftwareSerial *sw_serial, const char *driver_name,
               uint32_t port_num)
      : drvUartBase(sw_serial, driver_name, port_num) {
    // Handled by drvUartBase constructor
  }
#endif // HAS_SW_SERIAL

  /*!
      @brief    Destructor for a US-100 UART device.
  */
  ~drvUartUs100() {
    if (_uart_dev) {
      delete _uart_dev; // Clean up the UARTDevice instance
      _uart_dev = nullptr;
    }
  }

  /*!
      @brief    Initializes the US-100 UART device.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
#if HAS_SW_SERIAL
    if (_sw_serial == nullptr)
      return false;
    _uart_dev = new UARTDevice(_sw_serial);
#else
    if (_hw_serial == nullptr)
      return false;
    _uart_dev = new UARTDevice(_hw_serial);
#endif // HAS_SW_SERIAL

    // Create a GenericDevice instance using the UARTDevice
    return _uart_dev->CreateDevice();
    return true;
  }

  /*!
      @brief    Gets the US-100 Ultrasonic Distance Sensor's temperature, in
     Celsius.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    uint8_t len_write_buf = 1;
    // Write the temperature request command to the US-100
    uint8_t write_buf[len_write_buf] = {CMD_US100_REQ_TEMP};
    if (!_uart_dev->getGenericDevice()->write(write_buf, len_write_buf))
      return false;
    // Wait for US-100 to respond
    delay(US_100_DELAY_TIME);

    // Attempt to read the response from the US-100
    uint8_t len_read_buf = 1;
    uint8_t read_buf[len_read_buf] = {0};
    if (!_uart_dev->getGenericDevice()->read(read_buf, len_read_buf))
      return false;

    // Check if the response is within a valid range
    if ((read_buf[0] > 1) && (read_buf[0] < 130)) {
      // Populate the temperature event
      tempEvent->temperature = read_buf[0] - 45;
    } else {
      return false;
    }

    return true;
  }

  /*!
      @brief    Gets the US-100 sensor's distance measurement.
      @param    rawEvent
                The Raw value, contains the distance in centimeters.
      @returns  True if the distance value was obtained successfully, False
                otherwise.
  */
  bool getEventRaw(sensors_event_t *rawEvent) {
    // Write the distance request command to the US-100
    uint8_t len_write_buf = 1;
    uint8_t write_buf[len_write_buf] = {CMD_US100_REQ_DISTANCE};
    WS_DEBUG_PRINTLN("[uart] Requesting distance from US-100...");

    if (!_uart_dev->getGenericDevice()->write(write_buf, len_write_buf))
      return false;
    // Wait for US-100 to respond
    delay(US_100_DELAY_TIME);

    // Read the response from the US-100
    uint8_t len_read_buf = 2;
    uint8_t read_buf[len_read_buf] = {0};
    if (!_uart_dev->getGenericDevice()->read(read_buf, len_read_buf))
      return false;

    // Calculate distance
    uint16_t dist_high_byte = read_buf[0];
    uint16_t dist_low_byte = read_buf[1];
    uint16_t distance = dist_high_byte * 256 + dist_low_byte;

    // Check if the distance is within valid range
    if ((distance < 1) || (distance > 10000)) {
      return false;
    }

    // Populate the raw event with the distance value in cm
    rawEvent->data[0] = (float)distance;
    return true;
  }

protected:
  UARTDevice *_uart_dev = nullptr; ///< Pointer to the UARTDevice instance
};
#endif // DRV_UART_US100_H