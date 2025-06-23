/*!
 * @file drvUartUs100.h
 *
 * UART driver for a GPS component
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

#ifndef DRV_UART_GPS_H
#define DRV_UART_GPS_H
#include "drvUartBase.h"
#include "../../gps/controller.h"


/*!
    @brief  Provides an interface for the US-100 Ultrasonic Distance Sensor over
            UART.
*/
class drvUartGps : public drvUartBase {

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
  drvUartGps(HardwareSerial *hw_serial, const char *driver_name,
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
  drvUartGps(SoftwareSerial *sw_serial, const char *driver_name,
               uint32_t port_num)
      : drvUartBase(sw_serial, driver_name, port_num) {
    // Handled by drvUartBase constructor
  }
#endif // HAS_SW_SERIAL

  /*!
      @brief    Destructor for a UART GPS.
  */
  ~drvUartGps() { /* TODO: Add back dtor */ }

  /*!
      @brief    Initializes a UART GPS device.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    if (_hw_serial == nullptr)
      return false;
    _gps = new GPSController();
    // TODO TUES: Note that _hw_serial isnt a pointer to the UARTHardware
    // and instead the raw hw serial. This is fine 
    // but we'll have to refactor the GPS controller iface to take in
    // a hardwareserial ptr rather than a UARTHardware ptr.
    if (!_gps->SetInterface(_hw_serial)) {
      WS_DEBUG_PRINTLN("[uart] ERROR: Failed to set GPS interface!");
      delete _gps; // cleanup
      _gps = nullptr;
      return false;
    }
    WS_DEBUG_PRINTLN("[uart] Initializing GPS device...");
    if (!_gps->begin()) {
      WS_DEBUG_PRINTLN("[uart] ERROR: Failed to initialize GPS device!");
      delete _gps; // cleanup
      _gps = nullptr;
      return false;
    }
    WS_DEBUG_PRINTLN("[uart] GPS device initialized successfully!");
    return true;
  }


protected:
  GPSController *_gps;
};
#endif // DRV_UART_US100_H