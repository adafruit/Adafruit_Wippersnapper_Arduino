/*!
 * @file hardware.cpp
 *
 * Hardware driver for the i2c API
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_I2C_HARDWARE_H
#define WS_I2C_HARDWARE_H
#include "Wippersnapper_V2.h"
#include "drivers/drvBase.h" ///< Base driver class

#ifdef ARDUINO_ARCH_RP2040
// Wire uses GPIO4 (SDA) and GPIO5 (SCL) automatically.
#define WIRE Wire
#endif

#define I2C_WDT_TIMEOUT_MS 50

/**************************************************************************/
/*!
    @brief  Interfaces with the I2C bus via the Arduino "Wire" API.
*/
/**************************************************************************/
class I2cHardware {
public:
  I2cHardware();
  ~I2cHardware();
  void InitBus(bool is_default, const char *sda = nullptr,
               const char *scl = nullptr);
  TwoWire *GetBus();
  wippersnapper_i2c_I2cBusStatus GetBusStatus();
  bool ScanBus(wippersnapper_i2c_I2cBusScanned* scan_results);
  // MUX
  bool AddMuxToBus(uint32_t address_register, const char *name);
  void SelectMuxChannel(uint32_t channel);
  bool HasMux();
  void ClearMuxChannel();
private:
  void TogglePowerPin();
  wippersnapper_i2c_I2cBusStatus _bus_status; ///< I2C bus status
  TwoWire *_bus = nullptr;                    ///< I2C bus
  uint8_t _bus_sda;                           ///< SDA pin
  uint8_t _bus_scl;                           ///< SCL pin
  bool _has_mux;                              ///< Is a MUX present on the bus?
  uint32_t _mux_address_register;             ///< I2C address for the MUX
  int _mux_max_channels;                      ///< Maximum possible number of MUX channels
};
#endif // WS_I2C_HARDWARE_H