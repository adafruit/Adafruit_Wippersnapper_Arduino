/*!
 * @file src/components/i2c/hardware.h
 *
 * Hardware instance for the i2c.proto API
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2026 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_I2C_HARDWARE_H
#define WS_I2C_HARDWARE_H
#include "drivers/drvBase.h" ///< Base driver class
#include "wippersnapper.h"

#ifdef ARDUINO_ARCH_RP2040
// Wire uses GPIO4 (SDA) and GPIO5 (SCL) automatically.
#define WIRE Wire
#endif

#define I2C_WDT_TIMEOUT_MS 50

/*!
    @brief  Interfaces with the I2C bus via the Arduino "Wire" API.
*/
class I2cHardware {
public:
  I2cHardware(uint32_t sda, uint32_t scl, uint8_t instance = 0);
  ~I2cHardware();
  // Bus API
  bool begin();
  bool ScanBus(ws_i2c_Scanned *scan_results);
  TwoWire *GetBus();
  uint8_t getSDA() { return _sda; }
  uint8_t getSCL() { return _scl; }
  ws_i2c_BusStatus GetBusStatus();
  void TogglePowerPin();
  // MUX API
  bool AddMuxToBus(uint32_t address_register, const char *name);
  void RemoveMux();
  bool HasMux();
  void ClearMuxChannel();
  void SelectMuxChannel(uint32_t channel);
  bool ScanMux(ws_i2c_Scanned *scan_results);

private:
  TwoWire *_bus = nullptr;      ///< I2C bus instance
  ws_i2c_BusStatus _bus_status; ///< I2C bus status
  uint8_t _sda;                 ///< SDA pin
  uint8_t _scl;                 ///< SCL pin
  uint8_t _instance; ///< I2C bus instance number (for hardware with multiple
                     ///< I2C buses)
  bool _has_mux;     ///< Is a MUX present on the bus?
  uint32_t _mux_address_register; ///< I2C address for the MUX
  int _mux_max_channels;          ///< Maximum possible number of MUX channels
};
#endif // WS_I2C_HARDWARE_H