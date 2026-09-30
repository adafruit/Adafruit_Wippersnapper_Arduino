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

/// Standard-mode I2C clock for ESP32 buses. 100 kHz; a slower 50 kHz failed to
/// detect some devices on the STEMMA bus during HIL testing.
#define I2C_STD_CLOCK_HZ 100000
#define I2C_WDT_TIMEOUT_MS 50 ///< I2C timeout
#define MAX_I2C_ADDRESSES 112 ///< 128 total 7-bit addresses minus 16 reserved

/** Defines the result codes returned by I2cHardware::ProbeAddresses() */
typedef enum {
  WS_I2C_PROBE_OK = 0,                 // Probe completed
  WS_I2C_PROBE_ERR_INVALID_ARGS = 1,   // Null result/found_buf/found_count
  WS_I2C_PROBE_ERR_NO_MUX = 2,         // AddressSpace specifies MUX, none on bus
  WS_I2C_PROBE_ERR_TOO_MANY_ADDRS = 3, // Address list exceeds MAX_I2C_ADDRESSES
} ws_i2c_probe_err_t;

/*!
    @brief  Interfaces with the I2C bus via the Arduino "Wire" API.
*/
class I2cHardware {
public:
  /*!
      @brief    Constructor for the I2cHardware class.
      @param    sda       The pin number to use for the SDA line.
      @param    scl       The pin number to use for the SCL line.
      @param    instance  The I2C bus instance (for platforms with multiple
                          hardware buses).
  */
  I2cHardware(uint32_t sda, uint32_t scl, uint8_t instance = 0);
  ~I2cHardware();
  // Bus API
  bool begin();
  ws_i2c_probe_err_t ProbeAddresses(ws_i2c_AddressSpace *address_space,
                                    uint32_t *addresses, size_t addresses_count,
                                    ws_i2c_AddressSpaceResult *result,
                                    uint32_t *found_buf, size_t *found_count);
  static const char *ProbeErrorToString(ws_i2c_probe_err_t err);
  TwoWire *GetBus();
  /*!
      @brief  Returns the SDA pin number.
      @returns The SDA pin number.
  */
  uint8_t getSDA() { return _sda; }
  /*!
      @brief  Returns the SCL pin number.
      @returns The SCL pin number.
  */
  uint8_t getSCL() { return _scl; }
  bool isBusInitialized();
  void TogglePowerPin();
  // MUX API
  bool AddMuxToBus(uint32_t address_register, const char *name);
  void RemoveMux();
  bool HasMux();
  void ClearMuxChannel();
  void SelectMuxChannel(uint32_t channel);
  /*!
      @brief  Returns the max number of MUX channels.
      @returns The max number of MUX channels.
  */
  int GetMuxMaxChannels();

private:
  TwoWire *_bus = nullptr; ///< I2C bus instance
  bool _bus_init = false;  ///< I2C bus status
  bool _has_mux = false;   ///< Is a MUX present on the bus?
  uint8_t _sda;            ///< SDA pin
  uint8_t _scl;            ///< SCL pin
  uint8_t _instance; ///< I2C bus instance number (for hardware with multiple
                     ///< I2C buses)
  uint32_t _mux_address_register; ///< I2C address for the MUX
  int _mux_max_channels;          ///< Maximum possible number of MUX channels
};
#endif // WS_I2C_HARDWARE_H