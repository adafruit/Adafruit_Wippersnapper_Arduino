/*!
 * @file src/components/i2c/hardware.cpp
 *
 * Hardware implementation for the i2c.proto API
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
#include "hardware.h"

/*!
    @brief  Default I2C bus hardware class constructor
*/
I2cHardware::I2cHardware(uint32_t sda, uint32_t scl, uint8_t instance) {
  _bus_status = ws_i2c_BusStatus_BS_UNSPECIFIED;
  _has_mux = false;
  _scl = (uint8_t)scl;
  _sda = (uint8_t)sda;
  _instance = instance;
}

/*!
    @brief  I2C hardware class destructor
*/
I2cHardware::~I2cHardware() { _has_mux = false; }

/*!
    @brief  Returns the I2C bus' status.
    @returns  The I2C bus status, as a ws_i2c_BusStatus.
*/
ws_i2c_BusStatus I2cHardware::GetBusStatus() { return _bus_status; }

/*!
    @brief  Optionally turns on the I2C bus, used for hardware with
            a power control pin for the I2C bus.
*/
void I2cHardware::TogglePowerPin() {
#if defined(PIN_I2C_POWER)
  // turn on the I2C power by setting pin to opposite of 'rest state'
  pinMode(PIN_I2C_POWER, INPUT);
  delay(1);
  bool polarity = digitalRead(PIN_I2C_POWER);
  WS_DEBUG_PRINT("[i2c] PIN_I2C_POWER rest state: ");
  WS_DEBUG_PRINTLNVAR(polarity);
  pinMode(PIN_I2C_POWER, OUTPUT);
  digitalWrite(PIN_I2C_POWER, !polarity);
  WS_DEBUG_PRINT("[i2c] PIN_I2C_POWER set to: ");
  WS_DEBUG_PRINTLNVAR(!polarity);
#elif defined(TFT_I2C_POWER)
  // ADAFRUIT_FEATHER_ESP32S2_TFT
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
#elif defined(NEOPIXEL_I2C_POWER)
  // ADAFRUIT_FEATHER_ESP32_V2
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
#endif
}

/*!
    @brief   Attempts to initialize the I2C bus
    @returns True if the bus was successfully initialized, False otherwise.
    NOTE: If False, the bus' status can be retrieved with GetBusStatus() to
   provide the failure reason.
*/
bool I2cHardware::begin() {
// Some development boards define a pin that controls power
// to the i2c bus. If the pin is defined, turn the power to the i2c bus on.
#if defined(PIN_I2C_POWER) || defined(TFT_I2C_POWER) ||                        \
    defined(NEOPIXEL_I2C_POWER)
  TogglePowerPin();
#endif

  // Enable pullups
  pinMode(_scl, INPUT_PULLUP);
  pinMode(_sda, INPUT_PULLUP);
  delay(150);

  // Is the bus stuck LOW?
  if (digitalRead(_scl) == 0 || digitalRead(_sda) == 0) {
    _bus_status = ws_i2c_BusStatus_BS_ERROR_PULLUPS;
    return false;
  }

  // Reset bus to a high-impedance state
  pinMode(_scl, INPUT);
  pinMode(_sda, INPUT);

// Initialize bus
// NOTE: Each platform has a slightly different bus initialization routine
#ifdef ARDUINO_ARCH_ESP32
  _bus = new TwoWire(_instance);
  _bus->setPins(_sda, _scl); // TODO: This is possibly not required due to ctor
  if (!_bus->begin(_sda, _scl)) {
    _bus_status = ws_i2c_BusStatus_BS_ERROR_HANG;
    return false;
  }
  _bus->setClock(50000);
#elif defined(ARDUINO_ARCH_ESP8266)
  // NOTE: Wire on ESP8266 has only one instance
  _bus = new TwoWire();
  _bus->begin(_sda, _scl);
  _bus->setClock(50000);
#elif defined(ARDUINO_ARCH_RP2040)
  // arduino-pico uses two global instances for TwoWire, select based on
  // instance number
  if (_instance == 0) {
    _bus = &Wire;
  } else if (_instance == 1) {
    _bus = &Wire1;
  } else {
    return false;
  }

  // Select IO pins to use before calling begin()
  if (!_bus->setSDA(_sda)) {
    _bus_status = ws_i2c_BusStatus_BS_ERROR_WIRING;
    return false;
  }
  if (!_bus->setSCL(_scl)) {
    _bus_status = ws_i2c_BusStatus_BS_ERROR_WIRING;
    return false;
  }

  // Start the bus as Master at 100kHz (default)
  _bus->begin();
#elif defined(ARDUINO_ARCH_SAMD)
  _bus = new TwoWire(&PERIPH_WIRE, _sda, _scl);
  _bus->begin();
#else
#error "I2C implementation not supported by this platform!"
#endif

  _bus_status = ws_i2c_BusStatus_BS_SUCCESS;
  return true;
}

/*!
    @brief    Probes specific I2C addresses on this bus for a given address
   space. Handles MUX channel selection/clearing internally.
    @param    address_space
                The address space to probe (bus pins + optional mux info).
    @param    addresses
                Array of specific addresses to probe. NULL or count==0 means
   scan all (1-126).
    @param    addresses_count
                Number of addresses in the array.
    @param    result
                Output AddressSpaceResult to populate.
    @param    found_buf
                Buffer to store found addresses (caller-owned).
    @param    found_count
                Output: number of addresses found.
    @returns  True if the probe completed, False on bus error.
*/
bool I2cHardware::ProbeAddresses(ws_i2c_AddressSpace *address_space,
                                 uint32_t *addresses, size_t addresses_count,
                                 ws_i2c_AddressSpaceResult *result,
                                 uint32_t *found_buf, size_t *found_count) {
  if (!result || !found_buf || !found_count)
    return false;

  *found_count = 0;

  // Copy address space into result
  result->has_address_space = true;
  result->address_space = *address_space;

  // If this address space uses a MUX, select the channel
  bool using_mux = (address_space->mux_address != 0);
  if (using_mux) {
    if (!_has_mux) {
      WS_DEBUG_PRINTLN(
          "[i2c] ERROR: AddressSpace specifies MUX but none on bus!");
      result->bus_status = ws_i2c_BusStatus_BS_ERROR_INVALID_CHANNEL;
      return false;
    }
    SelectMuxChannel(address_space->mux_channel);
  }

  // Probe addresses
  for (size_t i = 0; i < addresses_count; i++) {
    uint8_t addr = (uint8_t)addresses[i];
    // Skip reserved I2C addresses (0x00-0x07 and 0x78-0x7F)
    if (addr <= 0x07 || addr >= 0x78) {
      continue;
    }
    _bus->beginTransmission(addr);
    uint8_t rc = _bus->endTransmission();
    if (rc == 0) {
      if (*found_count >= MAX_I2C_ADDRESSES) {
        WS_DEBUG_PRINTLN("[i2c] WARNING: found_buf full, stopping probe");
        break;
      }
      WS_DEBUG_PRINT("[i2c] Found device at 0x");
      WS_DEBUG_PRINTHEX(addr);
      WS_DEBUG_PRINTLN("");
      found_buf[*found_count] = addr;
      (*found_count)++;
    }
  }

  // Clear MUX channel if we used one
  if (using_mux) {
    ClearMuxChannel();
  }

  result->bus_status = ws_i2c_BusStatus_BS_SUCCESS;
  return true;
}

/*!
    @brief  Returns a pointer to the I2C bus.
    @returns  Pointer to the I2C bus.
*/
TwoWire *I2cHardware::GetBus() { return _bus; }

/*!
    @brief  Adds a MUX to the I2C bus.
    @param    address_register
                The MUX's address register.
    @param    name
                The MUX's name.
    @returns  True if the MUX was successfully added to the bus,
              False otherwise.
*/
bool I2cHardware::AddMuxToBus(uint32_t address_register, const char *name) {
  if (strcmp(name, "pca9546") == 0) {
    _mux_max_channels = 4; // PCA9546 supports 4 channels
  } else if (strcmp(name, "pca9548") == 0 || strcmp(name, "tca9548a") == 0) {
    _mux_max_channels = 8; // PCA9548 supports 8 channels
  } else {
    return false;
  }

  _mux_address_register = address_register;
  _has_mux = true;
  // Put MUX in back into its default state cuz we don't know if we're about to
  // use it again
  ClearMuxChannel();
  return true;
}

/*!
    @brief  Removes a MUX from the I2C bus.
*/
void I2cHardware::RemoveMux() {
  ClearMuxChannel();
  _has_mux = false;
  _mux_address_register = 0;
  _mux_max_channels = 0;
}

/*!
    @brief  Clears the enabled MUX channel.
*/
void I2cHardware::ClearMuxChannel() {
  if (!_has_mux)
    return;
  _bus->beginTransmission((uint8_t)_mux_address_register);
  if (_mux_max_channels == 4)
    _bus->write(0b0000);
  else if (_mux_max_channels == 8)
    _bus->write(0b00000000);
  _bus->endTransmission();
}

/*!
    @brief  Enables a specific channel on a MUX.
    @param    channel
                The desired MUX channel to enable.
*/
void I2cHardware::SelectMuxChannel(uint32_t channel) {
  if (channel > _mux_max_channels - 1)
    return;
  _bus->beginTransmission((uint8_t)_mux_address_register);
  _bus->write(1 << channel);
  _bus->endTransmission();
}

/*!
    @brief  Returns if a MUX is present on the I2C bus.
    @returns  True if a MUX is present on the bus, False otherwise.
*/
bool I2cHardware::HasMux() { return _has_mux; }

int I2cHardware::GetMuxMaxChannels() { return _mux_max_channels; }
