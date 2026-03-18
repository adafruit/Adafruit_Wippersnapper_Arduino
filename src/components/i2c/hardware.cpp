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
I2cHardware::I2cHardware(uint32_t sda, uint32_t scl, uint8_t instance = 0) {
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
  pinMode(PIN_I2C_POWER, OUTPUT);
  digitalWrite(PIN_I2C_POWER, !polarity);
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
    NOTE: If False, the bus' status can be retrieved with GetBusStatus() to provide the failure reason.
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
  // _bus->setPins(_sda, _scl); TODO: This is possibly not required due to ctor
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
  // arduino-pico uses two global instances for TwoWire, select based on instance number
  if (instance == 0) {
    _bus = &Wire;
  } else if (instance == 1) {
    _bus = &Wire1;
  } else {
    return false;
  }

  // Select IO pins to use before calling begin()
  if (!_bus->setSDA(_bus_sda)) {
    _bus_status = ws_i2c_BusStatus_BS_ERROR_WIRING;
    return false;
  }
  if (!_bus->setSCL(_bus_scl)) {
    _bus_status = ws_i2c_BusStatus_BS_ERROR_WIRING;
    return false;
  }

  // Start the bus as Master at 100kHz (default)
  _bus->begin();
#elif defined(ARDUINO_ARCH_SAMD)
  _bus = new TwoWire(&PERIPH_WIRE, _bus_sda, _bus_scl);
  _bus->begin();
#else
#error "I2C implementation not supported by this platform!"
#endif

  _bus_status = ws_i2c_BusStatus_BS_SUCCESS;
  return true;
}

/*!
    @brief  Clears the MUX channel.
    @param  scan_results
                The I2C bus scan results.
    @returns  True if the MUX channel was successfully cleared,
              False otherwise.
*/
bool I2cHardware::ScanBus(ws_i2c_Scanned *scan_results) {
  if (!scan_results)
    return false;

  // TODO: WS object needs to be added for this to work?
  /*     #ifndef ARDUINO_ARCH_ESP32
        // Set I2C WDT timeout to catch I2C hangs, SAMD-specific
        WS.enableWDT(I2C_WDT_TIMEOUT_MS);
        WS.feedWDT();
      #endif */

  // Get the SDA and SCL pins from the bus
  // TODO: Abstract this?
  char i2c_bus_scl[15] = {0}, i2c_bus_sda[15] = {0};
  snprintf(i2c_bus_scl, sizeof(i2c_bus_scl), "D%u", _bus_scl);
  snprintf(i2c_bus_sda, sizeof(i2c_bus_sda), "D%u", _bus_sda);

  // Perform a bus scan
  WS_DEBUG_PRINTLN("[i2c]: Scanning I2C Bus for Devices...");
  for (uint8_t address = 1; address < 127; ++address) {
    WS_DEBUG_PRINT("[i2c] Scanning Address: 0x");
    WS_DEBUG_PRINTHEX(address);
    WS_DEBUG_PRINTLN("");
    _bus->beginTransmission(address);
    uint8_t endTransmissionRC = _bus->endTransmission();

    if (endTransmissionRC == 0) {
      WS_DEBUG_PRINTLN("[i2c] Found Device!");
      // TODO: Abstract this? Allow for mux flags to be set here, too
      scan_results->bus_found_devices[scan_results->bus_found_devices_count]
          .device_address = address;
      scan_results->bus_found_devices[scan_results->bus_found_devices_count]
          .mux_address = 0xFFFF; // Tell user that device is not on a mux
      strcpy(
          scan_results->bus_found_devices[scan_results->bus_found_devices_count]
              .bus_sda,
          i2c_bus_sda);
      strcpy(
          scan_results->bus_found_devices[scan_results->bus_found_devices_count]
              .bus_scl,
          i2c_bus_scl);
      scan_results->bus_found_devices_count++;
    }
#if defined(ARDUINO_ARCH_ESP32)
    // Check endTransmission()'s return code (Arduino-ESP32 ONLY)
    else if (endTransmissionRC == 3) {
      WS_DEBUG_PRINTLN("[i2c] Did not find device: NACK on transmit of data!");
      continue;
    } else if (endTransmissionRC == 2) {
      // WS_DEBUG_PRINTLN("[i2c] Did not find device: NACK on transmit of
      // address!");
      continue;
    } else if (endTransmissionRC == 1) {
      WS_DEBUG_PRINTLN(
          "[i2c] Did not find device: data too long to fit in xmit buffer!");
      continue;
    } else if (endTransmissionRC == 4) {
      WS_DEBUG_PRINTLN(
          "[i2c] Did not find device: Unspecified bus error occured!");
      continue;
    } else if (endTransmissionRC == 5) {
      WS_DEBUG_PRINTLN("[i2c] Did not find device: Bus timed out!");
      continue;
    }
#endif // ARDUINO_ARCH_ESP32
    else {
      WS_DEBUG_PRINTLN(
          "[i2c] Did not find device: Unknown bus error has occured!");
      continue;
    }
  }

  /*
  #ifndef ARDUINO_ARCH_ESP32
      // re-enable WipperSnapper SAMD WDT global timeout
      WS.enableWDT(WS_TIMEOUT_WDT);
      WS.feedWDT();
  #endif
  */
  return true; // TODO: Change this!
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

/*!
    @brief  Returns the maximum number of channels on the MUX.
    @param  scan_results
                The I2C bus scan results.
    @returns  The maximum number of channels on the MUX.
*/
bool I2cHardware::ScanMux(ws_i2c_Scanned *scan_results) {
  if (!HasMux()) {
    WS_DEBUG_PRINTLN("[i2c] ERROR: No MUX present on the bus!");
    return false;
  }

  for (uint8_t ch = 0; ch < _mux_max_channels; ch++) {
    SelectMuxChannel(ch);
    WS_DEBUG_PRINT("[i2c] Scanning MUX Channel # ");
    WS_DEBUG_PRINTLNVAR(ch);
    if (!ScanBus(scan_results)) {
      WS_DEBUG_PRINTLN("[i2c] ERROR: Failed to scan MUX channel!");
      return false;
    }
  }
  return true;
}