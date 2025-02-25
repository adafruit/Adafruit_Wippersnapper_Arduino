#include "hardware.h"

/***********************************************************************/
/*!
    @brief  I2C hardware class constructor
*/
/***********************************************************************/
I2cHardware::I2cHardware() { _has_mux = false; }

/***********************************************************************/
/*!
    @brief  I2C hardware class destructor
*/
/***********************************************************************/
I2cHardware::~I2cHardware() { _has_mux = false; }

/***********************************************************************/
/*!
    @brief  Returns the I2C bus' status.
    @returns  The I2C bus status, as a wippersnapper_i2c_I2cBusStatus.
*/
/***********************************************************************/
wippersnapper_i2c_I2cBusStatus I2cHardware::GetBusStatus() {
  return _bus_status;
}

/***********************************************************************/
/*!
    @brief  Optionally turns on the I2C bus, used for hardware with
            a power control pin for the I2C bus.
*/
/***********************************************************************/
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

/***********************************************************************/
/*!
    @brief  Initializes an I2C bus.
    @param    is_default
                True if the default I2C bus is being used,
                False if an alternative I2C bus is being used.
    @param    sda
                The desired SDA pin.
    @param    scl
                The desired SCL pin.
*/
/***********************************************************************/
void I2cHardware::InitBus(bool is_default, const char *sda, const char *scl) {
  uint8_t pin_sda, pin_scl;
  if (!is_default && (sda == nullptr || scl == nullptr)) {
    _bus_status = wippersnapper_i2c_I2cBusStatus_I2C_BUS_STATUS_ERROR_WIRING;
    return;
  }
// Some development boards define a pin that controls power
// to the i2c bus. If the pin is defined, turn the power to the i2c bus on.
#if defined(PIN_I2C_POWER) || defined(TFT_I2C_POWER) ||                        \
    defined(NEOPIXEL_I2C_POWER)
  TogglePowerPin();
#endif

  // Assign I2C bus pins
  if (is_default) {
#ifndef ARDUINO_ARCH_RP2040
    pin_sda = SDA;
    pin_scl = SCL;
#else
    // RP2040 BSP uses a different naming scheme than Espressif for I2C pins
    pin_sda = PIN_WIRE0_SDA;
    pin_scl = PIN_WIRE0_SCL;
#endif
  } else {
    pin_sda = atoi(sda);
    pin_scl = atoi(scl);
  }

  // Enable pullups
  pinMode(pin_scl, INPUT_PULLUP);
  pinMode(pin_sda, INPUT_PULLUP);
  delay(150);

  // Is the bus stuck LOW?
  if (digitalRead(pin_scl) == 0 || digitalRead(pin_sda) == 0) {
    _bus_status = wippersnapper_i2c_I2cBusStatus_I2C_BUS_STATUS_ERROR_PULLUPS;
    return;
  }

  // Reset bus to a high-impedance state
  pinMode(pin_scl, INPUT);
  pinMode(pin_sda, INPUT);

// Initialize bus
// NOTE: Each platform has a slightly different bus initialization routine
#ifdef ARDUINO_ARCH_ESP32
  if (is_default) {
    _bus = new TwoWire(0);
  } else {
    _bus = new TwoWire(1);
    Wire1.setPins(pin_sda, pin_scl);
  }
  if (!_bus->begin(pin_sda, pin_scl)) {
    _bus_status = wippersnapper_i2c_I2cBusStatus_I2C_BUS_STATUS_ERROR_HANG;
    return;
  }
  _bus->setClock(50000);
#elif defined(ARDUINO_ARCH_ESP8266)
  _bus = new TwoWire();
  _bus->begin(pin_sda, pin_scl);
  _bus->setClock(50000);
#elif defined(ARDUINO_ARCH_RP2040)
  _bus = &WIRE;
  _bus->setSDA(pin_sda);
  _bus->setSCL(pin_scl);
  _bus->begin();
#elif defined(ARDUINO_ARCH_SAM)
  _bus = new TwoWire(&PERIPH_WIRE, pin_sda, pin_scl);
  _bus->begin();
#else
#error "I2C bus implementation not supported by this platform!"
#endif

  _bus_status = wippersnapper_i2c_I2cBusStatus_I2C_BUS_STATUS_SUCCESS;
}

/***********************************************************************/
/*!
    @brief  Returns a pointer to the I2C bus.
    @returns  Pointer to the I2C bus.
*/
/***********************************************************************/
TwoWire *I2cHardware::GetBus() { return _bus; }

/***********************************************************************/
/*!
    @brief  Adds a MUX to the I2C bus.
    @param    address_register
                The MUX's address register.
    @param    name
                The MUX's name.
    @returns  True if the MUX was successfully added to the bus,
              False otherwise.
*/
/***********************************************************************/
bool I2cHardware::AddMuxToBus(uint32_t address_register, const char *name) {
  if (strcmp(name, "pca9546") == 0) {
    _mux_max_channels = 4; // PCA9546 supports 4 channels
  } else if (strcmp(name, "pca9548") == 0) {
    _mux_max_channels = 4; // PCA9548 supports 4 channels
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

/***********************************************************************/
/*!
    @brief  Clears the enabled MUX channel.
*/
/***********************************************************************/
void I2cHardware::ClearMuxChannel() {
  if (!_has_mux)
    return;
  _bus->beginTransmission(_mux_address_register);
  if (_mux_max_channels == 4)
    _bus->write(0b0000);
  else if (_mux_max_channels == 8)
    _bus->write(0b00000000);
  _bus->endTransmission();
}

/***********************************************************************/
/*!
    @brief  Enables a specific channel on a MUX.
    @param    channel
                The desired MUX channel to enable.
*/
/***********************************************************************/
void I2cHardware::SelectMuxChannel(uint32_t channel) {
  if (channel > _mux_max_channels - 1)
    return;
  _bus->beginTransmission(_mux_address_register);
  _bus->write(1 << channel);
  _bus->endTransmission();
}

/***********************************************************************/
/*!
    @brief  Returns if a MUX is present on the I2C bus.
    @returns  True if a MUX is present on the bus, False otherwise.
*/
/***********************************************************************/
bool I2cHardware::HasMux() { return _has_mux; }