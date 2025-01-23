#include "hardware.h"

I2cHardware::I2cHardware() {
  // TODO
}
I2cHardware::~I2cHardware() {
  // TODO
}

void I2cHardware::ToggleI2CPowerPin() {
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
  ToggleI2CPowerPin();
#endif

  // Assign I2C bus pins
  if (is_default) {
    pin_sda = SDA;
    pin_scl = SCL;
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
  _i2c_bus = new TwoWire(0);
  if (!_i2c_bus->begin(pin_sda, pin_scl))
    _bus_status = wippersnapper_i2c_I2cBusStatus_I2C_BUS_STATUS_ERROR_HANG;
  _i2c_bus->setClock(50000);
#elif defined(ARDUINO_ARCH_ESP8266)
  _i2c_bus = new TwoWire();
  _i2c_bus->begin(pin_sda, pin_scl);
  _i2c_bus->setClock(50000);
#elif defined(ARDUINO_ARCH_RP2040)
  _i2c = &WIRE;
  _i2c->begin();
#elif defined(ARDUINO_ARCH_SAM)
  _i2c = new TwoWire(&PERIPH_WIRE, pin_sda, pin_scl);
  _i2c->begin();
#else
#error "I2C bus implementation not supported by this platform!"
#endif

  _bus_status = wippersnapper_i2c_I2cBusStatus_I2C_BUS_STATUS_SUCCESS;
}