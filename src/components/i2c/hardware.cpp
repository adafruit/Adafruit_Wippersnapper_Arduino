#include "hardware.h"

I2cHardware::I2cHardware() {
  // TODO
}
I2cHardware::~I2cHardware() {
  // TODO
}

void ToggleI2CPowerPin() {
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

void I2cHardware::InitDefaultBus() {
// Some development boards define a pin that controls power
// to the i2c bus. If the pin is defined, turn the power to the i2c bus on.
#if defined(PIN_I2C_POWER) || defined(TFT_I2C_POWER) ||                        \
    defined(NEOPIXEL_I2C_POWER)
  ToggleI2CPowerPin();
#endif

  // Enable pullups on I2c bus
  pinMode(SCL, INPUT_PULLUP);
  pinMode(SDA, INPUT_PULLUP);
  delay(150);

  // Are the default I2c pins stuck LOW?
  if (digitalRead(SCL) == 0 || digitalRead(SDA) == 0) {
    _bus_status = wippersnapper_i2c_I2cBusStatus_I2C_BUS_STATUS_ERROR_PULLUPS;
    return;
  }

  // Reset I2c bus to a high-impedance state
  pinMode(SCL, INPUT);
  pinMode(SDA, INPUT);

// Each platform has a slightly different I2c bus initialization routine
#ifdef ARDUINO_ARCH_ESP32
  _i2c_bus = new TwoWire(0);
  if (!_i2c_bus->begin(SDA, SCL))
    _bus_status = wippersnapper_i2c_I2cBusStatus_I2C_BUS_STATUS_ERROR_HANG;
  _i2c_bus->setClock(50000);
#elif defined(ARDUINO_ARCH_ESP8266)
  _i2c_bus = new TwoWire();
  _i2c_bus->begin(SDA, SCL);
  _i2c_bus->setClock(50000);
#elif defined(ARDUINO_ARCH_RP2040)
  _i2c = &WIRE;
  _i2c->begin();
#elif defined(ARDUINO_ARCH_SAM)
  _i2c = new TwoWire(&PERIPH_WIRE, SDA, SCL);
  _i2c->begin();
#else
#error "I2C bus implementation not supported by this platform!"
#endif

  _bus_status = wippersnapper_i2c_I2cBusStatus_I2C_BUS_STATUS_SUCCESS;
}