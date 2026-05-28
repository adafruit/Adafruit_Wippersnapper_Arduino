/*!
 * @file src/components/expander/drivers/drv_ads1115.h
 *
 * Header-only expander driver for the ADS1115 4-channel, 16-bit, ADC.
 * Wraps the Adafruit_ADS1X15 library.
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
#ifndef WS_EXPANDER_DRV_ADS1115_H
#define WS_EXPANDER_DRV_ADS1115_H
#include "../hardware.h"
#include <Adafruit_ADS1X15.h>

#define ADS1115_RESOLUTION 16 ///< ADC resolution, in bits

/*!
    @brief  Expander driver for the ADS1115 4-channel, 16-bit ADC.
*/
class ExpanderADS1115 : public ExpanderHardware {
public:
  /*!
   * @brief Constructor for the ADS1115 expander driver.
   */
  ExpanderADS1115() {}

  /*!
   * @brief Destructor for the ADS1115 expander driver.
   */
  ~ExpanderADS1115() {}

  uint8_t getAdcResolution() override { return ADS1115_RESOLUTION; }

  /*!
   * @brief Initializes the ADS1115 driver with the given I2C address and bus.
   * @param i2c_addr The I2C address of the expander.
   * @param wire The TwoWire I2C bus to use for communication.
   * @return True if initialization was successful, False otherwise.
   */
  bool begin(uint8_t i2c_addr, TwoWire *wire) override {
    _i2c_addr = i2c_addr;
    return _ads.begin(i2c_addr, wire);
  }

  /*!
   * @brief Reads the raw ADC value from the specified pin.
   * @param pin The pin number to read from (0-3 for ADS1115).
   * @return The raw ADC value read from the pin.
   */
  uint16_t analogRead(uint8_t pin) override {
    return _ads.readADC_SingleEnded(pin);
  }

private:
  Adafruit_ADS1115 _ads; ///< Adafruit ADS1115 driver instance
};

#endif // WS_EXPANDER_DRV_ADS1115_H
