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
  uint32_t analogRead(uint8_t pin) override {
    int16_t value = _ads.readADC_SingleEnded(pin);
    if (value < 0) {
      value = 0;
    }
    return (uint32_t)value;
  }

  /*!
   * @brief Applies a gain setting to the ADS1115.
   * @param gain The gain index (0-5) from the broker's settings, mapping to:
   *             0=2/3x, 1=1x, 2=2x, 3=4x, 4=8x, 5=16x
   * @return True if applied successfully, False if index is out of range.
   */
  bool setGain(int32_t gain) override {
    adsGain_t g;
    switch (gain) {
    case 0:
      g = GAIN_TWOTHIRDS;
      break;
    case 1:
      g = GAIN_ONE;
      break;
    case 2:
      g = GAIN_TWO;
      break;
    case 3:
      g = GAIN_FOUR;
      break;
    case 4:
      g = GAIN_EIGHT;
      break;
    case 5:
      g = GAIN_SIXTEEN;
      break;
    default:
      return false;
    }
    _ads.setGain(g);
    return true;
  }

private:
  Adafruit_ADS1115 _ads; ///< Adafruit ADS1115 driver instance
};

#endif // WS_EXPANDER_DRV_ADS1115_H
