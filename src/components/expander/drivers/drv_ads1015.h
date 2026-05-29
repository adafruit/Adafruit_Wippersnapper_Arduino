/*!
 * @file src/components/expander/drivers/drv_ads1015.h
 *
 * Header-only expander driver for the ADS1015 4-channel, 12-bit, ADC.
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
#ifndef WS_EXPANDER_DRV_ADS1015_H
#define WS_EXPANDER_DRV_ADS1015_H
#include "../hardware.h"
#include <Adafruit_ADS1X15.h>

#define ADS1015_RESOLUTION 12 ///< ADC resolution, in bits

/*!
    @brief  Expander driver for the ADS1015 4-channel, 12-bit ADC.
*/
class ExpanderADS1015 : public ExpanderHardware {
public:
  /*!
   * @brief Constructor for the ADS1015 expander driver.
   */
  ExpanderADS1015() {}

  /*!
   * @brief Destructor for the ADS1015 expander driver.
   */
  ~ExpanderADS1015() {}

  uint8_t getAdcResolution() override { return ADS1015_RESOLUTION; }

  /*!
   * @brief Initializes the ADS1015 driver with the given I2C address and bus.
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
   * @param pin The pin number to read from (0-3 for ADS1015).
   * @return The raw ADC value read from the pin.
   */
  uint32_t analogRead(uint8_t pin) override {
    return (uint32_t)(uint16_t)_ads.readADC_SingleEnded(pin);
  }

  /*!
   * @brief Sets the gain for the ADC readings.
   * @param gain The gain setting from ws_analogin_Gain.
   *             Values: 1=1x, 2=2x, 3=4x, 4=8x, 5=16x, 9=2/3x.
   *             Unsupported gains (32x, 64x, 128x) fall back to 1x.
   * @return True if the gain was successfully set, False otherwise.
   */
  bool setGain(uint8_t gain) override {
    adsGain_t ads_gain;
    switch (gain) {
    case 1: // G_1X
      ads_gain = GAIN_ONE;
      break;
    case 2: // G_2X
      ads_gain = GAIN_TWO;
      break;
    case 3: // G_4X
      ads_gain = GAIN_FOUR;
      break;
    case 4: // G_8X
      ads_gain = GAIN_EIGHT;
      break;
    case 5: // G_16X
      ads_gain = GAIN_SIXTEEN;
      break;
    case 9: // G_2_3X
      ads_gain = GAIN_TWOTHIRDS;
      break;
    default:
      ads_gain = GAIN_ONE; // default to 1x gain
      break;
    }
    _ads.setGain(ads_gain);
    return true;
  }

private:
  Adafruit_ADS1015 _ads; ///< Adafruit ADS1015 driver instance
};

#endif // WS_EXPANDER_DRV_ADS1015_H
