/*!
 * @file hardware.cpp
 *
 * Hardware interface for the ds18x20.proto API
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2024 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "hardware.h"

/***********************************************************************/
/*!
    @brief  DS18X20Hardware constructor
    @param  onewire_pin
            The OneWire bus pin to use.
*/
/***********************************************************************/
DS18X20Hardware::DS18X20Hardware(uint8_t onewire_pin) : _drv_therm(_ow) {
  is_read_temp_c = false;
  is_read_temp_f = false;
  // Initialize the OneWire bus object
  _onewire_pin = onewire_pin;
  new (&_ow) OneWireNg_CurrentPlatform(onewire_pin, false);
}

/***********************************************************************/
/*!
    @brief  DS18X20Hardware destructor
*/
/***********************************************************************/
DS18X20Hardware::~DS18X20Hardware() {
  pinMode(_onewire_pin,
          INPUT); // Set the pin to hi-z and release it for other uses
}

void DS18X20Hardware::printErrorCode(OneWireNg::ErrorCode ec) {
  switch (ec) {
  case OneWireNg::EC_SUCCESS:
    WS_DEBUG_PRINTLN("EC_SUCCESS");
    break;
  case OneWireNg::EC_NO_DEVS:
    WS_DEBUG_PRINTLN("EC_NO_DEVSNo slave devices; search process finished");
    break;
  case OneWireNg::EC_BUS_ERROR:
    WS_DEBUG_PRINTLN("EC_BUS_ERROR-wire bus error");
    break;
  case OneWireNg::EC_CRC_ERROR:
    WS_DEBUG_PRINTLN("EC_CRC_ERRORCRC error");
    break;
  case OneWireNg::EC_UNSUPPORED:
    WS_DEBUG_PRINTLN("EC_UNSUPPOREDService is not supported by the platform");
    break;
  case OneWireNg::EC_FULL:
    WS_DEBUG_PRINTLN("EC_FULLNo space (e.g. filters table is full)");
    break;
  default:
    WS_DEBUG_PRINTLN("EC ?!?1 Unknown error");
    break;
  }
}

/***********************************************************************/
/*!
    @brief  Get the sensor's ID
    @returns True if the sensor was successfully identified, False otherwise.
*/
/***********************************************************************/
bool DS18X20Hardware::GetSensor() {
  OneWireNg::ErrorCode ec = _ow->readSingleId(_sensorId);
  printErrorCode(ec);
  return ec == OneWireNg::EC_SUCCESS;
}

/***********************************************************************/
/*!
    @brief  Gets the pin used as a OneWire bus.
    @returns The OneWire bus pin.
*/
/***********************************************************************/
uint8_t DS18X20Hardware::GetOneWirePin() { return _onewire_pin; }

/*************************************************************************/
/*!
    @brief  Sets the DS18X20 sensor's resolution.
    @param  resolution
            The desired resolution of the DS18X20 sensor, in bits (from
            9 to 12).
*/
/*************************************************************************/
void DS18X20Hardware::SetResolution(int resolution) {
  // Set the resolution of the DS18X20 sensor driver
  switch (resolution) {
  case 9:
    _resolution = DSTherm::Resolution::RES_9_BIT;
    break;
  case 10:
    _resolution = DSTherm::Resolution::RES_10_BIT;
    break;
  case 11:
    _resolution = DSTherm::Resolution::RES_11_BIT;
    break;
  case 12:
    _resolution = DSTherm::Resolution::RES_12_BIT;
    break;
  default:
    _resolution =
        DSTherm::Resolution::RES_12_BIT; // Default to 12-bit resolution
    break;
  }

  // Set common resolution for all sensors.
  // Th, Tl (high/low alarm triggers) are set to 0.
  _drv_therm.writeScratchpadAll(0, 0, _resolution);

  // The configuration above is stored in volatile sensors scratchpad
  // memory and will be lost after power unplug. Therefore store the
  // configuration permanently in sensors EEPROM.
  _drv_therm.copyScratchpadAll(false);
}

/*************************************************************************/
/*!
    @brief  Sets the timer to read from the sensor.
    @param  period
            The desired period to read the sensor, in seconds.
*/
/*************************************************************************/
void DS18X20Hardware::SetPeriod(float period) {
  _period = period * 1000; // Convert to milliseconds
  _prv_period = 0;         // Also reset the previous period here
}

/*************************************************************************/
/*!
    @brief  Obtains the current time in milliseconds and compares it to
            the last time the sensor was polled.
    @returns True if the timer has expired, False otherwise.
*/
/*************************************************************************/
bool DS18X20Hardware::IsTimerExpired() {
  return millis() - _prv_period > _period;
}

/*************************************************************************/
/*!
    @brief  Gets the temperature value last read by the sensor, in Celsius.
    @returns The temperature in Celsius.
*/
/*************************************************************************/
float DS18X20Hardware::GetTemperatureC() { return _temp_c; }

/*************************************************************************/
/*!
    @brief  Gets the temperature value last read by the sensor, in Fahrenheit.
    @returns The temperature in Fahrenheit.
*/
/*************************************************************************/
float DS18X20Hardware::GetTemperatureF() { 
    _temp_f = _temp_c * 9.0 / 5.0 + 32.0;
    return _temp_f;
}

/*************************************************************************/
/*!
    @brief  Attempts to obtain the temperature from the sensor, in
            degrees Celsius.
    @returns True if the temperature was successfully read, False otherwise.
*/
/*************************************************************************/
bool DS18X20Hardware::ReadTemperatureC() {
  // Start temperature conversion for the first identified sensor on the OneWire
  // bus
  OneWireNg::ErrorCode ec =
      _drv_therm.convertTemp(_sensorId, DSTherm::MAX_CONV_TIME, false);
  if (ec != OneWireNg::EC_SUCCESS)
    return false;

  // Scratchpad placeholder is static to allow reuse of the associated
  // sensor id while reissuing readScratchpadSingle() calls.
  // Note, due to its storage class the placeholder is zero initialized.
  static Placeholder<DSTherm::Scratchpad> scrpd;
  ec = _drv_therm.readScratchpadSingle(scrpd);
  if (ec != OneWireNg::EC_SUCCESS)
    return false;

  // Read the temperature from the sensor
  long temp = scrpd->getTemp2();
  _temp_c = temp / 16.0; // Convert from 16-bit int to float

  _prv_period = millis(); // Update the last time the sensor was polled
  return true;
}