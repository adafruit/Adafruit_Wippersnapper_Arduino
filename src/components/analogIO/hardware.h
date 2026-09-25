/*!
 * @file src/components/analogIO/hardware.h
 *
 * Hardware implementation for the analogin.proto message.
 * Each instance represents a single analog input pin and
 * carries its own ADC configuration as instance members.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2024-2026 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_ANALOGIO_HARDWARE_H
#define WS_ANALOGIO_HARDWARE_H
#include "wippersnapper.h"

#define DEFAULT_ADC_RESOLUTION 16 ///< Default ADC resolution, in bits
#define MAX_ADC_RESOLUTION 24 ///< Maximum configurable ADC resolution, in bits
#define DEFAULT_MCU_VREF 3.3  ///< Default reference voltage, in volts

class ExpanderHardware;

/*!
    @brief  Represents a single analog input pin and provides
            hardware-level operations for reading and polling
            its state. Each instance carries its own ADC
            configuration.
*/
class AnalogIOHardware {
public:
  AnalogIOHardware(const char *pin_name, uint8_t pin_num,
                   ws_sensor_Type read_mode, ws_analogin_SampleMode sample_mode,
                   ulong period, float ref_voltage,
                   ExpanderHardware *expander_drv);
  ~AnalogIOHardware();
  float readValue();
  bool checkEvent();
  bool checkTimer();
  uint8_t getPinNum() const;
  const char *getPinName() const;
  ws_sensor_Type getReadMode() const;
  ws_analogin_SampleMode getSampleMode() const;
  float getValue() const;
  ExpanderHardware *getExpander() const;
  bool didReadSend() const;
  void markSent();
  void resetSendFlag();

private:
  uint32_t readValueRaw();
  float readVoltage();
  void init();
  void deinit();
  void setAdcResolutionNative();
  void setAdcResolution(uint8_t resolution);
  void getScaleFactor();
  uint8_t _name; ///< The pin's number.
  char _pin_name[sizeof(
      ws_analogin_Add::pin_name)];     ///< Broker-provided pin name.
  ws_sensor_Type _read_mode;           ///< Type of analog read (RAW or VOLTAGE)
  ws_analogin_SampleMode _sample_mode; ///< Sample mode (TIMER or EVENT)
  ulong _period;                       ///< The pin's period, in milliseconds.
  ulong _prv_time;                     ///< Last read timestamp.
  bool _did_read_send;             ///< True if the last read was sent to IO.
  uint32_t _value_raw;             ///< Last raw ADC reading.
  float _value_voltage;            ///< Last voltage reading.
  uint32_t _prv_value_raw;         ///< Previous raw value for event detection.
  uint8_t _native_adc_resolution;  ///< Hardware's native ADC resolution.
  uint8_t _desired_adc_resolution; ///< Desired (final) ADC resolution.
  uint32_t _max_scale_resolution_desired; ///< Maximum scale resolution desired.
  uint32_t _max_scale_resolution_native;  ///< Maximum scale resolution native.
  float _ref_voltage; ///< Reference voltage for reading this pin (MCU vref for
                      ///< native pins, message vref for expander pins).
  ExpanderHardware *_expander_drv; ///< Pointer to expander driver, or nullptr.
};
#endif // WS_ANALOGIO_HARDWARE_H
