/*!
 * @file src/components/analogIO/hardware.h
 *
 * Hardware implementation for the analogio.proto message.
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
#define DEFAULT_MCU_VREF 3.3      ///< Default reference voltage, in volts

class ExpanderHardware;

/*!
    @brief  Represents a single analog input pin and provides
            hardware-level operations for reading and polling
            its state. Each instance carries its own ADC
            configuration.
*/
class AnalogIOHardware {
public:
  AnalogIOHardware(uint8_t pin_name, ws_sensor_Type read_mode,
                   ws_analogio_SampleMode sample_mode, ulong period,
                   float ref_voltage, ExpanderHardware *expander_drv);
  ~AnalogIOHardware();
  float ReadValue();
  bool CheckEvent();
  bool CheckTimer();

  // Getters
  uint8_t GetPinNum() const;
  ws_sensor_Type GetReadMode() const;
  ws_analogio_SampleMode GetSampleMode() const;
  float GetValue() const;

  // Sleep cycle support
  bool DidReadSend() const;
  void MarkSent();
  void ResetSendFlag();

private:
  uint16_t ReadRawValue();
  float ReadVoltage();
  void InitPin();
  void DeinitPin();
  void SetNativeADCResolution();
  void SetResolution(uint8_t resolution);
  void CalculateScaleFactor();

  // Pin identity and config
  uint8_t _name;                       ///< The pin's number.
  ws_sensor_Type _read_mode;           ///< Type of analog read (RAW or VOLTAGE)
  ws_analogio_SampleMode _sample_mode; ///< Sample mode (TIMER or EVENT)
  ulong _period;                       ///< The pin's period, in milliseconds.
  ulong _prv_time;                     ///< Last read timestamp.
  bool _did_read_send; ///< True if the last read was sent to IO.

  // Values
  uint16_t _value_raw;     ///< Last raw ADC reading.
  float _value_voltage;    ///< Last voltage reading.
  uint16_t _prv_value_raw; ///< Previous raw value for event detection.

  // ADC config (instance members, not static)
  uint8_t _native_adc_resolution;    ///< Hardware's native ADC resolution.
  uint8_t _desired_adc_resolution;   ///< Desired (final) ADC resolution.
  int _max_scale_resolution_desired; ///< Maximum scale resolution desired.
  int _max_scale_resolution_native;  ///< Maximum scale resolution native.
  float _mcu_vref; ///< Reference voltage for reading analog pins.

  // Expander support
  ExpanderHardware *_expander_drv; ///< Pointer to expander driver, or nullptr.
};
#endif // WS_ANALOGIO_HARDWARE_H
