# Sleep EXT0 Test Patterns

Each section defines a serial output pattern to match during EXT0 GPIO wakeup testing.

## sd_init
- **Pattern**: `Is SD Card initialized\?|SD card (initialized|mounted|found)`
- **Case Insensitive**: yes
- **Message**: SD card initialized

## sd_init_confirm
- **Pattern**: `^1$`
- **Case Insensitive**: no
- **Message**: SD card init confirmed

## config_parsed
- **Pattern**: `\[SD\].*Parsing config\.json|Successfully deserialized JSON`
- **Case Insensitive**: yes
- **Message**: Config parsed

## ext0_config
- **Pattern**: `\[sleep\].*EXT0 wakeup set on pin`
- **Case Insensitive**: yes
- **Message**: EXT0 wakeup configured

## entered_loop_sleep
- **Pattern**: `\[app\].*Running sleep loop`
- **Case Insensitive**: yes
- **Message**: Running sleep loop

## analog_events
- **Pattern**: `\[app\].*Processing analog IO events`
- **Case Insensitive**: yes
- **Message**: Processing analog IO events

## i2c_events
- **Pattern**: `\[app\].*Processing I2C events`
- **Case Insensitive**: yes
- **Message**: Processing I2C events

## update_complete
- **Pattern**: `completed updates|UpdateComplete.*true`
- **Case Insensitive**: yes
- **Message**: Update complete

## entering_sleep
- **Pattern**: `\[app\].*All components.*entering sleep`
- **Case Insensitive**: yes
- **Message**: Entering sleep message

## deep_sleep
- **Pattern**: `\[sleep\].*Disabling SD card before sleep|\[sleep\].*Entering deep sleep`
- **Case Insensitive**: yes
- **Message**: Deep sleep - waiting for button press...

## wake_cause_ext0
- **Pattern**: `ESP Reset Reason:\s*EXT0|wake.*cause.*ext|Wakeup caused by external signal.*EXT0`
- **Case Insensitive**: yes
- **Message**: Wake cause: ESP_EXT0

## wake_cause_poweron
- **Pattern**: `ESP Reset Reason:\s*PowerOn|Power.?on reset`
- **Case Insensitive**: yes
- **Message**: Wake cause: PowerOn (first boot)

## device_reset
- **Pattern**: `Is SD Card initialized\?|Adafruit\.io WipperSnapper`
- **Case Insensitive**: yes
- **Message**: Device reset detected
