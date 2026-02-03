# Sleep Timer Test Patterns

Each section defines a serial output pattern to match during sleep/wake cycle testing.

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

## entered_loop_sleep
- **Pattern**: `\[app\].*Running sleep loop`
- **Case Insensitive**: yes
- **Message**: Running sleep loop

## i2c_events
- **Pattern**: `\[app\].*Processing I2C events`
- **Case Insensitive**: yes
- **Message**: Processing I2C events

## update_complete
- **Pattern**: `completed updates|UpdateComplete.*true`
- **Case Insensitive**: yes
- **Message**: Update complete

## entering_sleep
- **Pattern**: `\[app\].*All components completed.*entering sleep`
- **Case Insensitive**: yes
- **Message**: Entering sleep message

## deep_sleep
- **Pattern**: `\[sleep\].*Disabling SD card before sleep|\[sleep\].*Entering deep sleep`
- **Case Insensitive**: yes
- **Message**: Deep sleep message - device sleeping...

## wake_cause
- **Pattern**: `ESP Reset Reason:\s*(Sleep|Timer)|wake.*cause.*timer|Wakeup caused by timer`
- **Case Insensitive**: yes
- **Message**: Wake cause: ESP_TIMER

## device_reset
- **Pattern**: `Is SD Card initialized\?|Adafruit\.io WipperSnapper`
- **Case Insensitive**: yes
- **Message**: Device reset detected
