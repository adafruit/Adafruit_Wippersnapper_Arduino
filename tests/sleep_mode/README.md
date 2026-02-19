# Sleep Mode Test Suite

Hardware tests for validating WipperSnapper deep-sleep functionality on ESP32x and RP2350 platforms.

## Prerequisites

- Python 3.10+
- Device flashed with WipperSnapper v2.0 firmware (or newer)
- `config.json` (for offline mode) must be present on device's filesystem
- USB serial connection to device
- MicroSD card

Dependencies:
- pytest
- pyserial

## Tests

| Test | Description |
|------|-------------|
| `test_rtc_timestamp.py` | Validates soft RTC cycle counter concurrency and persistence across sleep |
| `test_sleep_timer.py` | Validates timer-based wakeup with configurable duration |
| `test_sleep_ext0.py` | (ESP32-only) Validates EXT0 GPIO button wakeup |
| `test_offline_sleep.py` | Validates sleep timing from SD card log files (offline mode) |

## Running Tests

### Options

| Option | Description | Default |
|--------|-------------|---------|
| `--port` | Serial port (or `auto` to detect) | `auto` |
| `--baud` | Baud rate | `115200` |
| `--cycles` / `-C` | Number of wake cycles to capture | `20` |
| `--timeout` | Timeout per cycle in seconds | `60` |
| `--sleep-duration` | Expected sleep duration (timer test only, MUST match config.json `duration` value) | `300` |
| `--patterns` | Path to patterns file (timer/ext0 tests) | auto |
| `--sd-log-file` | Path to JSONL log file from SD card (offline test) | None |
| `--sd-config-file` | Path to config.json from SD card (offline test) | None |
| `--tolerance` | Allowed timing deviation in seconds (offline test) | 20% (min 2s) |
| `--skip-first-cycle` | Exclude first boot cycle from analysis (offline test) | False |
| `--min-cycles` | Minimum wake cycles required (offline test) | `3` |

### test_rtc_timestamp.py

Validates soft RTC cycle counter behavior across deep sleep cycles. Runs two validations:

- **Concurrency**: Timestamps increment correctly within each wake cycle (no duplicates, strictly increasing)
- **Persistence**: Cycle counter persists across deep sleep using RTC_SLOW_ATTR memory (no resets to 0, counter continues incrementing)

Note: The soft RTC is a **cycle counter** (increments per reading), not wall-clock time.

```bash
# Run with 20 cycles (default)
./env/bin/python -m pytest tests/sleep_mode/test_rtc_timestamp.py -v -s

# Run with custom cycle count
./env/bin/python -m pytest tests/sleep_mode/test_rtc_timestamp.py -v -s --cycles 10

# Specify serial port
./env/bin/python -m pytest tests/sleep_mode/test_rtc_timestamp.py -v -s --port /dev/tty.usbmodem1201
```

### test_sleep_timer.py

Validates timer-based deep sleep wakeup with timing accuracy checks.

NOTE: config.json's `duration` value must match `sleep-duration` argument exactly.

```bash
# Run with 4 cycles, 5-minute sleep duration
./env/bin/python -m pytest tests/sleep_mode/test_sleep_timer.py -v -s --cycles 4 --sleep-duration 300

# Run with shorter sleep for faster testing 
./env/bin/python -m pytest tests/sleep_mode/test_sleep_timer.py -v -s --cycles 4 --sleep-duration 60
```

### test_sleep_ext0.py

Validates GPIO pin wakeup (EXT0). Requires manual pin triggering.
NOTE: This test is ESP32-only.
NOTE: Ensure `config.json` is set up for EXT0 wakeup on the desired pin.

```bash
# Run with 4 cycles, user triggers pin each cycle
./env/bin/python -m pytest tests/sleep_mode/test_sleep_ext0.py -v -s --cycles 4
```

### test_offline_sleep.py

Validates sleep timing by analyzing JSONL log files from an SD card. Works with any board running in Offline Mode (RP2350, ESP32, etc.). This test does **not** require a serial connection - it reads log files after the device has completed its sleep/wake cycles.

**Workflow:**
1. Configure device with `config.json` (set RTC type, sleep duration, sensors)
2. Run device through multiple sleep/wake cycles (data logs to SD card)
3. Remove SD card and mount on computer
4. Run the test against the log files

**RTC Types:**
- **Hardware RTC** (DS3231, DS1307, PCF8523): Validates sleep duration by detecting timestamp gaps
- **Soft RTC**: Validates timestamp monotonicity only (counter-based, cannot measure sleep time)

```bash
# Basic usage - reads sleep duration from config.json
./env/bin/python -m pytest tests/sleep_mode/test_offline_sleep.py -v -s \
    --sd-log-file /Volumes/SDCARD/log_123.log \
    --sd-config-file /Volumes/SDCARD/config.json

# Override sleep duration and tolerance
./env/bin/python -m pytest tests/sleep_mode/test_offline_sleep.py -v -s \
    --sd-log-file /Volumes/SDCARD/log_123.log \
    --sd-config-file /Volumes/SDCARD/config.json \
    --sleep-duration 12 \
    --tolerance 5

# Require more cycles and skip first boot
./env/bin/python -m pytest tests/sleep_mode/test_offline_sleep.py -v -s \
    --sd-log-file /Volumes/SDCARD/log_123.log \
    --sd-config-file /Volumes/SDCARD/config.json \
    --min-cycles 5 \
    --skip-first-cycle

# Standalone mode (no pytest required, uses --log-file and --config-file)
python tests/sleep_mode/test_offline_sleep.py \
    --log-file /Volumes/SDCARD/log_123.log \
    --config-file /Volumes/SDCARD/config.json
```

**Example config.json structure (Adafruit Metro ESP32-S3 and BME280):**
```json
{
    "exportedFromDevice": {
        "referenceVoltage": 2.6,
        "totalGPIOPins": 27,
        "totalAnalogPins": 18,
        "statusLEDBrightness": 0.5,
        "autoConfig": false,
        "sd_cs_pin": 45
    },
    "components": [
        {
            "name": "BME280",
            "componentAPI": "i2c",
            "period": 30,
            "i2cDeviceName": "bme280",
            "i2cDeviceAddress": "0x77",
            "i2cDeviceSensorTypes": [
                {
                    "type": "ambient-temp"
                },
                {
                    "type": "relative-humidity"
                },
                {
                    "type": "pressure"
                }
            ],
            "autoConfig": "false"
        }
    ],
    "sleepConfig": [
        {
            "lock": true,
            "mode": "deep",
            "runDuration": 60,
            "timerConfig": {
                "duration": 300
            }
        }
    ]
}
```

**Expected log file format (JSONL):**
```json
{"timestamp": 1770819104, "i2c_address": "0x77", "value": 25.5, "si_unit": "celsius"}
{"timestamp": 1770819105, "i2c_address": "0x77", "value": 1013.2, "si_unit": "hPa"}
```

## Pattern Files

The timer and EXT0 tests use regex pattern files to match serial output at key checkpoints:

- `sleep_timer_patterns.md` - Patterns for timer wakeup test
- `sleep_ext0_patterns.md` - Patterns for EXT0 wakeup test

## Test Output

Tests capture serial output and validate:

1. **Checkpoints** - Required log messages appear each cycle
2. **Concurrency** - Timestamps monotonically increasing within cycles, no duplicates
3. **Persistence** - Timestamps persist across sleep (no resets), advance by sleep duration
4. **Wake cause** - Correct wakeup source reported (Timer/EXT0)
5. **Timing** - Sleep duration within expected tolerance (timer test)

Example successful timestamp analysis output (test_rtc_timestamp.py):

```
============================================================
TIMESTAMP CONCURRENCY ANALYSIS
============================================================
Total Entries: 20
Timestamp Range: 0 - 19
Unique Timestamps: 20
------------------------------------------------------------
[PASS] All timestamps are monotonically increasing
[PASS] No duplicate timestamps
------------------------------------------------------------
PER-SOURCE TIMESTAMP GAPS:
  pin:A0: avg_gap=1.00, count=20
------------------------------------------------------------
CROSS-CYCLE PERSISTENCE (cycle counter):
[PASS] No timestamp resets detected
[PASS] All 19 cross-cycle gaps valid (counter incremented)
  Gap details:
  ✓ Cycle 0→1: ts 0→1 (delta=+1)
  ✓ Cycle 1→2: ts 1→2 (delta=+1)
  ...
============================================================
Concurrency: PASS
Persistence: PASS
============================================================
```

Example successful offline sleep test output (test_offline_sleep.py with hardware RTC):

```
============================================================
OFFLINE MODE LOG FILE VALIDATION
============================================================
Log File: /Volumes/SDCARD/log_1770819104.log
Config: /Volumes/SDCARD/config.json
RTC Type: DS3231 (Hardware RTC)
Expected Sleep Duration: 12s ± 5s
------------------------------------------------------------
LOG FILE:
  Total Entries: 36
  First Timestamp: 1770819104 (2026-02-10 14:25:04)
  Last Timestamp: 1770819519 (2026-02-10 14:31:59)
------------------------------------------------------------
SLEEP GAPS DETECTED: 3
  [PASS] Gap 1→2: 12s (expected 7-17s)
  [PASS] Gap 2→3: 11s (expected 7-17s)
  [PASS] Gap 3→4: 12s (expected 7-17s)
------------------------------------------------------------
STATISTICS:
  Average: 11.7s | Min: 11s | Max: 12s | StdDev: 0.5s
------------------------------------------------------------
VALIDATION:
  [PASS] Timestamps monotonically increasing
  [PASS] No duplicate entries
  [PASS] All sleep gaps within tolerance
  [PASS] Minimum cycles (4 >= 3)
============================================================
RESULT: PASS
============================================================
```

Example output with soft RTC (cycle counter):

```
============================================================
OFFLINE MODE LOG FILE VALIDATION
============================================================
Log File: /Volumes/SDCARD/log_0.log
Config: /Volumes/SDCARD/config.json
RTC Type: SOFT (Cycle Counter)
------------------------------------------------------------
LOG FILE:
  Total Entries: 156
  First Timestamp: 0
  Last Timestamp: 155
------------------------------------------------------------
VALIDATION:
  [PASS] Timestamps monotonically increasing (0 → 155)
  [PASS] No duplicate entries
  [PASS] No timestamp resets detected
  [INFO] Sleep timing validation skipped (soft RTC is counter-based)
============================================================
RESULT: PASS
============================================================
```
