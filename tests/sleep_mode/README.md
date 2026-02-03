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
| `test_timestamp_concurrency.py` | Validates timestamps persist and increment correctly across sleep cycles |
| `test_sleep_timer.py` | Validates timer-based wakeup with configurable duration |
| `test_sleep_ext0.py` | (ESP32-only) Validates EXT0 GPIO button wakeup |

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

### test_timestamp_concurrency.py

Validates that the soft RTC counter persists and increments across sleep cycles.

```bash
# Run with 20 cycles (default)
./env/bin/python -m pytest tests/sleep_mode/test_timestamp_concurrency.py -v -s

# Run with custom cycle count
./env/bin/python -m pytest tests/sleep_mode/test_timestamp_concurrency.py -v -s --cycles 10

# Specify serial port
./env/bin/python -m pytest tests/sleep_mode/test_timestamp_concurrency.py -v -s --port /dev/tty.usbmodem1201
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

## Pattern Files

The timer and EXT0 tests use regex pattern files to match serial output at key checkpoints:

- `sleep_timer_patterns.md` - Patterns for timer wakeup test
- `sleep_ext0_patterns.md` - Patterns for EXT0 wakeup test

## Test Output

Tests capture serial output and validate:

1. **Checkpoints** - Required log messages appear each cycle
2. **Timestamps** - Monotonically increasing, no duplicates
3. **Wake cause** - Correct wakeup source reported (Timer/EXT0)
4. **Timing** - Sleep duration within expected tolerance (timer test)

Example successful timestamp concurrency analysis output:
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
OVERALL: PASS
============================================================
```
