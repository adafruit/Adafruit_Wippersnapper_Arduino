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

Example successful timestamp analysis output:

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
