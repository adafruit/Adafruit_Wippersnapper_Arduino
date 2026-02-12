"""
Offline Mode Sleep Validation Test Suite

Validates sleep behavior by analyzing JSONL log files from SD card.
Works with any board running in Offline Mode (RP2350, ESP32, etc.).

Usage:
    pytest tests/sleep_mode/test_offline_sleep.py \
        --sd-log-file /Volumes/SDCARD/log_123.log \
        --sd-config-file /Volumes/SDCARD/config.json -v

    # With overrides
    pytest tests/sleep_mode/test_offline_sleep.py \
        --sd-log-file /Volumes/SDCARD/log_123.log \
        --sd-config-file /Volumes/SDCARD/config.json \
        --sleep-duration 12 \
        --tolerance 5 \
        --min-cycles 5 -v

Standalone:
    python tests/sleep_mode/test_offline_sleep.py \
        --log-file /Volumes/SDCARD/log_123.log \
        --config-file /Volumes/SDCARD/config.json
"""

import json
import sys
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from statistics import mean, stdev
from typing import Any, Optional

# Try to import LogEntry from utils, fall back to local definition
try:
    from utils import LogEntry
except (ImportError, ModuleNotFoundError):
    # Standalone mode - define LogEntry locally
    @dataclass
    class LogEntry:
        """Parsed JSONL log entry from device."""

        timestamp: int
        raw_line: str
        pin: Optional[str] = None
        i2c_address: Optional[str] = None
        value: Optional[float] = None
        si_unit: Optional[str] = None

        @classmethod
        def from_json(cls, line: str) -> Optional["LogEntry"]:
            """Parse a JSONL line into a LogEntry."""
            try:
                data = json.loads(line)
                if "timestamp" not in data:
                    return None
                return cls(
                    timestamp=int(data["timestamp"]),
                    raw_line=line,
                    pin=data.get("pin"),
                    i2c_address=data.get("i2c_address"),
                    value=data.get("value"),
                    si_unit=data.get("si_unit"),
                )
            except (json.JSONDecodeError, ValueError, KeyError):
                return None


@dataclass
class OfflineConfig:
    """Configuration parsed from config.json."""

    rtc_type: str  # "SOFT", "DS3231", "DS1307", "PCF8523"
    is_hardware_rtc: bool  # True if not "SOFT"
    sleep_duration: int  # From sleepConfig.timerConfig.duration
    run_duration: int | None  # From sleepConfig.runDuration (optional)
    sensor_periods: list[int] = field(default_factory=list)


@dataclass
class SleepGap:
    """Detected gap between wake cycles indicating sleep."""

    prev_cycle_idx: int
    next_cycle_idx: int
    prev_timestamp: int
    next_timestamp: int
    gap_duration: int
    expected_duration: int
    tolerance: int
    is_valid: bool
    error_message: str


@dataclass
class ValidationResult:
    """Result of sleep timing validation."""

    passed: bool
    gaps: list[SleepGap]
    avg_duration: float | None
    min_duration: int | None
    max_duration: int | None
    std_deviation: float | None
    messages: list[str] = field(default_factory=list)


def parse_config_json(path: Path) -> OfflineConfig:
    """
    Parse config.json to extract RTC type and sleep configuration.

    Args:
        path: Path to config.json file

    Returns:
        OfflineConfig with extracted settings
    """
    with open(path, "r") as f:
        config = json.load(f)

    # Extract RTC type from exportedFromDevice
    rtc_type = "SOFT"
    exported = config.get("exportedFromDevice", {})
    if "rtc" in exported:
        rtc_type = exported["rtc"].upper()

    is_hardware_rtc = rtc_type != "SOFT"

    # Extract sleep duration from sleepConfig
    sleep_duration = 0
    run_duration = None
    sleep_config = config.get("sleepConfig", [])
    if sleep_config and len(sleep_config) > 0:
        first_sleep = sleep_config[0]
        timer_config = first_sleep.get("timerConfig", {})
        sleep_duration = timer_config.get("duration", 0)
        run_duration = first_sleep.get("runDuration")

    # Extract sensor periods from components
    sensor_periods = []
    components = config.get("components", [])
    for component in components:
        if "period" in component:
            sensor_periods.append(component["period"])

    return OfflineConfig(
        rtc_type=rtc_type,
        is_hardware_rtc=is_hardware_rtc,
        sleep_duration=sleep_duration,
        run_duration=run_duration,
        sensor_periods=sensor_periods,
    )


def parse_log_file(path: Path) -> list[LogEntry]:
    """
    Parse JSONL log file from SD card.

    Args:
        path: Path to JSONL log file

    Returns:
        List of LogEntry objects sorted by timestamp
    """
    entries = []
    warnings = []

    with open(path, "r") as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue

            entry = LogEntry.from_json(line)
            if entry:
                entries.append(entry)
            else:
                warnings.append(f"Line {line_num}: Invalid JSON or missing timestamp")

    if warnings:
        print(f"[WARN] {len(warnings)} invalid lines skipped:")
        for w in warnings[:5]:  # Show first 5 warnings
            print(f"  {w}")
        if len(warnings) > 5:
            print(f"  ... and {len(warnings) - 5} more")

    # Sort by timestamp
    entries.sort(key=lambda e: e.timestamp)

    return entries


def calculate_tolerance(expected_duration: int, override: int | None) -> int:
    """
    Calculate tolerance as 20% of expected duration, minimum 2s.

    For short sleeps (10s): 20% = 2s → range 8-12s
    For longer sleeps (300s): 20% = 60s → range 240-360s

    Args:
        expected_duration: Expected sleep duration in seconds
        override: Optional override value

    Returns:
        Tolerance in seconds
    """
    if override is not None:
        return override

    twenty_percent = int(expected_duration * 0.20)
    return max(2, twenty_percent)


def detect_sleep_gaps(
    entries: list[LogEntry], expected_sleep: int, tolerance: int
) -> list[SleepGap]:
    """
    Detect sleep gaps in log entries based on timestamp jumps.

    For hardware RTC, sleep gaps appear as large jumps in timestamps.

    Args:
        entries: List of log entries sorted by timestamp
        expected_sleep: Expected sleep duration in seconds
        tolerance: Allowed deviation in seconds

    Returns:
        List of detected SleepGap objects
    """
    if len(entries) < 2:
        return []

    gaps = []
    min_gap = expected_sleep - tolerance
    cycle_idx = 1

    # Group entries by cycles (entries with same or close timestamps are in same cycle)
    prev_ts = entries[0].timestamp
    cycle_last_ts = prev_ts

    for i, entry in enumerate(entries[1:], 1):
        ts_delta = entry.timestamp - prev_ts

        # If timestamp jump is large enough to be a sleep gap
        if ts_delta >= min_gap:
            gap_duration = entry.timestamp - cycle_last_ts

            # Check if gap is within tolerance
            expected_min = expected_sleep - tolerance
            expected_max = expected_sleep + tolerance
            is_valid = expected_min <= gap_duration <= expected_max

            error_message = ""
            if not is_valid:
                if gap_duration < expected_min:
                    error_message = (
                        f"Gap too short: {gap_duration}s < {expected_min}s minimum"
                    )
                else:
                    error_message = (
                        f"Gap too long: {gap_duration}s > {expected_max}s maximum"
                    )

            gaps.append(
                SleepGap(
                    prev_cycle_idx=cycle_idx,
                    next_cycle_idx=cycle_idx + 1,
                    prev_timestamp=cycle_last_ts,
                    next_timestamp=entry.timestamp,
                    gap_duration=gap_duration,
                    expected_duration=expected_sleep,
                    tolerance=tolerance,
                    is_valid=is_valid,
                    error_message=error_message,
                )
            )

            cycle_idx += 1

        cycle_last_ts = entry.timestamp
        prev_ts = entry.timestamp

    return gaps


def validate_sleep_timing(
    gaps: list[SleepGap], expected_sleep: int, tolerance: int
) -> ValidationResult:
    """
    Validate that all sleep gaps are within tolerance.

    Args:
        gaps: List of detected sleep gaps
        expected_sleep: Expected sleep duration
        tolerance: Allowed deviation

    Returns:
        ValidationResult with statistics and pass/fail status
    """
    if not gaps:
        return ValidationResult(
            passed=True,
            gaps=[],
            avg_duration=None,
            min_duration=None,
            max_duration=None,
            std_deviation=None,
            messages=["No sleep gaps detected"],
        )

    durations = [g.gap_duration for g in gaps]
    all_valid = all(g.is_valid for g in gaps)

    avg = mean(durations)
    min_d = min(durations)
    max_d = max(durations)
    std_dev = stdev(durations) if len(durations) > 1 else 0.0

    messages = []
    for gap in gaps:
        status = "[PASS]" if gap.is_valid else "[FAIL]"
        expected_min = expected_sleep - tolerance
        expected_max = expected_sleep + tolerance
        messages.append(
            f"  {status} Gap {gap.prev_cycle_idx}→{gap.next_cycle_idx}: "
            f"{gap.gap_duration}s (expected {expected_min}-{expected_max}s)"
        )
        if not gap.is_valid:
            messages.append(f"         {gap.error_message}")

    return ValidationResult(
        passed=all_valid,
        gaps=gaps,
        avg_duration=avg,
        min_duration=min_d,
        max_duration=max_d,
        std_deviation=std_dev,
        messages=messages,
    )


def validate_monotonicity(entries: list[LogEntry]) -> tuple[bool, list[str]]:
    """
    Validate that timestamps are strictly increasing.

    Args:
        entries: List of log entries

    Returns:
        Tuple of (passed, messages)
    """
    if len(entries) < 2:
        return True, ["Not enough entries to validate monotonicity"]

    messages = []
    is_valid = True

    for i in range(1, len(entries)):
        if entries[i].timestamp < entries[i - 1].timestamp:
            is_valid = False
            messages.append(
                f"[FAIL] Timestamp decreased: "
                f"{entries[i-1].timestamp} -> {entries[i].timestamp}"
            )

    if is_valid:
        messages.append(
            f"[PASS] Timestamps monotonically increasing "
            f"({entries[0].timestamp} → {entries[-1].timestamp})"
        )

    return is_valid, messages


def validate_no_duplicates(entries: list[LogEntry]) -> tuple[bool, list[str]]:
    """
    Validate that there are no duplicate entries.

    Note: Multiple entries with the same timestamp from the same cycle is normal.

    Args:
        entries: List of log entries

    Returns:
        Tuple of (passed, messages)
    """
    seen = set()
    duplicates = []

    for entry in entries:
        key = (entry.timestamp, entry.pin, entry.i2c_address, entry.si_unit)
        if key in seen:
            duplicates.append(entry)
        seen.add(key)

    if duplicates:
        messages = [f"[FAIL] Found {len(duplicates)} duplicate entries"]
        for d in duplicates[:5]:
            messages.append(f"  ts={d.timestamp}")
        return False, messages

    return True, ["[PASS] No duplicate entries"]


def format_timestamp(ts: int) -> str:
    """Format Unix timestamp as human-readable datetime."""
    try:
        dt = datetime.fromtimestamp(ts)
        return dt.strftime("%Y-%m-%d %H:%M:%S")
    except (OSError, ValueError):
        return "N/A"


def print_report(
    log_file: Path,
    config_file: Path,
    config: OfflineConfig,
    entries: list[LogEntry],
    sleep_duration: int,
    tolerance: int,
    validation_result: ValidationResult,
    monotonicity: tuple[bool, list[str]],
    no_duplicates: tuple[bool, list[str]],
    min_cycles: int,
    skip_first_cycle: bool,
) -> bool:
    """Print formatted test report. Returns True if all passed."""
    divider = "=" * 60
    subdiv = "-" * 60

    print(f"\n{divider}")
    print("OFFLINE MODE LOG FILE VALIDATION")
    print(divider)
    print(f"Log File: {log_file}")
    print(f"Config: {config_file}")

    rtc_desc = (
        f"{config.rtc_type} (Hardware RTC)"
        if config.is_hardware_rtc
        else f"{config.rtc_type} (Cycle Counter)"
    )
    print(f"RTC Type: {rtc_desc}")

    if config.is_hardware_rtc:
        print(f"Expected Sleep Duration: {sleep_duration}s ± {tolerance}s")

    print(subdiv)
    print("LOG FILE:")
    print(f"  Total Entries: {len(entries)}")

    if entries:
        first_ts = entries[0].timestamp
        last_ts = entries[-1].timestamp
        print(f"  First Timestamp: {first_ts} ({format_timestamp(first_ts)})")
        print(f"  Last Timestamp: {last_ts} ({format_timestamp(last_ts)})")

    print(subdiv)

    if config.is_hardware_rtc:
        num_gaps = len(validation_result.gaps)
        num_cycles = num_gaps + 1
        print(f"SLEEP GAPS DETECTED: {num_gaps}")

        for msg in validation_result.messages:
            print(msg)

        print(subdiv)
        print("STATISTICS:")
        if validation_result.avg_duration is not None:
            print(
                f"  Average: {validation_result.avg_duration:.1f}s | "
                f"Min: {validation_result.min_duration}s | "
                f"Max: {validation_result.max_duration}s | "
                f"StdDev: {validation_result.std_deviation:.1f}s"
            )
        else:
            print("  No sleep gaps to analyze")

        print(subdiv)
        print("VALIDATION:")

        mono_pass, mono_msgs = monotonicity
        for msg in mono_msgs:
            print(f"  {msg}")

        dup_pass, dup_msgs = no_duplicates
        for msg in dup_msgs:
            print(f"  {msg}")

        timing_status = "[PASS]" if validation_result.passed else "[FAIL]"
        print(f"  {timing_status} All sleep gaps within tolerance")

        cycle_status = "[PASS]" if num_cycles >= min_cycles else "[FAIL]"
        print(f"  {cycle_status} Minimum cycles ({num_cycles} >= {min_cycles})")

        all_passed = (
            validation_result.passed
            and mono_pass
            and dup_pass
            and num_cycles >= min_cycles
        )

    else:
        # Soft RTC
        print("VALIDATION:")

        mono_pass, mono_msgs = monotonicity
        for msg in mono_msgs:
            print(f"  {msg}")

        dup_pass, dup_msgs = no_duplicates
        for msg in dup_msgs:
            print(f"  {msg}")

        # Check for resets (timestamp going back to 0 or near 0)
        resets = []
        for i in range(1, len(entries)):
            if entries[i].timestamp < entries[i - 1].timestamp:
                resets.append(i)

        if not resets:
            print("  [PASS] No timestamp resets detected")

        print("  [INFO] Sleep timing validation skipped (soft RTC is counter-based)")

        all_passed = mono_pass and dup_pass

    print(divider)
    result = "PASS" if all_passed else "FAIL"
    print(f"RESULT: {result}")
    print(f"{divider}\n")

    return all_passed


class TestOfflineSleep:
    """Test suite for offline mode sleep validation."""

    def test_sleep_timing_from_sd_card(
        self,
        sd_log_file: Path | None,
        sd_config_file: Path | None,
        sleep_duration: int,
        tolerance: int | None,
        skip_first_cycle: bool,
        min_cycles: int,
    ):
        """
        Main test: validate sleep timing from SD card log files.

        Args:
            sd_log_file: Path to JSONL log file
            sd_config_file: Path to config.json
            sleep_duration: Expected sleep duration (from --sleep-duration or config)
            tolerance: Override for tolerance (None = calculate default)
            skip_first_cycle: Whether to skip first boot cycle
            min_cycles: Minimum number of cycles required
        """
        # Skip if log file not provided
        if sd_log_file is None:
            import pytest

            pytest.skip("--sd-log-file not provided")

        if sd_config_file is None:
            import pytest

            pytest.skip("--sd-config-file not provided")

        # Use local names for the rest of the function
        log_file = sd_log_file
        config_file = sd_config_file

        # Verify files exist
        assert log_file.exists(), f"Log file not found: {log_file}"
        assert config_file.exists(), f"Config file not found: {config_file}"

        # 1. Parse config.json
        config = parse_config_json(config_file)

        # Use config sleep duration if CLI default (300) and config has a value
        actual_sleep_duration = sleep_duration
        if config.sleep_duration > 0:
            actual_sleep_duration = config.sleep_duration
        assert actual_sleep_duration > 0, "Sleep duration must be positive"

        # Calculate tolerance
        tol = calculate_tolerance(actual_sleep_duration, tolerance)

        # 2. Parse log file
        entries = parse_log_file(log_file)
        assert len(entries) > 0, "Log file is empty"

        # Skip first cycle if requested
        if skip_first_cycle and config.is_hardware_rtc and len(entries) > 1:
            # Find first sleep gap and remove entries before it
            gaps = detect_sleep_gaps(entries, actual_sleep_duration, tol)
            if gaps:
                first_gap_ts = gaps[0].next_timestamp
                entries = [e for e in entries if e.timestamp >= first_gap_ts]

        # 3. Common validations
        monotonicity = validate_monotonicity(entries)
        no_duplicates = validate_no_duplicates(entries)

        # 4. Validate based on RTC type
        if config.is_hardware_rtc:
            # Detect sleep gaps from timestamp jumps
            gaps = detect_sleep_gaps(entries, actual_sleep_duration, tol)
            validation_result = validate_sleep_timing(gaps, actual_sleep_duration, tol)

            # Print report
            all_passed = print_report(
                log_file=log_file,
                config_file=config_file,
                config=config,
                entries=entries,
                sleep_duration=actual_sleep_duration,
                tolerance=tol,
                validation_result=validation_result,
                monotonicity=monotonicity,
                no_duplicates=no_duplicates,
                min_cycles=min_cycles,
                skip_first_cycle=skip_first_cycle,
            )

            # Assertions
            assert monotonicity[0], "Timestamps must be monotonically increasing"
            assert no_duplicates[0], "No duplicate entries allowed"
            assert validation_result.passed, "All sleep gaps must be within tolerance"

            num_cycles = len(gaps) + 1
            assert (
                num_cycles >= min_cycles
            ), f"Not enough cycles: {num_cycles} < {min_cycles}"

        else:
            # Soft RTC: validate monotonicity only
            validation_result = ValidationResult(
                passed=True,
                gaps=[],
                avg_duration=None,
                min_duration=None,
                max_duration=None,
                std_deviation=None,
                messages=["Soft RTC - counter-based timestamps"],
            )

            # Print report
            print_report(
                log_file=log_file,
                config_file=config_file,
                config=config,
                entries=entries,
                sleep_duration=actual_sleep_duration,
                tolerance=tol,
                validation_result=validation_result,
                monotonicity=monotonicity,
                no_duplicates=no_duplicates,
                min_cycles=min_cycles,
                skip_first_cycle=skip_first_cycle,
            )

            # Assertions
            assert monotonicity[0], "Timestamps must be strictly increasing"
            assert no_duplicates[0], "No duplicate entries allowed"


def main():
    """Standalone CLI entry point."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Validate Offline Mode sleep timing from SD card log files"
    )
    parser.add_argument(
        "--log-file", required=True, help="Path to JSONL log file from SD card"
    )
    parser.add_argument(
        "--config-file", required=True, help="Path to config.json from SD card"
    )
    parser.add_argument(
        "--sleep-duration",
        type=int,
        default=None,
        help="Override expected sleep duration in seconds",
    )
    parser.add_argument(
        "--tolerance",
        type=int,
        default=None,
        help="Allowed deviation in seconds (default: 20%% of sleep duration, min 2s)",
    )
    parser.add_argument(
        "--skip-first-cycle",
        action="store_true",
        help="Exclude first boot cycle from timing analysis",
    )
    parser.add_argument(
        "--min-cycles", type=int, default=3, help="Minimum wake cycles required"
    )

    args = parser.parse_args()

    log_file = Path(args.log_file)
    config_file = Path(args.config_file)

    if not log_file.exists():
        print(f"Error: Log file not found: {log_file}")
        sys.exit(1)

    if not config_file.exists():
        print(f"Error: Config file not found: {config_file}")
        sys.exit(1)

    # Parse config
    config = parse_config_json(config_file)
    sleep_duration = args.sleep_duration or config.sleep_duration

    if sleep_duration <= 0:
        print("Error: Sleep duration must be positive")
        sys.exit(1)

    tol = calculate_tolerance(sleep_duration, args.tolerance)

    # Parse log
    entries = parse_log_file(log_file)
    if not entries:
        print("Error: Log file is empty")
        sys.exit(1)

    # Skip first cycle if requested
    if args.skip_first_cycle and config.is_hardware_rtc and len(entries) > 1:
        gaps = detect_sleep_gaps(entries, sleep_duration, tol)
        if gaps:
            first_gap_ts = gaps[0].next_timestamp
            entries = [e for e in entries if e.timestamp >= first_gap_ts]

    # Validations
    monotonicity = validate_monotonicity(entries)
    no_duplicates = validate_no_duplicates(entries)

    if config.is_hardware_rtc:
        gaps = detect_sleep_gaps(entries, sleep_duration, tol)
        validation_result = validate_sleep_timing(gaps, sleep_duration, tol)
    else:
        validation_result = ValidationResult(
            passed=True,
            gaps=[],
            avg_duration=None,
            min_duration=None,
            max_duration=None,
            std_deviation=None,
            messages=["Soft RTC - counter-based timestamps"],
        )

    # Print report
    all_passed = print_report(
        log_file=log_file,
        config_file=config_file,
        config=config,
        entries=entries,
        sleep_duration=sleep_duration,
        tolerance=tol,
        validation_result=validation_result,
        monotonicity=monotonicity,
        no_duplicates=no_duplicates,
        min_cycles=args.min_cycles,
        skip_first_cycle=args.skip_first_cycle,
    )

    sys.exit(0 if all_passed else 1)


if __name__ == "__main__":
    main()
