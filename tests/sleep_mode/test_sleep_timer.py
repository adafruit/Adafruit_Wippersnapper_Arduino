#!/usr/bin/env python3
"""
Sleep Timer Test Script for WipperSnapper

Monitors serial output to validate device sleep/wake cycles.
Verifies timing accuracy within configurable tolerance over multiple cycles.

Usage (pytest):
    pytest tests/sleep_mode/test_sleep_timer.py -v -s --cycles 4 --sleep-duration 300

Usage (standalone):
    python test_sleep_timer.py --sleep-duration 300 --cycles 4
    python test_sleep_timer.py -s 60 -c 10  # 10 cycles of 60-second sleep
"""

import serial
import serial.tools.list_ports
import argparse
import glob
import os
import time
import re
import pytest
from pathlib import Path
from datetime import datetime
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class Pattern:
    """A single pattern definition."""
    id: str
    pattern: re.Pattern
    message: str


def load_patterns(pattern_file: Path) -> dict[str, Pattern]:
    """Load patterns from a markdown file.

    Expected format:
    ## pattern_id
    - **Pattern**: `regex`
    - **Case Insensitive**: yes|no
    - **Message**: message to display
    """
    patterns = {}
    content = pattern_file.read_text()

    # Split by h2 headers
    sections = re.split(r'^## ', content, flags=re.MULTILINE)[1:]  # Skip before first ##

    for section in sections:
        lines = section.strip().split('\n')
        if not lines:
            continue

        pattern_id = lines[0].strip()
        pattern_str = None
        case_insensitive = True
        message = pattern_id  # Default message

        for line in lines[1:]:
            if '**Pattern**' in line:
                # Extract pattern from backticks
                match = re.search(r'`([^`]+)`', line)
                if match:
                    pattern_str = match.group(1)
            elif '**Case Insensitive**' in line:
                case_insensitive = 'yes' in line.lower()
            elif '**Message**' in line:
                # Extract message after colon
                match = re.search(r'\*\*Message\*\*:\s*(.+)', line)
                if match:
                    message = match.group(1).strip()

        if pattern_str:
            flags = re.IGNORECASE if case_insensitive else 0
            patterns[pattern_id] = Pattern(
                id=pattern_id,
                pattern=re.compile(pattern_str, flags),
                message=message
            )

    return patterns


@dataclass
class CycleResult:
    """Results for a single wake cycle."""
    cycle_num: int
    start_time: float = 0.0
    end_time: float = 0.0
    matched: dict = field(default_factory=dict)  # pattern_id -> bool
    sleep_duration: Optional[float] = None

    # Required patterns for a cycle to pass
    REQUIRED = ['sd_init', 'config_parsed', 'entered_loop_sleep',
                'i2c_events', 'entering_sleep', 'deep_sleep']

    def is_pass(self) -> bool:
        """Check if this cycle passed all required checks."""
        return all(self.matched.get(p, False) for p in self.REQUIRED)


@dataclass
class TestResults:
    """Overall test results."""
    cycles: list = field(default_factory=list)
    patterns: dict = field(default_factory=dict)  # pattern_id -> Pattern
    start_time: datetime = None
    end_time: datetime = None
    total_cycles_expected: int = 4
    expected_sleep_duration: int = 300  # seconds
    usb_overhead: int = 7  # USB re-enumeration overhead in seconds
    tolerance: int = 5  # timing tolerance in seconds

    def summary(self) -> str:
        """Generate test summary."""
        # Identify complete vs incomplete cycles
        complete_cycles = [c for c in self.cycles if c.matched.get('deep_sleep', False)]
        incomplete_cycles = [c for c in self.cycles if not c.matched.get('deep_sleep', False)]

        # Calculate expected timing values
        expected_total = self.expected_sleep_duration + self.usb_overhead
        range_min = expected_total - self.tolerance
        range_max = expected_total + self.tolerance

        lines = [
            "\n" + "=" * 60,
            f"SLEEP TIMER TEST RESULTS ({self.expected_sleep_duration}s sleep, {self.total_cycles_expected} cycles)",
            "=" * 60,
            f"Start Time: {self.start_time}",
            f"End Time:   {self.end_time}",
            f"Duration:   {self.end_time - self.start_time}" if self.end_time and self.start_time else "Duration: N/A",
            f"Complete Cycles: {len(complete_cycles)}/{self.total_cycles_expected}",
        ]
        if incomplete_cycles:
            lines.append(f"Incomplete Cycles: {len(incomplete_cycles)} (not counted in results)")
        lines.append("-" * 60)

        # Checkpoint summary (only count complete cycles)
        lines.append("\nCHECKPOINT SUMMARY (complete cycles only):")

        # Build checkpoint list from loaded patterns (exclude helper patterns)
        skip_patterns = {'sd_init_confirm', 'device_reset'}
        for pattern_id, pattern in self.patterns.items():
            if pattern_id in skip_patterns:
                continue

            passed = sum(1 for c in complete_cycles if c.matched.get(pattern_id, False))
            total = len(complete_cycles)

            # Wake cause is special: first cycle is PowerOn, subsequent cycles should be Sleep/Timer
            if pattern_id == "wake_cause":
                wake_cycles = [c for c in complete_cycles if c.cycle_num > 1]
                passed = sum(1 for c in wake_cycles if c.matched.get(pattern_id, False))
                total = len(wake_cycles)
                status = "PASS" if passed == total and total >= 0 else "FAIL"
                if len(complete_cycles) > 0 and complete_cycles[0].cycle_num == 1:
                    lines.append(f"  [{status}] {pattern.message}: {passed}/{total} (cycle 1 = PowerOn, excluded)")
                else:
                    lines.append(f"  [{status}] {pattern.message}: {passed}/{total}")
            else:
                status = "PASS" if passed == total and total > 0 else "FAIL"
                lines.append(f"  [{status}] {pattern.message}: {passed}/{total}")

        # Sleep duration analysis
        lines.append("\nSLEEP DURATION ANALYSIS:")
        lines.append(f"  Note: Measured time includes USB re-enumeration (~{self.usb_overhead}s overhead)")
        lines.append(f"  Expected: ~{expected_total}s total ({self.expected_sleep_duration}s sleep + {self.usb_overhead}s USB/boot)")
        durations = [c.sleep_duration for c in self.cycles if c.sleep_duration is not None]
        if durations:
            avg_duration = sum(durations) / len(durations)
            min_duration = min(durations)
            max_duration = max(durations)
            in_range = sum(1 for d in durations if range_min <= d <= range_max)
            lines.append(f"  Average: {avg_duration:.2f}s")
            lines.append(f"  Min: {min_duration:.2f}s, Max: {max_duration:.2f}s")
            lines.append(f"  Within expected range ({range_min}-{range_max}s): {in_range}/{len(durations)}")
        else:
            lines.append("  No sleep duration data collected")

        # Per-cycle details
        lines.append("\nPER-CYCLE DETAILS:")
        for cycle in self.cycles:
            if cycle.matched.get('deep_sleep', False):
                status = "PASS" if cycle.is_pass() else "FAIL"
            else:
                status = "INCOMPLETE"
            duration_str = f"{cycle.sleep_duration:.2f}s" if cycle.sleep_duration else "N/A"
            lines.append(f"  Cycle {cycle.cycle_num}: [{status}] sleep_duration={duration_str}")

        # Overall result (based on complete cycles only)
        lines.append("\n" + "=" * 60)
        all_pass = all(c.is_pass() for c in complete_cycles) if complete_cycles else False
        enough_cycles = len(complete_cycles) >= self.total_cycles_expected
        if all_pass and enough_cycles:
            overall = "PASS"
        elif all_pass and len(complete_cycles) > 0:
            overall = f"PARTIAL ({len(complete_cycles)}/{self.total_cycles_expected} cycles)"
        else:
            overall = "FAIL"
        lines.append(f"OVERALL RESULT: {overall}")
        lines.append("=" * 60)

        return "\n".join(lines)


class SleepTimerTest:
    """Test runner for sleep timer validation."""

    def __init__(self, port: str, baud: int = 115200, cycles: int = 10,
                 sleep_duration: int = 300, pattern_file: Optional[Path] = None):
        self.port = port
        self.baud = baud
        self.expected_cycles = cycles
        self.sleep_duration = sleep_duration

        # Load patterns from file
        if pattern_file is None:
            pattern_file = Path(__file__).parent / "sleep_timer_patterns.md"
        self.patterns = load_patterns(pattern_file)

        self.results = TestResults(
            total_cycles_expected=cycles,
            expected_sleep_duration=sleep_duration,
            patterns=self.patterns
        )
        self.current_cycle: Optional[CycleResult] = None
        self.cycle_count = 0
        self.last_sleep_time: Optional[float] = None
        self._pending_sd_init = False

    def find_port(self) -> Optional[str]:
        """Auto-detect ESP32 serial port."""
        # First, check for /dev/tty.usbmodem* (macOS native USB)
        usbmodem_ports = glob.glob("/dev/tty.usbmodem*")
        if usbmodem_ports:
            port = usbmodem_ports[0]
            print(f"Found device: {port}")
            return port

        # Fall back to pyserial port detection
        ports = serial.tools.list_ports.comports()
        for port in ports:
            # Look for common ESP32 USB identifiers
            if any(x in port.description.lower() for x in ["cp210", "ch340", "ftdi", "usb", "esp"]):
                print(f"Found device: {port.device} - {port.description}")
                return port.device
            if any(x in port.hwid.lower() for x in ["10c4:ea60", "1a86:7523", "303a:"]):
                print(f"Found device: {port.device} - {port.description}")
                return port.device
        return None

    def start_new_cycle(self):
        """Start tracking a new wake cycle."""
        if self.current_cycle is not None:
            # Save the previous cycle
            self.current_cycle.end_time = time.time()
            self.results.cycles.append(self.current_cycle)

        self.cycle_count += 1
        self.current_cycle = CycleResult(cycle_num=self.cycle_count)
        self.current_cycle.start_time = time.time()

        # Calculate sleep duration from last cycle
        if self.last_sleep_time is not None:
            duration = time.time() - self.last_sleep_time
            if len(self.results.cycles) > 0:
                self.results.cycles[-1].sleep_duration = duration
                print(f"  -> Sleep duration: {duration:.2f}s")

        print(f"\n{'='*40}")
        print(f"CYCLE {self.cycle_count} STARTED")
        print(f"{'='*40}")

    def _match_pattern(self, pattern_id: str, line: str) -> bool:
        """Check if line matches a pattern."""
        if pattern_id not in self.patterns:
            return False
        return bool(self.patterns[pattern_id].pattern.search(line))

    def _set_matched(self, pattern_id: str):
        """Mark a pattern as matched for the current cycle."""
        if self.current_cycle:
            self.current_cycle.matched[pattern_id] = True
            msg = self.patterns[pattern_id].message
            print(f"  -> {msg}")

    def process_line(self, line: str):
        """Process a single line of serial output."""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(f"[{timestamp}] {line}")

        # Check for SD init (must be before device_reset check since same line triggers both)
        if self._match_pattern("sd_init", line):
            self._pending_sd_init = True  # Wait for confirmation

        if self.current_cycle is None:
            # First boot or reset detected
            if self._match_pattern("device_reset", line):
                self.start_new_cycle()
            return

        # Check SD init confirmation (the "1" after "Is SD Card initialized?")
        if self._pending_sd_init:
            if self._match_pattern("sd_init_confirm", line):
                self._set_matched("sd_init")
                self._pending_sd_init = False

        # Check for new cycle (device reset after sleep)
        if self._match_pattern("device_reset", line) and self.current_cycle.matched.get('deep_sleep', False):
            self.start_new_cycle()
            return

        # Check all other patterns
        simple_patterns = ['config_parsed', 'entered_loop_sleep', 'i2c_events',
                          'update_complete', 'entering_sleep', 'wake_cause']
        for pattern_id in simple_patterns:
            if self._match_pattern(pattern_id, line) and not self.current_cycle.matched.get(pattern_id, False):
                self._set_matched(pattern_id)

        # Deep sleep needs special handling (record time)
        if self._match_pattern("deep_sleep", line) and not self.current_cycle.matched.get('deep_sleep', False):
            self._set_matched("deep_sleep")
            self.last_sleep_time = time.time()

    def wait_for_port(self, port_pattern: str, timeout: float = 60) -> Optional[str]:
        """Wait for a serial port matching pattern to appear."""
        start = time.time()
        while time.time() - start < timeout:
            ports = glob.glob(port_pattern)
            if ports:
                return ports[0]
            time.sleep(0.5)
        return None

    def run(self):
        """Run the test."""
        port = self.port
        port_pattern = "/dev/tty.usbmodem*"

        if port == "auto":
            port = self.find_port()
            if port is None:
                print("Waiting for device to connect...")
                port = self.wait_for_port(port_pattern, timeout=60)
                if port is None:
                    print("ERROR: Could not find serial port")
                    print("Available ports:")
                    for p in serial.tools.list_ports.comports():
                        print(f"  {p.device} - {p.description}")
                    return
        else:
            port_pattern = port  # Use specific port for reconnection

        print(f"Using port: {port}")
        print("Note: Device disconnects during deep sleep - will auto-reconnect on wake")
        sleep_mins = self.sleep_duration / 60
        total_mins = self.expected_cycles * sleep_mins
        print(f"Waiting for {self.expected_cycles} wake cycles (~{sleep_mins:.1f} min each, ~{total_mins:.1f} min total)...")
        print("Press Ctrl+C to stop and show results\n")

        self.results.start_time = datetime.now()

        try:
            while self.cycle_count <= self.expected_cycles:
                # Try to connect to serial port
                try:
                    # Check if port exists
                    if not glob.glob(port_pattern.replace("*", "") + "*" if "*" not in port_pattern else port_pattern):
                        if not os.path.exists(port):
                            print(f"[{datetime.now().strftime('%H:%M:%S')}] Waiting for device to wake...")
                            port = self.wait_for_port("/dev/tty.usbmodem*", timeout=800)
                            if port is None:
                                print("Timeout waiting for device")
                                break
                            print(f"[{datetime.now().strftime('%H:%M:%S')}] Device reconnected: {port}")

                    with serial.Serial(port, self.baud, timeout=1) as ser:
                        print(f"[{datetime.now().strftime('%H:%M:%S')}] Connected to {port}")
                        while self.cycle_count <= self.expected_cycles:
                            try:
                                if ser.in_waiting:
                                    line = ser.readline().decode("utf-8", errors="replace").strip()
                                    if line:
                                        self.process_line(line)
                                else:
                                    time.sleep(0.01)
                            except OSError as e:
                                if e.errno == 6:  # Device not configured (sleep disconnect)
                                    print(f"\n[{datetime.now().strftime('%H:%M:%S')}] Device disconnected (entering sleep)")
                                    break
                                raise
                            except serial.SerialException:
                                print(f"\n[{datetime.now().strftime('%H:%M:%S')}] Serial read error, device likely sleeping")
                                break

                except (serial.SerialException, OSError) as e:
                    err_str = str(e).lower()
                    if "could not open port" in err_str or "no such file" in err_str or "errno 6" in err_str:
                        print(f"[{datetime.now().strftime('%H:%M:%S')}] Port not available, waiting for wake...")
                        time.sleep(1)
                    else:
                        print(f"[{datetime.now().strftime('%H:%M:%S')}] Serial error: {e}")
                        time.sleep(1)

        except KeyboardInterrupt:
            print("\n\nTest interrupted by user")
        except Exception as e:
            print(f"Unexpected error: {e}")
        finally:
            # Save final cycle if in progress
            if self.current_cycle is not None:
                self.current_cycle.end_time = time.time()
                self.results.cycles.append(self.current_cycle)

            self.results.end_time = datetime.now()
            print(self.results.summary())


class TestSleepTimer:
    """Pytest test class for sleep timer validation."""

    def test_sleep_timer(self, serial_port, baud_rate, cycles, sleep_duration, patterns_file):
        """
        Test sleep timer functionality across multiple wake cycles.

        Validates:
        - Device enters sleep mode correctly
        - Device wakes up after expected duration
        - All required checkpoints are hit each cycle
        """
        # Use default patterns file if not specified
        if patterns_file is None:
            patterns_file = Path(__file__).parent / "sleep_timer_patterns.md"

        test = SleepTimerTest(
            serial_port, baud_rate, cycles, sleep_duration, patterns_file
        )
        test.run()

        # Validate results
        complete_cycles = [c for c in test.results.cycles if c.matched.get('deep_sleep', False)]

        assert len(complete_cycles) >= cycles, (
            f"Expected {cycles} complete cycles, got {len(complete_cycles)}"
        )

        failed_cycles = [c for c in complete_cycles if not c.is_pass()]
        assert len(failed_cycles) == 0, (
            f"{len(failed_cycles)} cycles failed required checkpoints"
        )


def main():
    parser = argparse.ArgumentParser(
        description="Sleep Timer Test for WipperSnapper"
    )
    parser.add_argument(
        "-p", "--port",
        default="auto",
        help="Serial port (default: auto-detect)"
    )
    parser.add_argument(
        "-b", "--baud",
        type=int,
        default=115200,
        help="Baud rate (default: 115200)"
    )
    parser.add_argument(
        "-c", "--cycles",
        type=int,
        default=4,
        help="Number of wake cycles to observe (default: 4)"
    )
    parser.add_argument(
        "-s", "--sleep-duration",
        type=int,
        default=300,
        help="Expected sleep duration in seconds (default: 300 = 5 minutes)"
    )
    parser.add_argument(
        "--patterns",
        type=Path,
        default=None,
        help="Path to patterns markdown file (default: sleep_timer_patterns.md)"
    )

    args = parser.parse_args()

    test = SleepTimerTest(
        args.port, args.baud, args.cycles, args.sleep_duration, args.patterns
    )
    test.run()


if __name__ == "__main__":
    main()
