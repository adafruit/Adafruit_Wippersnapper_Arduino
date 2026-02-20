#!/usr/bin/env python3
"""
Sleep EXT0 GPIO Wakeup Test Script for WipperSnapper

Monitors serial output to validate device sleep/wake cycles using GPIO button press.
Verifies EXT0 wakeup configuration and wake cause detection.

Usage (pytest):
    pytest tests/sleep_mode/test_sleep_ext0.py -v -s --cycles 4

Usage (standalone):
    python test_sleep_ext0.py --cycles 4
    python test_sleep_ext0.py -p /dev/tty.usbmodem1201 -c 3
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
    """Load patterns from a markdown file."""
    patterns = {}
    content = pattern_file.read_text()

    sections = re.split(r'^## ', content, flags=re.MULTILINE)[1:]

    for section in sections:
        lines = section.strip().split('\n')
        if not lines:
            continue

        pattern_id = lines[0].strip()
        pattern_str = None
        case_insensitive = True
        message = pattern_id

        for line in lines[1:]:
            if '**Pattern**' in line:
                match = re.search(r'`([^`]+)`', line)
                if match:
                    pattern_str = match.group(1)
            elif '**Case Insensitive**' in line:
                case_insensitive = 'yes' in line.lower()
            elif '**Message**' in line:
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
    matched: dict = field(default_factory=dict)
    ext0_pin: Optional[str] = None
    ext0_level: Optional[str] = None

    # Required patterns for a cycle to pass
    REQUIRED = ['config_parsed', 'ext0_config', 'entered_loop_sleep', 'entering_sleep', 'deep_sleep']

    def is_pass(self) -> bool:
        """Check if this cycle passed all required checks."""
        return all(self.matched.get(p, False) for p in self.REQUIRED)


@dataclass
class TestResults:
    """Overall test results."""
    cycles: list = field(default_factory=list)
    patterns: dict = field(default_factory=dict)
    start_time: datetime = None
    end_time: datetime = None
    total_cycles_expected: int = 4
    ext0_pin: Optional[str] = None
    ext0_level: Optional[str] = None

    def summary(self) -> str:
        """Generate test summary."""
        complete_cycles = [c for c in self.cycles if c.matched.get('deep_sleep', False)]
        incomplete_cycles = [c for c in self.cycles if not c.matched.get('deep_sleep', False)]

        lines = [
            "\n" + "=" * 60,
            f"SLEEP EXT0 GPIO WAKEUP TEST RESULTS ({self.total_cycles_expected} cycles)",
            "=" * 60,
            f"Start Time: {self.start_time}",
            f"End Time:   {self.end_time}",
            f"Duration:   {self.end_time - self.start_time}" if self.end_time and self.start_time else "Duration: N/A",
            f"Complete Cycles: {len(complete_cycles)}/{self.total_cycles_expected}",
        ]

        if self.ext0_pin:
            lines.append(f"EXT0 Pin: {self.ext0_pin}, Level: {self.ext0_level}")

        if incomplete_cycles:
            lines.append(f"Incomplete Cycles: {len(incomplete_cycles)} (not counted)")
        lines.append("-" * 60)

        lines.append("\nCHECKPOINT SUMMARY:")

        skip_patterns = {'sd_init_confirm', 'device_reset', 'wake_cause_poweron'}
        for pattern_id, pattern in self.patterns.items():
            if pattern_id in skip_patterns:
                continue

            passed = sum(1 for c in complete_cycles if c.matched.get(pattern_id, False))
            total = len(complete_cycles)

            # Wake cause EXT0: first cycle is PowerOn, subsequent should be EXT0
            if pattern_id == "wake_cause_ext0":
                wake_cycles = [c for c in complete_cycles if c.cycle_num > 1]
                passed = sum(1 for c in wake_cycles if c.matched.get(pattern_id, False))
                total = len(wake_cycles)
                status = "PASS" if passed == total and total >= 0 else "FAIL"
                if len(complete_cycles) > 0 and complete_cycles[0].cycle_num == 1:
                    lines.append(f"  [{status}] {pattern.message}: {passed}/{total} (cycle 1 = PowerOn)")
                else:
                    lines.append(f"  [{status}] {pattern.message}: {passed}/{total}")
            else:
                status = "PASS" if passed == total and total > 0 else "FAIL"
                lines.append(f"  [{status}] {pattern.message}: {passed}/{total}")

        lines.append("\nPER-CYCLE DETAILS:")
        for cycle in self.cycles:
            if cycle.matched.get('deep_sleep', False):
                status = "PASS" if cycle.is_pass() else "FAIL"
            else:
                status = "INCOMPLETE"

            wake_type = "PowerOn" if cycle.cycle_num == 1 else ("EXT0" if cycle.matched.get('wake_cause_ext0') else "Unknown")
            lines.append(f"  Cycle {cycle.cycle_num}: [{status}] wake={wake_type}")

        lines.append("\n" + "=" * 60)
        all_pass = all(c.is_pass() for c in complete_cycles) if complete_cycles else False
        enough_cycles = len(complete_cycles) >= self.total_cycles_expected

        # Check EXT0 wake cause for cycles > 1
        ext0_wakes = [c for c in complete_cycles if c.cycle_num > 1]
        ext0_pass = all(c.matched.get('wake_cause_ext0', False) for c in ext0_wakes) if ext0_wakes else True

        if all_pass and enough_cycles and ext0_pass:
            overall = "PASS"
        elif all_pass and len(complete_cycles) > 0:
            overall = f"PARTIAL ({len(complete_cycles)}/{self.total_cycles_expected} cycles)"
        else:
            overall = "FAIL"
        lines.append(f"OVERALL RESULT: {overall}")
        lines.append("=" * 60)

        return "\n".join(lines)


class SleepExt0Test:
    """Test runner for EXT0 GPIO wakeup validation."""

    def __init__(self, port: str, baud: int = 115200, cycles: int = 4,
                 pattern_file: Optional[Path] = None):
        self.port = port
        self.baud = baud
        self.expected_cycles = cycles

        if pattern_file is None:
            pattern_file = Path(__file__).parent / "sleep_ext0_patterns.md"
        self.patterns = load_patterns(pattern_file)

        self.results = TestResults(
            total_cycles_expected=cycles,
            patterns=self.patterns
        )
        self.current_cycle: Optional[CycleResult] = None
        self.cycle_count = 0
        self._pending_sd_init = False

    def find_port(self) -> Optional[str]:
        """Auto-detect ESP32 serial port."""
        usbmodem_ports = glob.glob("/dev/tty.usbmodem*")
        if usbmodem_ports:
            port = usbmodem_ports[0]
            print(f"Found device: {port}")
            return port

        ports = serial.tools.list_ports.comports()
        for port in ports:
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
            self.current_cycle.end_time = time.time()
            self.results.cycles.append(self.current_cycle)

        self.cycle_count += 1
        self.current_cycle = CycleResult(cycle_num=self.cycle_count)
        self.current_cycle.start_time = time.time()

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

        # Check for SD init
        if self._match_pattern("sd_init", line):
            self._pending_sd_init = True

        if self.current_cycle is None:
            if self._match_pattern("device_reset", line):
                self.start_new_cycle()
            return

        # SD init confirmation
        if self._pending_sd_init:
            if self._match_pattern("sd_init_confirm", line):
                self._set_matched("sd_init")
                self._pending_sd_init = False

        # New cycle detection
        if self._match_pattern("device_reset", line) and self.current_cycle.matched.get('deep_sleep', False):
            self.start_new_cycle()
            return

        # Extract EXT0 config details
        if "EXT0 wakeup set on pin" in line:
            match = re.search(r'pin\s+(\w+)\s+with level\s+(\w+)', line)
            if match:
                self.results.ext0_pin = match.group(1)
                self.results.ext0_level = match.group(2)
                if self.current_cycle:
                    self.current_cycle.ext0_pin = match.group(1)
                    self.current_cycle.ext0_level = match.group(2)

        # Check patterns
        simple_patterns = ['config_parsed', 'ext0_config', 'entered_loop_sleep',
                          'analog_events', 'i2c_events', 'update_complete',
                          'entering_sleep', 'wake_cause_ext0', 'wake_cause_poweron']
        for pattern_id in simple_patterns:
            if self._match_pattern(pattern_id, line) and not self.current_cycle.matched.get(pattern_id, False):
                self._set_matched(pattern_id)

        # Deep sleep detection
        if self._match_pattern("deep_sleep", line) and not self.current_cycle.matched.get('deep_sleep', False):
            self._set_matched("deep_sleep")
            print("\n  *** DEVICE SLEEPING - PRESS BUTTON TO WAKE ***\n")

    def wait_for_port(self, port_pattern: str, timeout: float = 300) -> Optional[str]:
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
                    for p in serial.tools.list_ports.comports():
                        print(f"  {p.device} - {p.description}")
                    return
        else:
            port_pattern = port

        print(f"Using port: {port}")
        print("Note: Device disconnects during deep sleep - will auto-reconnect on wake")
        print(f"Testing {self.expected_cycles} wake cycles with EXT0 GPIO wakeup")
        print("Press the wakeup button when device enters sleep")
        print("Press Ctrl+C to stop and show results\n")

        self.results.start_time = datetime.now()

        try:
            while self.cycle_count <= self.expected_cycles:
                try:
                    if not glob.glob(port_pattern.replace("*", "") + "*" if "*" not in port_pattern else port_pattern):
                        if not os.path.exists(port):
                            print(f"[{datetime.now().strftime('%H:%M:%S')}] Waiting for device (press button)...")
                            port = self.wait_for_port("/dev/tty.usbmodem*", timeout=300)
                            if port is None:
                                print("Timeout waiting for device - did you press the button?")
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
                                if e.errno == 6:
                                    print(f"\n[{datetime.now().strftime('%H:%M:%S')}] Device disconnected (sleeping)")
                                    break
                                raise
                            except serial.SerialException:
                                print(f"\n[{datetime.now().strftime('%H:%M:%S')}] Serial error, device sleeping")
                                break

                except (serial.SerialException, OSError) as e:
                    err_str = str(e).lower()
                    if "could not open port" in err_str or "no such file" in err_str or "errno 6" in err_str:
                        print(f"[{datetime.now().strftime('%H:%M:%S')}] Port not available, waiting...")
                        time.sleep(1)
                    else:
                        print(f"[{datetime.now().strftime('%H:%M:%S')}] Serial error: {e}")
                        time.sleep(1)

        except KeyboardInterrupt:
            print("\n\nTest interrupted by user")
        except Exception as e:
            print(f"Unexpected error: {e}")
        finally:
            if self.current_cycle is not None:
                self.current_cycle.end_time = time.time()
                self.results.cycles.append(self.current_cycle)

            self.results.end_time = datetime.now()
            print(self.results.summary())


class TestSleepExt0:
    """Pytest test class for EXT0 GPIO wakeup validation."""

    def test_sleep_ext0(self, serial_port, baud_rate, cycles, patterns_file):
        """
        Test EXT0 GPIO wakeup functionality across multiple wake cycles.

        Validates:
        - Device configures EXT0 wakeup correctly
        - Device enters sleep mode
        - Device wakes on GPIO button press
        - Wake cause is correctly reported as EXT0
        """
        # Use default patterns file if not specified
        if patterns_file is None:
            patterns_file = Path(__file__).parent / "sleep_ext0_patterns.md"

        test = SleepExt0Test(serial_port, baud_rate, cycles, patterns_file)
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

        # Verify EXT0 wake cause for cycles > 1
        ext0_wakes = [c for c in complete_cycles if c.cycle_num > 1]
        ext0_correct = [c for c in ext0_wakes if c.matched.get('wake_cause_ext0', False)]
        assert len(ext0_correct) == len(ext0_wakes), (
            f"Expected all wake cycles to report EXT0 cause, "
            f"got {len(ext0_correct)}/{len(ext0_wakes)}"
        )


def main():
    parser = argparse.ArgumentParser(
        description="Sleep EXT0 GPIO Wakeup Test for WipperSnapper"
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
        "--patterns",
        type=Path,
        default=None,
        help="Path to patterns markdown file"
    )

    args = parser.parse_args()

    test = SleepExt0Test(args.port, args.baud, args.cycles, args.patterns)
    test.run()


if __name__ == "__main__":
    main()
