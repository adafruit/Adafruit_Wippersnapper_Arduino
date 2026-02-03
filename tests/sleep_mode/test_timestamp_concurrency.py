#!/usr/bin/env python3
"""
Timestamp Concurrency Test for WipperSnapper (Test 5.2.1)

Validates that timestamps increment correctly within a wake cycle:
- Timestamps increment as time passes within wake cycle
- Sensor readings at different times have different timestamps
- No duplicate or out-of-order timestamps

Usage:
    pytest tests/test_timestamp_concurrency.py -v --port /dev/tty.usbmodem14101
    pytest tests/test_timestamp_concurrency.py -v --port /dev/ttyUSB0 --timeout 120
"""

import json
import serial
import serial.tools.list_ports
import glob
import time
import pytest
from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional


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
                si_unit=data.get("si_unit")
            )
        except (json.JSONDecodeError, ValueError, KeyError):
            return None

    @property
    def source(self) -> str:
        """Get the source identifier (pin or i2c_address)."""
        if self.pin:
            return f"pin:{self.pin}"
        if self.i2c_address:
            return f"i2c:{self.i2c_address}"
        return "unknown"


@dataclass
class TimestampAnalysis:
    """Results of timestamp analysis."""
    entries: list = field(default_factory=list)
    duplicates: list = field(default_factory=list)
    out_of_order: list = field(default_factory=list)
    gaps_by_source: dict = field(default_factory=dict)

    @property
    def is_valid(self) -> bool:
        """Check if timestamps are valid (monotonic, no duplicates)."""
        return len(self.duplicates) == 0 and len(self.out_of_order) == 0

    @property
    def total_entries(self) -> int:
        """Total number of log entries analyzed."""
        return len(self.entries)

    def summary(self) -> str:
        """Generate analysis summary."""
        lines = [
            "\n" + "=" * 60,
            "TIMESTAMP CONCURRENCY ANALYSIS",
            "=" * 60,
            f"Total Entries: {self.total_entries}",
        ]

        if self.entries:
            timestamps = [e.timestamp for e in self.entries]
            lines.append(f"Timestamp Range: {min(timestamps)} - {max(timestamps)}")
            lines.append(f"Unique Timestamps: {len(set(timestamps))}")

        lines.append("-" * 60)

        # Monotonicity check
        if self.out_of_order:
            lines.append(f"[FAIL] Out-of-order timestamps: {len(self.out_of_order)}")
            for prev, curr in self.out_of_order[:5]:  # Show first 5
                lines.append(f"  {prev.timestamp} -> {curr.timestamp} ({curr.source})")
            if len(self.out_of_order) > 5:
                lines.append(f"  ... and {len(self.out_of_order) - 5} more")
        else:
            lines.append("[PASS] All timestamps are monotonically increasing")

        # Duplicate check
        if self.duplicates:
            lines.append(f"[FAIL] Duplicate timestamps: {len(self.duplicates)}")
            for entries in self.duplicates[:5]:  # Show first 5 groups
                ts = entries[0].timestamp
                sources = [e.source for e in entries]
                lines.append(f"  timestamp={ts}: {sources}")
            if len(self.duplicates) > 5:
                lines.append(f"  ... and {len(self.duplicates) - 5} more")
        else:
            lines.append("[PASS] No duplicate timestamps")

        # Per-source analysis
        lines.append("-" * 60)
        lines.append("PER-SOURCE TIMESTAMP GAPS:")
        for source, gaps in sorted(self.gaps_by_source.items()):
            if gaps:
                avg_gap = sum(gaps) / len(gaps)
                lines.append(f"  {source}: avg_gap={avg_gap:.2f}, count={len(gaps) + 1}")
            else:
                lines.append(f"  {source}: single reading")

        lines.append("=" * 60)
        status = "PASS" if self.is_valid else "FAIL"
        lines.append(f"OVERALL: {status}")
        lines.append("=" * 60)

        return "\n".join(lines)


def find_serial_port() -> Optional[str]:
    """Auto-detect serial port."""
    # Check for /dev/tty.usbmodem* (macOS native USB)
    usbmodem_ports = glob.glob("/dev/tty.usbmodem*")
    if usbmodem_ports:
        return usbmodem_ports[0]

    # Fall back to pyserial port detection
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if any(x in port.description.lower() for x in ["cp210", "ch340", "ftdi", "usb", "esp"]):
            return port.device
        if any(x in port.hwid.lower() for x in ["10c4:ea60", "1a86:7523", "303a:"]):
            return port.device
    return None


def wait_for_device(timeout: float = 30) -> Optional[str]:
    """
    Wait for a device to be connected, prompting the user.

    Checks multiple port patterns and pyserial detection.
    Returns the port path when found, or None on timeout.
    """
    print("\n" + "=" * 60)
    print("WAITING FOR DEVICE CONNECTION")
    print("=" * 60)
    print(f"Please connect your device now (timeout: {timeout}s)")
    print("Looking for USB serial ports...")
    print("-" * 60)

    start = time.time()
    last_status = ""

    while time.time() - start < timeout:
        elapsed = int(time.time() - start)
        remaining = int(timeout - elapsed)

        # Check for macOS native USB ports
        usbmodem_ports = glob.glob("/dev/tty.usbmodem*")
        if usbmodem_ports:
            print(f"\n[{elapsed}s] Found device: {usbmodem_ports[0]}")
            return usbmodem_ports[0]

        # Check for Linux USB ports
        usb_ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
        if usb_ports:
            print(f"\n[{elapsed}s] Found device: {usb_ports[0]}")
            return usb_ports[0]

        # Fall back to pyserial detection
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if any(x in port.description.lower() for x in ["cp210", "ch340", "ftdi", "usb", "esp"]):
                print(f"\n[{elapsed}s] Found device: {port.device}")
                return port.device
            if any(x in port.hwid.lower() for x in ["10c4:ea60", "1a86:7523", "303a:"]):
                print(f"\n[{elapsed}s] Found device: {port.device}")
                return port.device

        # Update status every second
        status = f"[{elapsed}s] Waiting... ({remaining}s remaining)"
        if status != last_status:
            print(status, end="\r")
            last_status = status

        time.sleep(0.5)

    print(f"\n[TIMEOUT] No device found within {timeout}s")
    return None


def is_jsonl_line(line: str) -> bool:
    """Check if a line looks like JSONL with timestamp."""
    return line.startswith("{") and '"timestamp"' in line


def is_sleep_entry(line: str) -> bool:
    """Check if a line indicates device is entering sleep."""
    lower = line.lower()
    return "[sleep] entering deep sleep" in lower or "entering deep sleep" in lower


class TestTimestampConcurrency:
    """Test class for timestamp concurrency validation."""

    def test_timestamp_concurrency(self, serial_port, timeout, baud_rate, cycles):
        """
        Test 5.2.1: Validate timestamp concurrency across wake cycles.

        Connects to device via serial, captures JSONL log output during
        multiple wake cycles, and validates that timestamps are strictly increasing.
        """
        # Resolve port - wait for device if auto-detect
        port = serial_port
        if port == "auto":
            # First quick check if device is already connected
            port = find_serial_port()
            if port is None:
                # Wait for user to connect device (30s timeout)
                port = wait_for_device(timeout=30)

        if port is None:
            available = [p.device for p in serial.tools.list_ports.comports()]
            pytest.fail(
                f"No device connected within 30 seconds.\n"
                f"Available ports: {available}\n"
                f"Please connect your device and try again."
            )

        print(f"\nUsing port: {port}")
        print(f"Timeout per cycle: {timeout}s")
        print(f"Baud rate: {baud_rate}")
        print(f"Cycles to capture: {cycles}")
        print("=" * 60)

        all_entries: list[LogEntry] = []
        cycle_entries: list[list[LogEntry]] = []
        completed_cycles = 0
        max_attempts = cycles * 3  # Allow retries for empty cycles
        attempts = 0
        first_connection = True

        while completed_cycles < cycles and attempts < max_attempts:
            attempts += 1

            # Wait for device to reconnect
            if not first_connection:
                print("\nWaiting for device to wake up and reconnect...")
                port = wait_for_device(timeout=60)
                if port is None:
                    print("[WARNING] Device did not reconnect, retrying...")
                    continue
            first_connection = False

            print(f"\n{'=' * 60}")
            print(f"Capturing data... (completed {completed_cycles}/{cycles} cycles)")
            print("=" * 60)

            entries = self._capture_cycle(port, baud_rate, timeout)

            # Only count as a cycle if we captured actual data
            if len(entries) > 0:
                completed_cycles += 1
                cycle_entries.append(entries)
                all_entries.extend(entries)
                print(f"\n-> CYCLE {completed_cycles}/{cycles} complete: captured {len(entries)} entries")
            else:
                print("\n-> No sensor data captured, not counting as cycle (device may still be booting)")

        # Analyze all captured entries
        if len(all_entries) == 0:
            pytest.fail(
                f"No JSONL log entries captured across {cycles} cycles. "
                "Ensure device is configured with sensors and logging enabled."
            )

        # Print per-cycle summary
        print(f"\n{'=' * 60}")
        print("CYCLE SUMMARY")
        print("=" * 60)
        for i, entries in enumerate(cycle_entries, 1):
            if entries:
                ts_list = [e.timestamp for e in entries]
                print(f"  Cycle {i}: {len(entries)} entries, timestamps {min(ts_list)}-{max(ts_list)}")
            else:
                print(f"  Cycle {i}: 0 entries")

        analysis = self._analyze_timestamps(all_entries)
        print(analysis.summary())

        # Assertions
        assert len(analysis.out_of_order) == 0, (
            f"Found {len(analysis.out_of_order)} out-of-order timestamps. "
            "Timestamps must be strictly increasing."
        )

        assert len(analysis.duplicates) == 0, (
            f"Found {len(analysis.duplicates)} duplicate timestamp groups. "
            "Each timestamp must be unique."
        )

        assert analysis.total_entries >= cycles, (
            f"Expected at least {cycles} entries (one per cycle), got {analysis.total_entries}. "
            "Device may not be logging sensor data."
        )

        print(f"\nTest passed: {analysis.total_entries} entries across {len(cycle_entries)} cycles with valid timestamps")

    def _capture_cycle(self, port: str, baud_rate: int, timeout: int) -> list[LogEntry]:
        """Capture log entries for a single wake cycle."""
        entries: list[LogEntry] = []
        start_time = time.time()

        # Retry opening serial port (device may not be fully ready)
        ser = None
        for attempt in range(60):
            try:
                time.sleep(0.5)  # Give device time to fully enumerate
                ser = serial.Serial(port, baud_rate, timeout=1)
                break
            except (serial.SerialException, OSError) as e:
                if attempt < 59:
                    print(f"  [Retry {attempt + 1}/60] Waiting for device to be ready...")
                    time.sleep(0.5)
                else:
                    print(f"[WARNING] Could not open port after 60 attempts: {e}")
                    return entries

        try:
            with ser:
                while time.time() - start_time < timeout:
                    try:
                        if ser.in_waiting:
                            line = ser.readline().decode("utf-8", errors="replace").strip()
                            if line:
                                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                                print(f"[{timestamp}] {line}")

                                # Check for sleep entry
                                if is_sleep_entry(line):
                                    print("\n-> Sleep entry detected, cycle complete")
                                    break

                                # Parse JSONL lines
                                if is_jsonl_line(line):
                                    entry = LogEntry.from_json(line)
                                    if entry:
                                        entries.append(entry)
                                        print(f"  -> Captured: ts={entry.timestamp}, source={entry.source}")
                        else:
                            time.sleep(0.01)

                    except OSError as e:
                        if e.errno == 6:  # Device not configured (sleep disconnect)
                            print("\n-> Device disconnected (entering sleep)")
                            break
                        raise

        except serial.SerialException as e:
            print(f"[WARNING] Serial error during capture: {e}")

        return entries

    def _analyze_timestamps(self, entries: list[LogEntry]) -> TimestampAnalysis:
        """Analyze timestamps for monotonicity and duplicates."""
        analysis = TimestampAnalysis(entries=entries)

        if not entries:
            return analysis

        # Check for out-of-order timestamps
        for i in range(1, len(entries)):
            if entries[i].timestamp < entries[i - 1].timestamp:
                analysis.out_of_order.append((entries[i - 1], entries[i]))

        # Check for duplicates
        timestamp_groups: dict[int, list[LogEntry]] = {}
        for entry in entries:
            if entry.timestamp not in timestamp_groups:
                timestamp_groups[entry.timestamp] = []
            timestamp_groups[entry.timestamp].append(entry)

        for ts, group in timestamp_groups.items():
            if len(group) > 1:
                analysis.duplicates.append(group)

        # Calculate gaps per source
        source_entries: dict[str, list[LogEntry]] = {}
        for entry in entries:
            source = entry.source
            if source not in source_entries:
                source_entries[source] = []
            source_entries[source].append(entry)

        for source, src_entries in source_entries.items():
            gaps = []
            for i in range(1, len(src_entries)):
                gap = src_entries[i].timestamp - src_entries[i - 1].timestamp
                gaps.append(gap)
            analysis.gaps_by_source[source] = gaps

        return analysis


# Allow running as standalone script
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Timestamp Concurrency Test for WipperSnapper"
    )
    parser.add_argument(
        "-p", "--port",
        default="auto",
        help="Serial port (default: auto-detect)"
    )
    parser.add_argument(
        "-t", "--timeout",
        type=int,
        default=60,
        help="Timeout per cycle in seconds (default: 60)"
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
        default=20,
        help="Number of wake cycles to capture (default: 20)"
    )

    args = parser.parse_args()

    # Run test directly
    test = TestTimestampConcurrency()
    test.test_timestamp_concurrency(args.port, args.timeout, args.baud, args.cycles)
