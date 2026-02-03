#!/usr/bin/env python3
"""
External RTC Timestamp Test for WipperSnapper's Sleep Mode
"""

import serial
import serial.tools.list_ports
import time
import pytest
from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional

try:
    from .utils import (
        LogEntry,
        CycleData,
        CrossCycleGap,
        find_serial_port,
        wait_for_device,
        is_jsonl_line,
        is_sleep_entry,
        parse_sleep_duration,
    )
except ImportError:
    # Fallback for standalone execution
    from utils import (
        LogEntry,
        CycleData,
        CrossCycleGap,
        find_serial_port,
        wait_for_device,
        is_jsonl_line,
        is_sleep_entry,
        parse_sleep_duration,
    )


# Minimum Unix timestamp to consider as "external RTC" (year 2023+)
MIN_UNIX_TIMESTAMP = 1700000000

# Tolerance for cross-cycle gap validation (accounts for boot time, sensor reads, RTC drift)
CROSS_CYCLE_TOLERANCE_SECONDS = 30


@dataclass
class ExternalRTCAnalysis:
    """Results of external RTC timestamp analysis."""
    entries: list = field(default_factory=list)
    duplicates: list = field(default_factory=list)
    out_of_order: list = field(default_factory=list)
    gaps_by_source: dict = field(default_factory=dict)
    # Cross-cycle persistence analysis
    cross_cycle_gaps: list = field(default_factory=list)  # List of CrossCycleGap
    timestamp_resets: list = field(default_factory=list)  # Cycles where timestamp reset
    # External RTC specific
    non_unix_timestamps: list = field(default_factory=list)  # Timestamps < MIN_UNIX_TIMESTAMP

    @property
    def is_external_rtc(self) -> bool:
        """Check if timestamps indicate external RTC is being used."""
        if not self.entries:
            return False
        return len(self.non_unix_timestamps) == 0

    @property
    def is_valid(self) -> bool:
        """Check if timestamps are valid (monotonic, no duplicates)."""
        return len(self.duplicates) == 0 and len(self.out_of_order) == 0

    @property
    def persistence_valid(self) -> bool:
        """Check if timestamps persist correctly across sleep cycles."""
        if self.timestamp_resets:
            return False
        return all(gap.is_valid for gap in self.cross_cycle_gaps)

    @property
    def total_entries(self) -> int:
        """Total number of log entries analyzed."""
        return len(self.entries)

    def summary(self) -> str:
        """Generate analysis summary."""
        lines = [
            "\n" + "=" * 60,
            "EXTERNAL RTC TIMESTAMP ANALYSIS",
            "=" * 60,
            f"Total Entries: {self.total_entries}",
        ]

        if self.entries:
            timestamps = [e.timestamp for e in self.entries]
            lines.append(f"Timestamp Range: {min(timestamps)} - {max(timestamps)}")

        # External RTC detection
        if self.non_unix_timestamps:
            lines.append(f"[FAIL] {len(self.non_unix_timestamps)} timestamps are NOT Unix time (external RTC not detected)")
            for entry in self.non_unix_timestamps[:5]:
                lines.append(f"  ts={entry.timestamp} (source={entry.source})")
            if len(self.non_unix_timestamps) > 5:
                lines.append(f"  ... and {len(self.non_unix_timestamps) - 5} more")
        else:
            lines.append("[PASS] All timestamps are Unix time (external RTC detected)")

        lines.append("-" * 60)

        # Monotonicity check
        if self.out_of_order:
            lines.append(f"[FAIL] Out-of-order timestamps: {len(self.out_of_order)}")
            for prev, curr in self.out_of_order[:5]:
                lines.append(f"  {prev.timestamp} -> {curr.timestamp} ({curr.source})")
            if len(self.out_of_order) > 5:
                lines.append(f"  ... and {len(self.out_of_order) - 5} more")
        else:
            lines.append("[PASS] All timestamps are monotonically increasing")

        # Duplicate check
        if self.duplicates:
            lines.append(f"[FAIL] Duplicate timestamps: {len(self.duplicates)}")
            for entries in self.duplicates[:5]:
                ts = entries[0].timestamp
                sources = [e.source for e in entries]
                lines.append(f"  timestamp={ts}: {sources}")
            if len(self.duplicates) > 5:
                lines.append(f"  ... and {len(self.duplicates) - 5} more")
        else:
            lines.append("[PASS] No duplicate timestamps")

        # Cross-cycle persistence analysis
        lines.append("-" * 60)
        lines.append("CROSS-CYCLE PERSISTENCE:")

        if self.timestamp_resets:
            lines.append(f"[FAIL] Timestamp resets detected in {len(self.timestamp_resets)} cycles")
            for cycle_idx in self.timestamp_resets[:5]:
                lines.append(f"  Cycle {cycle_idx}: timestamp reset detected")
        else:
            lines.append("[PASS] No timestamp resets detected")

        if self.cross_cycle_gaps:
            valid_gaps = [g for g in self.cross_cycle_gaps if g.is_valid]
            invalid_gaps = [g for g in self.cross_cycle_gaps if not g.is_valid]

            if invalid_gaps:
                lines.append(f"[FAIL] {len(invalid_gaps)}/{len(self.cross_cycle_gaps)} cross-cycle gaps invalid")
                for gap in invalid_gaps[:5]:
                    lines.append(f"  Cycle {gap.cycle_index}: {gap.error_message}")
            else:
                lines.append(f"[PASS] All {len(valid_gaps)} cross-cycle gaps valid (within ±{CROSS_CYCLE_TOLERANCE_SECONDS}s tolerance)")

            lines.append("  Gap details:")
            for gap in self.cross_cycle_gaps:
                status_mark = "✓" if gap.is_valid else "✗"
                expected_str = f"expected={gap.reported_sleep_duration}s ±{CROSS_CYCLE_TOLERANCE_SECONDS}s" if gap.reported_sleep_duration else "no expected duration"
                lines.append(
                    f"  {status_mark} Cycle {gap.cycle_index-1}→{gap.cycle_index}: "
                    f"ts {gap.last_timestamp}→{gap.first_timestamp} (delta={gap.timestamp_delta}s, {expected_str})"
                )
        else:
            lines.append("[INFO] No cross-cycle gaps to analyze (single cycle or no data)")

        lines.append("=" * 60)
        external_rtc_status = "PASS" if self.is_external_rtc else "FAIL"
        concurrency_status = "PASS" if self.is_valid else "FAIL"
        persistence_status = "PASS" if self.persistence_valid else "FAIL"
        lines.append(f"External RTC: {external_rtc_status}")
        lines.append(f"Concurrency: {concurrency_status}")
        lines.append(f"Persistence: {persistence_status}")
        lines.append("=" * 60)

        return "\n".join(lines)


class TestExternalRTCTimestamps:
    """Test class for external RTC timestamp validation."""

    def test_external_rtc_timestamps(self, serial_port, timeout, baud_rate, cycles):
        """
        Connects to device via serial, captures JSONL log output during
        multiple wake cycles, and validates that timestamps are Unix time
        from an external RTC with proper persistence across deep sleep.
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
        print(f"Expected: Unix timestamps (>{MIN_UNIX_TIMESTAMP})")
        print(f"Cross-cycle tolerance: ±{CROSS_CYCLE_TOLERANCE_SECONDS}s")
        print("=" * 60)

        all_entries: list[LogEntry] = []
        cycle_data_list: list[CycleData] = []
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

            cycle_data = self._capture_cycle(port, baud_rate, timeout)

            # Only count as a cycle if we captured actual data
            if len(cycle_data.entries) > 0:
                completed_cycles += 1
                cycle_data_list.append(cycle_data)
                all_entries.extend(cycle_data.entries)
                sleep_info = f", sleep_duration={cycle_data.sleep_duration}s" if cycle_data.sleep_duration else ""
                print(f"\n-> CYCLE {completed_cycles}/{cycles} complete: captured {len(cycle_data.entries)} entries{sleep_info}")
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
        for i, cycle_data in enumerate(cycle_data_list, 1):
            if cycle_data.entries:
                ts_list = [e.timestamp for e in cycle_data.entries]
                sleep_info = f", sleep={cycle_data.sleep_duration}s" if cycle_data.sleep_duration else ""
                print(f"  Cycle {i}: {len(cycle_data.entries)} entries, timestamps {min(ts_list)}-{max(ts_list)}{sleep_info}")
            else:
                print(f"  Cycle {i}: 0 entries")

        analysis = self._analyze_timestamps(all_entries, cycle_data_list)
        print(analysis.summary())

        # External RTC Assertion
        assert len(analysis.non_unix_timestamps) == 0, (
            f"[External RTC] Found {len(analysis.non_unix_timestamps)} timestamps below {MIN_UNIX_TIMESTAMP}. "
            "External RTC should provide Unix timestamps. Is the RTC connected and configured?"
        )

        # Concurrency Assertions
        assert len(analysis.out_of_order) == 0, (
            f"[Concurrency] Found {len(analysis.out_of_order)} out-of-order timestamps. "
            "Timestamps must be strictly increasing."
        )

        assert len(analysis.duplicates) == 0, (
            f"[Concurrency] Found {len(analysis.duplicates)} duplicate timestamp groups. "
            "Each timestamp must be unique."
        )

        assert analysis.total_entries >= cycles, (
            f"[Concurrency] Expected at least {cycles} entries (one per cycle), got {analysis.total_entries}. "
            "Device may not be logging sensor data."
        )

        # Persistence Assertions
        assert len(analysis.timestamp_resets) == 0, (
            f"[Persistence] Found {len(analysis.timestamp_resets)} cycles where timestamp reset. "
            "External RTC timestamps must persist across deep sleep."
        )

        invalid_gaps = [g for g in analysis.cross_cycle_gaps if not g.is_valid]
        assert len(invalid_gaps) == 0, (
            f"[Persistence] Found {len(invalid_gaps)} invalid cross-cycle timestamp gaps. "
            f"First error: {invalid_gaps[0].error_message if invalid_gaps else 'N/A'}"
        )

        print(f"\nExternal RTC passed: All {analysis.total_entries} timestamps are Unix time")
        print(f"Concurrency passed: {analysis.total_entries} entries with valid monotonic timestamps")
        print(f"Persistence passed: {len(analysis.cross_cycle_gaps)} cross-cycle gaps validated")

    def _capture_cycle(self, port: str, baud_rate: int, timeout: int) -> CycleData:
        """Capture log entries and metadata for a single wake cycle."""
        cycle_data = CycleData()
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
                    return cycle_data

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

                                # Parse sleep duration from debug output
                                sleep_dur = parse_sleep_duration(line)
                                if sleep_dur is not None:
                                    cycle_data.sleep_duration = sleep_dur
                                    print(f"  -> Parsed sleep duration: {sleep_dur}s")

                                # Parse JSONL lines
                                if is_jsonl_line(line):
                                    entry = LogEntry.from_json(line)
                                    if entry:
                                        cycle_data.entries.append(entry)
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

        return cycle_data

    def _analyze_timestamps(
        self,
        entries: list[LogEntry],
        cycle_data_list: Optional[list[CycleData]] = None
    ) -> ExternalRTCAnalysis:
        """Analyze timestamps for external RTC validation."""
        analysis = ExternalRTCAnalysis(entries=entries)

        if not entries:
            return analysis

        # Check for Unix timestamps (external RTC detection)
        for entry in entries:
            if entry.timestamp < MIN_UNIX_TIMESTAMP:
                analysis.non_unix_timestamps.append(entry)

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

        # Cross-cycle persistence analysis
        if cycle_data_list and len(cycle_data_list) > 1:
            self._analyze_cross_cycle_persistence(analysis, cycle_data_list)

        return analysis

    def _analyze_cross_cycle_persistence(
        self,
        analysis: ExternalRTCAnalysis,
        cycle_data_list: list[CycleData]
    ) -> None:
        """Analyze timestamp persistence across deep sleep cycles for external RTC."""
        for i in range(1, len(cycle_data_list)):
            prev_cycle = cycle_data_list[i - 1]
            curr_cycle = cycle_data_list[i]

            # Skip if either cycle has no entries
            if not prev_cycle.entries or not curr_cycle.entries:
                continue

            last_ts = prev_cycle.entries[-1].timestamp
            first_ts = curr_cycle.entries[0].timestamp
            delta = first_ts - last_ts

            # Check for timestamp reset (goes backwards significantly)
            if first_ts < last_ts:
                analysis.timestamp_resets.append(i)

            # Create cross-cycle gap record
            gap = CrossCycleGap(
                cycle_index=i,
                last_timestamp=last_ts,
                first_timestamp=first_ts,
                timestamp_delta=delta,
                reported_sleep_duration=curr_cycle.sleep_duration
            )

            # Validate the gap - for external RTC, delta should match reported sleep duration
            if first_ts < last_ts:
                gap.is_valid = False
                gap.error_message = f"Timestamp went backwards: {last_ts} -> {first_ts}"
            elif curr_cycle.sleep_duration is not None:
                # External RTC: check that delta approximately matches sleep duration
                expected_min = curr_cycle.sleep_duration - CROSS_CYCLE_TOLERANCE_SECONDS
                expected_max = curr_cycle.sleep_duration + CROSS_CYCLE_TOLERANCE_SECONDS

                if delta < expected_min:
                    gap.is_valid = False
                    gap.error_message = (
                        f"Delta {delta}s is too small for sleep duration {curr_cycle.sleep_duration}s "
                        f"(expected {expected_min}s-{expected_max}s)"
                    )
                elif delta > expected_max:
                    gap.is_valid = False
                    gap.error_message = (
                        f"Delta {delta}s is too large for sleep duration {curr_cycle.sleep_duration}s "
                        f"(expected {expected_min}s-{expected_max}s)"
                    )
            else:
                # No reported sleep duration - just check timestamps don't go backwards
                if delta <= 0:
                    gap.is_valid = False
                    gap.error_message = f"Timestamp did not advance: {last_ts} -> {first_ts}"

            analysis.cross_cycle_gaps.append(gap)


# Allow running as standalone script
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="External RTC Timestamp Tests for WipperSnapper Sleep Mode"
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
        default=5,
        help="Number of wake cycles to capture (default: 5)"
    )

    args = parser.parse_args()

    # Run test directly
    test = TestExternalRTCTimestamps()
    test.test_external_rtc_timestamps(args.port, args.timeout, args.baud, args.cycles)
