"""
Shared utilities for WipperSnapper sleep mode tests.
"""

import json
import re
import serial
import serial.tools.list_ports
import glob
import time
from dataclasses import dataclass, field
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
class CycleData:
    """Data captured during a single wake cycle."""
    entries: list = field(default_factory=list)
    sleep_duration: Optional[int] = None  # Parsed from "Total Sleep Duration (sec): X"


@dataclass
class CrossCycleGap:
    """Represents a timestamp gap between two wake cycles."""
    cycle_index: int  # Index of the later cycle (e.g., 1 means gap between cycle 0 and 1)
    last_timestamp: int  # Last timestamp of previous cycle
    first_timestamp: int  # First timestamp of current cycle
    timestamp_delta: int  # Difference in timestamps
    reported_sleep_duration: Optional[int]  # From device debug output
    is_valid: bool = True  # Whether gap matches expected sleep duration
    error_message: str = ""


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


def parse_sleep_duration(line: str) -> Optional[int]:
    """Parse sleep duration from debug output line."""
    # Matches: "Total Sleep Duration (sec): 10"
    match = re.search(r"Total Sleep Duration \(sec\):\s*(\d+)", line)
    if match:
        return int(match.group(1))
    return None
