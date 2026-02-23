# SPDX-FileCopyrightText: 2025, Brent Rubell for Adafruit Industries
# SPDX-License-Identifier: MIT
# SPDX-FileDescription: Unit tests for WipperSnapper Offline Mode Functionality
"""
Tests for WipperSnapper Offline Mode using wokwi-client Python library.

This module uses the synchronous WokwiClientSync API to run simulations
and validate serial output, replacing the previous wokwi-cli subprocess approach.

Usage:
    cd tests
    python -m pytest test_offline.py -v

Requirements:
    - WOKWI_CLI_TOKEN environment variable must be set
    - Firmware must be built: pio run -e esp32dev
"""
import pytest
import os
import time
import threading
from pathlib import Path

from wokwi_client import WokwiClientSync, GET_TOKEN_URL


# Paths to firmware files
BUILD_DIR = Path(
    "/Users/brentrubell/Documents/Arduino/libraries/Adafruit_Wippersnapper_Arduino/.pio/build/esp32dev"
)
FIRMWARE_ELF = BUILD_DIR / "firmware.elf"
FIRMWARE_BIN = BUILD_DIR / "firmware.bin"
DIAGRAM_PATH = Path(__file__).parent / "diagrams" / "offline.json"

# Default timeout for waiting on serial output (seconds)
DEFAULT_TIMEOUT = 60


class SerialBuffer:
    """Thread-safe buffer for capturing serial output from the simulator."""

    def __init__(self):
        self._buffer = ""
        self._lock = threading.Lock()

    def append(self, data: bytes) -> None:
        """Append data to the buffer (called from serial monitor callback)."""
        with self._lock:
            self._buffer += data.decode("utf-8", errors="replace")

    def get(self) -> str:
        """Get the current buffer contents."""
        with self._lock:
            return self._buffer

    def clear(self) -> None:
        """Clear the buffer."""
        with self._lock:
            self._buffer = ""

    def wait_for(self, text: str, timeout: float = DEFAULT_TIMEOUT) -> bool:
        """
        Wait until the specified text appears in the buffer.

        Args:
            text: The text to wait for
            timeout: Maximum time to wait in seconds

        Returns:
            True if text was found

        Raises:
            TimeoutError: If text is not found within timeout
        """
        start = time.time()
        while time.time() - start < timeout:
            if text in self.get():
                return True
            time.sleep(0.1)

        # Timeout reached - raise with helpful debug info
        current_buffer = self.get()
        raise TimeoutError(
            f"Timed out after {timeout}s waiting for: {text!r}\n"
            f"Buffer contents ({len(current_buffer)} chars):\n{current_buffer}"
        )


# Global serial buffer instance
serial_buffer = SerialBuffer()


def on_serial_data(data: bytes) -> None:
    """Callback for serial monitor - appends data to global buffer."""
    serial_buffer.append(data)


@pytest.fixture
def client():
    """
    Create a connected Wokwi client with simulation running.

    This fixture:
    1. Checks for WOKWI_CLI_TOKEN environment variable
    2. Connects to the Wokwi simulator
    3. Uploads diagram and firmware files
    4. Starts the simulation with serial monitoring
    5. Yields the client for test use
    6. Disconnects when test completes
    """
    serial_buffer.clear()

    token = os.getenv("WOKWI_CLI_TOKEN")
    if not token:
        pytest.skip(f"WOKWI_CLI_TOKEN not set. Get it from {GET_TOKEN_URL}")

    if not FIRMWARE_ELF.exists() or not FIRMWARE_BIN.exists():
        pytest.skip(f"Firmware not found at {BUILD_DIR}. Run: pio run -e esp32dev")

    if not DIAGRAM_PATH.exists():
        pytest.skip(f"Diagram not found at {DIAGRAM_PATH}")

    client = WokwiClientSync(token)

    try:
        client.connect()
        client.upload_file("diagram.json", DIAGRAM_PATH)
        client.upload_file("firmware.bin", FIRMWARE_BIN)
        client.upload_file("firmware.elf", FIRMWARE_ELF)
        client.start_simulation(firmware="firmware.bin", elf="firmware.elf")
        client.serial_monitor(on_serial_data)

        yield client
    finally:
        client.disconnect()


# =============================================================================
# Test JSON Validation
# =============================================================================


def test_invalid_json(client):
    """Test that invalid JSON is properly rejected."""
    serial_buffer.wait_for("[SD] Waiting for incoming JSON string...")
    client.serial_write('{"exportVersion":"1.0.0",')
    client.serial_write("\n")
    serial_buffer.wait_for("[SD] Runtime Error: Unable to deserialize config.json")


def test_invalid_checksum(client):
    """Test that JSON with invalid checksum is rejected."""
    serial_buffer.wait_for("[SD] Waiting for incoming JSON string...")
    # JSON with checksum=5 which is incorrect
    client.serial_write(
        '{"exportVersion": "1.0.0", "exportedBy": "wokwi", "exportedAt": "2024-10-28T18:58:23.976Z", '
        '"exportedFromDevice": {"board": "metroesp32s3", "firmwareVersion": "1.0.0-beta.93", '
        '"referenceVoltage": 2.6, "totalGPIOPins": 11, "totalAnalogPins": 6}, '
        '"components": [{"componentAPI": "analogio", "name": "Analog Pin", "pinName": "D14", '
        '"type": "analog_pin", "mode": "ANALOG", "direction": "INPUT", "sampleMode": "TIMER", '
        '"analogReadMode": "PIN_VALUE", "period": 5, "isPin": true}], "checksum": 5}'
    )
    client.serial_write("\n")
    serial_buffer.wait_for(
        "[SD] Checksum mismatch, file has been modified from its original state!"
    )


def test_valid_checksum(client):
    """Test that JSON with valid checksum is accepted."""
    serial_buffer.wait_for("[SD] Waiting for incoming JSON string...")
    # JSON with correct checksum=28
    client.serial_write(
        '{"exportVersion": "1.0.0", "exportedBy": "wokwi", "exportedAt": "2024-10-28T18:58:23.976Z", '
        '"exportedFromDevice": {"board": "metroesp32s3", "firmwareVersion": "1.0.0-beta.93", '
        '"referenceVoltage": 2.6, "totalGPIOPins": 11, "totalAnalogPins": 6}, '
        '"components": [{"componentAPI": "analogio", "name": "Analog Pin", "pinName": "D14", '
        '"type": "analog_pin", "mode": "ANALOG", "direction": "INPUT", "sampleMode": "TIMER", '
        '"analogReadMode": "raw", "period": 5, "isPin": true}], "checksum": 28}'
    )
    client.serial_write("\n")
    serial_buffer.wait_for("[SD] Checksum OK!")


# =============================================================================
# Test Hardware Validation
# =============================================================================


def test_digital_input(client):
    """Test digital input pin with button press."""
    serial_buffer.wait_for("[SD] Waiting for incoming JSON string...")
    # Configure digital input on D4 with pull-up
    client.serial_write(
        '{"checksum":183,"components":[{"componentAPI":"digitalio","direction":"INPUT",'
        '"isPin":true,"mode":"DIGITAL","name":"Button (D4)","period":5,"pinName":"D4",'
        '"pull":"UP","sampleMode":"TIMER","type":"push_button"}],"exportVersion":"1.0.0",'
        '"exportedAt":"2024-10-28T18:58:23.976Z","exportedBy":"wokwi",'
        '"exportedFromDevice":{"board":"metroesp32s3","firmwareVersion":"1.0.0-beta.93",'
        '"referenceVoltage":2.6,"totalAnalogPins":6,"totalGPIOPins":11}}'
    )
    client.serial_write("\n")

    # Wait for pin configuration
    serial_buffer.wait_for("[SD] JSON string received!")
    serial_buffer.wait_for("[digitalio] Added new pin:")
    serial_buffer.wait_for("Pin Name: 4")
    serial_buffer.wait_for("Period: 5000")
    serial_buffer.wait_for("Sample Mode: 1")
    serial_buffer.wait_for("Direction: 2")

    # Wait for initial pin state (button not pressed = true with pull-up)
    serial_buffer.wait_for('{"timestamp":0,"pin":"D4","value":true,"si_unit":"boolean"}')

    # Press the button
    client.set_control("btn1", "pressed", 1)

    # Wait for button pressed state (value = false)
    serial_buffer.wait_for(
        '{"timestamp":0,"pin":"D4","value":false,"si_unit":"boolean"}'
    )


def test_analog_input(client):
    """Test analog input pin with potentiometer."""
    serial_buffer.wait_for("[SD] Waiting for incoming JSON string...")
    # Configure analog input on D14
    client.serial_write(
        '{"exportVersion": "1.0.0", "exportedBy": "wokwi", "exportedAt": "2024-10-28T18:58:23.976Z", '
        '"exportedFromDevice": {"board": "metroesp32s3", "firmwareVersion": "1.0.0-beta.93", '
        '"referenceVoltage": 2.6, "totalGPIOPins": 11, "totalAnalogPins": 6}, '
        '"components": [{"componentAPI": "analogio", "name": "Analog Pin", "pinName": "D14", '
        '"type": "analog_pin", "mode": "ANALOG", "direction": "INPUT", "sampleMode": "TIMER", '
        '"analogReadMode": "raw", "period": 5, "isPin": true}], "checksum": 149}'
    )
    client.serial_write("\n")

    # Wait for pin configuration
    serial_buffer.wait_for("[analogio] Added new pin:")
    serial_buffer.wait_for("Pin Name: 14")
    serial_buffer.wait_for("Period: 5000.00")
    serial_buffer.wait_for("Read Mode: 18")

    # Wait for initial analog reading (pot at 0 position)
    time.sleep(5)
    serial_buffer.wait_for('{"timestamp":0,"pin":"A14","value":0,"si_unit":"none"}')

    # Move potentiometer to middle position
    client.set_control("pot1", "position", 0.5)

    # Wait for new analog reading
    time.sleep(3)
    serial_buffer.wait_for('{"timestamp":0,"pin":"A14","value":16384,"si_unit":"none"}')


def test_ds18b20(client):
    """Test DS18B20 temperature sensor."""
    serial_buffer.wait_for("[SD] Waiting for incoming JSON string...")
    # Configure DS18B20 on D25
    client.serial_write(
        '{"exportVersion": "1.0.0", "exportedBy": "wokwi", "exportedAt": "2024-10-28T18:58:23.976Z", '
        '"exportedFromDevice": {"board": "metroesp32s3", "firmwareVersion": "1.0.0-beta.93", '
        '"referenceVoltage": 2.6, "totalGPIOPins": 11, "totalAnalogPins": 6}, '
        '"components": [{"componentAPI": "ds18x20", "name": "DS18B20: Temperature Sensor", '
        '"sensorTypeCount": 2, "sensorType1": "object-temp-fahrenheit", "sensorType2": "object-temp", '
        '"pinName": "D25", "sensorResolution": 12, "period": 5}], "checksum": 34}'
    )
    client.serial_write("\n")

    # Wait for sensor initialization and readings
    serial_buffer.wait_for("Sensor found on OneWire bus and initialized")
    serial_buffer.wait_for('{"timestamp":0,"pin":"D25","value":0.5,"si_unit":"C"}')
    serial_buffer.wait_for('{"timestamp":0,"pin":"D25","value":32.9,"si_unit":"F"}')
