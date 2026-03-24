# SPDX-FileCopyrightText: 2025, Brent Rubell for Adafruit Industries
# SPDX-License-Identifier: MIT
# SPDX-FileDescription: Unit tests for WipperSnapper Offline Mode Functionality
"""
Tests for WipperSnapper Offline Mode using wokwi-client Python library.

This module uses the native async WokwiClient API to run simulations
and validate serial output.

Usage:
    cd tests
    python -m pytest test_offline.py -v

Requirements:
    - WOKWI_CLI_TOKEN environment variable must be set
    - Firmware must be built: pio run -e esp32dev
"""
import pytest
import os
import asyncio
from pathlib import Path

from wokwi_client import WokwiClient, GET_TOKEN_URL


# Paths to firmware files
BUILD_DIR = Path(
    "/Users/brentrubell/Documents/Arduino/libraries/Adafruit_Wippersnapper_Arduino/.pio/build/esp32dev"
)
FIRMWARE_ELF = BUILD_DIR / "firmware.elf"
FIRMWARE_BIN = BUILD_DIR / "firmware.bin"
DIAGRAM_PATH = Path(__file__).parent / "diagrams" / "offline.json"

# Default timeout for waiting on serial output (seconds)
DEFAULT_TIMEOUT = 60

# Global serial buffer
_serial_output = ""


def on_serial_data(data: bytes) -> None:
    """Callback for serial_monitor - accumulates output."""
    global _serial_output
    decoded = data.decode("utf-8", errors="replace")
    _serial_output += decoded
    # Print serial output in real-time
    for line in decoded.splitlines():
        print(f"{line}")


async def wait_for_text(text: str, timeout: float = DEFAULT_TIMEOUT) -> bool:
    """
    Async wait until the specified text appears in the serial buffer.

    Args:
        text: The text to wait for
        timeout: Maximum time to wait in seconds

    Returns:
        True if text was found

    Raises:
        TimeoutError: If text is not found within timeout
    """
    print(f"Waiting for: {text!r}")
    start = asyncio.get_event_loop().time()
    while asyncio.get_event_loop().time() - start < timeout:
        if text in _serial_output:
            print(f"✓ Found: {text!r}")
            return True
        await asyncio.sleep(0.1)

    # Timeout reached - raise with helpful debug info
    raise TimeoutError(
        f"Timed out after {timeout}s waiting for: {text!r}\n"
        f"Buffer contents ({len(_serial_output)} chars):\n{_serial_output}"
    )


def clear_serial() -> None:
    """Clear the serial buffer."""
    global _serial_output
    _serial_output = ""


@pytest.fixture
async def client():
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
    clear_serial()

    token = os.getenv("WOKWI_CLI_TOKEN")
    if not token:
        pytest.skip(f"WOKWI_CLI_TOKEN not set. Get it from {GET_TOKEN_URL}")

    if not FIRMWARE_ELF.exists() or not FIRMWARE_BIN.exists():
        pytest.skip(f"Firmware not found at {BUILD_DIR}. Run: pio run -e esp32dev")

    if not DIAGRAM_PATH.exists():
        pytest.skip(f"Diagram not found at {DIAGRAM_PATH}")

    wokwi_client = WokwiClient(token)
    monitor_task = None

    try:
        print("Connecting to Wokwi...")
        await wokwi_client.connect()
        print("Uploading diagram.json...")
        await wokwi_client.upload_file("diagram.json", DIAGRAM_PATH)
        print("Uploading firmware.bin...")
        await wokwi_client.upload_file("firmware.bin", FIRMWARE_BIN)
        print("Uploading firmware.elf...")
        await wokwi_client.upload_file("firmware.elf", FIRMWARE_ELF)
        print("Starting simulation...")
        await wokwi_client.start_simulation(firmware="firmware.bin", elf="firmware.elf")

        # Start serial monitor as background task
        monitor_task = wokwi_client.serial_monitor(on_serial_data)
        print("Simulation running, serial monitor active")

        yield wokwi_client
    finally:
        if monitor_task is not None:
            monitor_task.cancel()
            try:
                await monitor_task
            except asyncio.CancelledError:
                pass
        await wokwi_client.disconnect()


# =============================================================================
# Test JSON Validation
# =============================================================================


@pytest.mark.asyncio
async def test_invalid_json(client):
    """Test that invalid JSON is properly rejected."""
    await wait_for_text("[SD] Waiting for incoming JSON string...")
    await client.serial_write('{"exportVersion":"1.0.0",\\n')
    await wait_for_text("[SD] Runtime Error: Unable to deserialize config.json")


@pytest.mark.asyncio
async def test_invalid_checksum(client):
    """Test that JSON with invalid checksum is rejected."""
    await wait_for_text("[SD] Waiting for incoming JSON string...")
    # JSON with checksum=5 which is incorrect
    await client.serial_write(
        '{"exportVersion": "1.0.0", "exportedBy": "wokwi", "exportedAt": "2024-10-28T18:58:23.976Z", '
        '"exportedFromDevice": {"board": "metroesp32s3", "firmwareVersion": "1.0.0-beta.93", '
        '"referenceVoltage": 2.6, "totalGPIOPins": 11, "totalAnalogPins": 6}, '
        '"components": [{"componentAPI": "analogio", "name": "Analog Pin", "pinName": "D14", '
        '"type": "analog_pin", "mode": "ANALOG", "direction": "INPUT", "sampleMode": "TIMER", '
        '"analogReadMode": "PIN_VALUE", "period": 5, "isPin": true}], "checksum": 5}\\n'
    )
    await wait_for_text("[SD] Checksum mismatch, file has been modified from its original state!")


@pytest.mark.asyncio
async def test_valid_checksum(client):
    """Test that JSON with valid checksum is accepted."""
    await wait_for_text("[SD] Waiting for incoming JSON string...")
    # JSON with correct checksum=28
    await client.serial_write(
        '{"exportVersion": "1.0.0", "exportedBy": "wokwi", "exportedAt": "2024-10-28T18:58:23.976Z", '
        '"exportedFromDevice": {"board": "metroesp32s3", "firmwareVersion": "1.0.0-beta.93", '
        '"referenceVoltage": 2.6, "totalGPIOPins": 11, "totalAnalogPins": 6}, '
        '"components": [{"componentAPI": "analogio", "name": "Analog Pin", "pinName": "D14", '
        '"type": "analog_pin", "mode": "ANALOG", "direction": "INPUT", "sampleMode": "TIMER", '
        '"analogReadMode": "raw", "period": 5, "isPin": true}], "checksum": 28}\\n'
    )
    await wait_for_text("[SD] Successfully deserialized JSON config file!")


# =============================================================================
# Test Hardware Validation
# =============================================================================


@pytest.mark.asyncio
async def test_digital_input(client):
    """Test digital input pin with button press."""
    await wait_for_text("[SD] Waiting for incoming JSON string...")
    # Configure digital input on D4 with pull-up
    await client.serial_write(
        '{"checksum":183,"components":[{"componentAPI":"digitalio","direction":"INPUT",'
        '"isPin":true,"mode":"DIGITAL","name":"Button (D4)","period":5,"pinName":"D4",'
        '"pull":"UP","sampleMode":"EVENT","type":"push_button"}],"exportVersion":"1.0.0",'
        '"exportedAt":"2024-10-28T18:58:23.976Z","exportedBy":"wokwi",'
        '"exportedFromDevice":{"board":"metroesp32s3","firmwareVersion":"1.0.0-beta.93",'
        '"referenceVoltage":2.6,"totalAnalogPins":6,"totalGPIOPins":11}}\\n'
    )

    # Wait for pin configuration
    await wait_for_text("[SD] JSON string received!")
    await wait_for_text("[digitalio] Added new pin:")
    await wait_for_text("Pin Name: 4")

    # Wait for initial pin state (button not pressed = true with pull-up)
    await wait_for_text('{"timestamp":0,"pin":"D4","value":true,"si_unit":"boolean"}')

    # Press and hold the button
    await client.set_control("btn1", "pressed", 1)
    await asyncio.sleep(0.5)

    # Wait for button pressed state (value = false)
    await wait_for_text('{"timestamp":0,"pin":"D4","value":false,"si_unit":"boolean"}')

    # Release the button
    await client.set_control("btn1", "pressed", 0)


@pytest.mark.asyncio
async def test_analog_input(client):
    """Test analog input pin with potentiometer."""
    await wait_for_text("[SD] Waiting for incoming JSON string...")
    # Configure analog input on D14
    await client.serial_write(
        '{"exportVersion": "1.0.0", "exportedBy": "wokwi", "exportedAt": "2024-10-28T18:58:23.976Z", '
        '"exportedFromDevice": {"board": "metroesp32s3", "firmwareVersion": "1.0.0-beta.93", '
        '"referenceVoltage": 2.6, "totalGPIOPins": 11, "totalAnalogPins": 6}, '
        '"components": [{"componentAPI": "analogio", "name": "Analog Pin", "pinName": "D14", '
        '"type": "analog_pin", "mode": "ANALOG", "direction": "INPUT", "sampleMode": "TIMER", '
        '"analogReadMode": "raw", "period": 5, "isPin": true}], "checksum": 149}\\n'
    )
    await client.serial_write("\n")

    # Wait for pin configuration
    await wait_for_text("[analogio] Added new pin:")
    await wait_for_text("Pin Name: 14")
    await wait_for_text("Period: 5000")
    await wait_for_text("Read Mode: 18")

    # Wait for initial analog reading (pot at 0 position)
    await asyncio.sleep(5)
    await wait_for_text('{"timestamp":0,"pin":"A14","value":0,"si_unit":"none"}')

    # Move potentiometer to middle position
    await client.set_control("pot1", "position", 0.5)

    # Wait for new analog reading
    await asyncio.sleep(3)
    await wait_for_text('{"timestamp":0,"pin":"A14","value":16384,"si_unit":"none"}')


@pytest.mark.asyncio
async def test_ds18b20(client):
    """Test DS18B20 temperature sensor."""
    await wait_for_text("[SD] Waiting for incoming JSON string...")
    # Configure DS18B20 on D25
    await client.serial_write(
        '{"exportVersion": "1.0.0", "exportedBy": "wokwi", "exportedAt": "2024-10-28T18:58:23.976Z", '
        '"exportedFromDevice": {"board": "metroesp32s3", "firmwareVersion": "1.0.0-beta.93", '
        '"referenceVoltage": 2.6, "totalGPIOPins": 11, "totalAnalogPins": 6}, '
        '"components": [{"componentAPI": "ds18x20", "name": "DS18B20: Temperature Sensor", '
        '"sensorTypeCount": 2, "sensorType1": "object-temp-fahrenheit", "sensorType2": "object-temp", '
        '"pinName": "D25", "sensorResolution": 12, "period": 5}], "checksum": 34}\\n'
    )

    # Wait for sensor initialization and readings
    await wait_for_text("Sensor found on OneWire bus and initialized")
    await wait_for_text('{"timestamp":0,"pin":"D25","value":22,"si_unit":"C"}')
    await wait_for_text('{"timestamp":0,"pin":"D25","value":71.6,"si_unit":"F"}')
