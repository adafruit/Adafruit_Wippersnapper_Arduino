# SPDX-FileCopyrightText: 2025 Brent Rubell for Adafruit Industries
# SPDX-License-Identifier: MIT
"""
Pytest fixtures for MQTT testing with Wokwi and ProtoMQ.

This module provides fixtures for:
- ProtoMQ service management (MQTT broker)
- Wokwi simulation client
- MQTT client for observing device messages
"""
import os
import sys
import asyncio
from pathlib import Path

import pytest

from wokwi_client import WokwiClient, GET_TOKEN_URL

# Add the WS-Python src directory to the path for ProtoMQ imports
WS_PYTHON_PATH = Path(__file__).parent.parent / "Adafruit_Wippersnapper_Python" / "src"
sys.path.insert(0, str(WS_PYTHON_PATH))

from ProtoMQ.service import ProtoMQService  # noqa: E402
from ProtoMQ.client import ProtoMQClient  # noqa: E402

# Paths to firmware files
BUILD_DIR = Path(__file__).parent.parent.parent / ".pio" / "build" / "esp32dev_mqtt_test"
FIRMWARE_ELF = BUILD_DIR / "firmware.elf"
FIRMWARE_BIN = BUILD_DIR / "firmware.bin"
DIAGRAM_PATH = Path(__file__).parent / "diagram.json"

# Default timeout for waiting on serial output (seconds)
DEFAULT_TIMEOUT = 60

# Global serial buffer for capturing Wokwi output
_serial_output = ""


def on_serial_data(data: bytes) -> None:
    """Callback for serial_monitor - accumulates output."""
    global _serial_output
    decoded = data.decode("utf-8", errors="replace")
    _serial_output += decoded
    # Print serial output in real-time
    for line in decoded.splitlines():
        print(f"[SERIAL] {line}")


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
            print(f"Found: {text!r}")
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


def get_serial_output() -> str:
    """Get the current serial output buffer."""
    return _serial_output


@pytest.fixture(scope="session")
def protomq():
    """
    Start ProtoMQ service for the test session.

    This fixture starts the ProtoMQ MQTT broker and provides
    access to its command API for tracking deliveries.
    """
    service = ProtoMQService()
    service.start()
    yield service
    # Note: ProtoMQService.stop() is async but atexit handles it


@pytest.fixture
async def wokwi(protomq):
    """
    Start Wokwi simulation with private gateway for localhost access.

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
        pytest.skip(f"Firmware not found at {BUILD_DIR}. Run: pio run -e esp32dev_mqtt_test")

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


@pytest.fixture
async def mqtt_client(protomq):
    """
    MQTT client for observing device messages.

    This client connects to ProtoMQ and can subscribe to device topics
    to observe the messages being sent by the firmware.
    """
    async with ProtoMQClient() as client:
        # Subscribe to all topics from our test device
        await client.subscribe("wokwi_test/#")
        yield client
