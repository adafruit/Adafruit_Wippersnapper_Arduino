# SPDX-FileCopyrightText: 2025 Brent Rubell for Adafruit Industries
# SPDX-License-Identifier: MIT
"""
Tests for WipperSnapper device checkin over MQTT using ProtoMQ.

This module tests the device's MQTT connection and checkin handshake
using a Wokwi simulation connected to a local ProtoMQ MQTT broker.

Usage:
    cd tests/mqtt
    python -m pytest test_checkin.py -v -s

Requirements:
    - WOKWI_CLI_TOKEN environment variable must be set
    - Firmware must be built: pio run -e esp32dev_mqtt_test
    - ProtoMQ must be available (auto-started by fixtures)
"""
import pytest
import asyncio

from conftest import wait_for_text, get_serial_output


@pytest.mark.asyncio
async def test_device_boots_and_connects_wifi(wokwi, protomq):
    """Test that device boots and connects to Wokwi-GUEST WiFi."""
    # Wait for firmware boot message
    await wait_for_text("Adafruit.io WipperSnapper")

    # Verify pre-baked credentials are being used
    await wait_for_text("WOKWI_MQTT_TEST: Using pre-baked credentials")

    # Check WiFi SSID is set correctly
    await wait_for_text("WiFi SSID: Wokwi-GUEST")

    # Verify MQTT broker settings
    # host.wokwi.internal is the hostname for localhost when using Wokwi Private Gateway
    await wait_for_text("AIO URL: host.wokwi.internal")
    await wait_for_text("AIO Port: 1884")


@pytest.mark.asyncio
async def test_device_mqtt_connection(wokwi, protomq):
    """Test that device successfully connects to MQTT broker."""
    # Wait for firmware to boot and configure
    await wait_for_text("WOKWI_MQTT_TEST: Using pre-baked credentials")

    # Wait for MQTT client setup
    await wait_for_text("Setting up MQTT client...")
    await wait_for_text("Set up MQTT client successfully!")

    # Wait for MQTT topic generation
    await wait_for_text("Generating device's MQTT topics...")
    await wait_for_text("Generated device's MQTT topics successfully!")


@pytest.mark.asyncio
async def test_device_checkin_request(wokwi, protomq):
    """Test that device sends a checkin request after connecting."""
    # Start tracking deliveries for our test device
    await protomq.command.track_deliveries("wokwi_test")

    # Wait for device to complete MQTT connection
    await wait_for_text("Generated device's MQTT topics successfully!")

    # Wait for network FSM and possible checkin attempt
    await wait_for_text("Running Network FSM...")

    # Give firmware time to connect and send checkin
    await asyncio.sleep(15)

    # Dump tracked messages from ProtoMQ
    inbox, outbox = await protomq.command.dump_deliveries("wokwi_test")

    # Log what was captured for debugging
    print(f"\n=== ProtoMQ Deliveries for 'wokwi_test' ===")
    print(f"Inbox (messages TO device): {len(inbox)}")
    for msg in inbox:
        print(f"  - {msg}")
    print(f"Outbox (messages FROM device): {len(outbox)}")
    for msg in outbox:
        print(f"  - {msg}")

    # Device should have sent at least one message (checkin request)
    assert len(outbox) >= 1, (
        f"Expected device to send at least 1 message, got {len(outbox)}. "
        f"Serial output:\n{get_serial_output()}"
    )
