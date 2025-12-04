# SPDX-FileCopyrightText: 2025, Brent Rubell for Adafruit Industries
# SPDX-License-Identifier: MIT
# SPDX-FileDescription: Unit tests for WipperSnapper Offline Mode Functionality
import pytest
import subprocess
import shutil
import os


# Path to the firmware.elf file
FIRMWARE_ELF = "/Users/brentrubell/Documents/Arduino/libraries/Adafruit_Wippersnapper_Arduino/.pio/build/esp32dev/firmware.elf"
# Copy to tests directory where diagram file expects it
TEST_ELF_PATH = "test.elf"

@pytest.fixture(scope="session", autouse=True)
def setup_firmware():
    """Copy firmware.elf to tests directory and rename to test.elf before running tests"""
    # Copy firmware to current directory (tests/) and rename it
    shutil.copy2(FIRMWARE_ELF, TEST_ELF_PATH)
    print(f"\nCopied {FIRMWARE_ELF} to {os.path.abspath(TEST_ELF_PATH)}")
    print(f"File exists: {os.path.exists(TEST_ELF_PATH)}")
    print(f"Current working directory: {os.getcwd()}")
    yield
    # Cleanup after tests
    if os.path.exists(TEST_ELF_PATH):
        os.remove(TEST_ELF_PATH)

def run_wokwi_cli(binary, timeout, scenario, diagram):
    result = subprocess.run(
        [
            "wokwi-cli",
            "--elf",
            binary,
            "--timeout",
            timeout,
            "--scenario",
            scenario,
            "--diagram-file",
            diagram,
        ]
    )
    return result

# Test JSON validation

def test_invalid_json():
    result = run_wokwi_cli(FIRMWARE_ELF, "120000", f"scenarios/offline/test-invalid-json.scenario.yaml", f"diagrams/offline.json")
    assert result.returncode == 0


def test_invalid_checksum():
    result = run_wokwi_cli(FIRMWARE_ELF, "120000", f"scenarios/offline/test-invalid-checksum.scenario.yaml", f"diagrams/offline.json")
    assert result.returncode == 0


def test_valid_checksum():
    result = run_wokwi_cli(FIRMWARE_ELF, "120000", f"scenarios/offline/test-valid-checksum.scenario.yaml", f"diagrams/offline.json")
    assert result.returncode == 0


# Test hardware validation
def test_digital_input():
    result = run_wokwi_cli("test.elf", "120000", f"scenarios/offline/test-log-digital-in.scenario.yaml", f"diagrams/offline.json")
    assert result.returncode == 0


def test_analog_input():
    result = run_wokwi_cli(FIRMWARE_ELF, "120000", f"scenarios/offline/test-log-analogin.scenario.yaml", f"diagrams/offline.json")
    assert result.returncode == 0


def test_ds18b20():
    result = run_wokwi_cli(FIRMWARE_ELF, "120000", f"scenarios/offline/test-log-ds18b20.scenario.yaml", f"diagrams/offline.json")
    assert result.returncode == 0

