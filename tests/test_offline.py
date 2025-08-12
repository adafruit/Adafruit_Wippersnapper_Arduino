# SPDX-FileCopyrightText: 2024-2025, Brent Rubell for Adafruit Industries
# SPDX-License-Identifier: MIT
# SPDX-FileDescription: Unit tests for WipperSnapper Offline Mode Functionality
import pytest
import subprocess

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

def test_digital_input():
    result = run_wokwi_cli(f"tests/bin/offline/firmware.elf", "120000", f"tests/scenarios/offline/test-log-digital-in.scenario.yaml", f"tests/diagrams/offline.json")
    assert result.returncode == 0


def test_analog_input():
    result = run_wokwi_cli(f"tests/bin/offline/firmware.elf", "120000", f"tests/scenarios/offline/test-log-analogin.scenario.yaml", f"tests/diagrams/offline.json")
    assert result.returncode == 0


"""def test_ds18b20():
    result = run_wokwi_cli(f"tests/bin/offline/firmware.elf", "120000", f"tests/scenarios/offline/test-log-ds18b20.scenario.yaml", f"tests/diagrams/offline.json")
    assert result.returncode == 0
"""


def test_invalid_json():
    result = run_wokwi_cli(f"tests/bin/offline/firmware.elf", "120000", f"tests/scenarios/offline/test-invalid-json.scenario.yaml", f"tests/diagrams/offline.json")
    assert result.returncode == 0


def test_invalid_checksum():
    result = run_wokwi_cli(f"tests/bin/offline/firmware.elf", "120000", f"tests/scenarios/offline/test-invalid-checksum.scenario.yaml", f"tests/diagrams/offline.json")
    assert result.returncode == 0


def test_valid_checksum():
    result = run_wokwi_cli(f"tests/bin/offline/firmware.elf", "120000", f"tests/scenarios/offline/test-valid-checksum.scenario.yaml", f"tests/diagrams/offline.json")
    assert result.returncode == 0