import pytest
import subprocess


def test_invalid_json():
    # Run the Wokwi CLI
    result = subprocess.run(
        [
            "wokwi-cli",
            "--elf",
            f"../.pio/build/esp32dev/firmware.elf",
            "--timeout",
            "120000",
            "--scenario",
            f"scenarios/offline/test-invalid-json.scenario.yaml",
            "--diagram-file",
            f"diagram.json",
        ]
    )
    assert result.returncode == 0


def test_analog_input():
    # Run the Wokwi CLI
    result = subprocess.run(
        [
            "wokwi-cli",
            "--elf",
            f"../.pio/build/esp32dev/firmware.elf",
            "--timeout",
            "120000",
            "--scenario",
            f"scenarios/offline/test-log-analogin.scenario.yaml",
            "--diagram-file",
            f"diagram.json",
        ]
    )
    assert result.returncode == 0


def test_digital_input():
    # Run the Wokwi CLI
    result = subprocess.run(
        [
            "wokwi-cli",
            "--elf",
            f"../.pio/build/esp32dev/firmware.elf",
            "--timeout",
            "120000",
            "--scenario",
            f"scenarios/offline/test-log-digital-in.scenario.yaml",
            "--diagram-file",
            f"diagram.json",
        ]
    )
    assert result.returncode == 0

def test_ds18b20():
    # Run the Wokwi CLI
    result = subprocess.run(
        [
            "wokwi-cli",
            "--elf",
            f"../.pio/build/esp32dev/firmware.elf",
            "--timeout",
            "120000",
            "--scenario",
            f"scenarios/offline/test-log-ds18b20.scenario.yaml",
            "--diagram-file",
            f"diagram.json",
        ]
    )
    assert result.returncode == 0