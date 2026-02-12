"""
Pytest configuration and fixtures for WipperSnapper tests.
"""

import pytest
from pathlib import Path


def pytest_addoption(parser):
    """Add custom command line options for serial-based tests."""
    parser.addoption(
        "--port",
        action="store",
        default="auto",
        help="Serial port to use (default: auto-detect)"
    )
    parser.addoption(
        "--timeout",
        action="store",
        type=int,
        default=60,
        help="Timeout in seconds to wait for data (default: 60)"
    )
    parser.addoption(
        "--baud",
        action="store",
        type=int,
        default=115200,
        help="Baud rate (default: 115200)"
    )
    parser.addoption(
        "-C", "--cycles",
        action="store",
        type=int,
        default=20,
        help="Number of wake cycles to capture (default: 20)"
    )
    parser.addoption(
        "--sleep-duration",
        action="store",
        type=int,
        default=300,
        help="Expected sleep duration in seconds (default: 300)"
    )
    parser.addoption(
        "--patterns",
        action="store",
        type=str,
        default=None,
        help="Path to patterns markdown file (default: auto-detect based on test)"
    )
    # Offline mode log file validation options
    parser.addoption(
        "--sd-log-file",
        action="store",
        default=None,
        help="Path to JSONL log file from SD card (for offline mode tests)"
    )
    parser.addoption(
        "--sd-config-file",
        action="store",
        default=None,
        help="Path to config.json from SD card (for offline mode tests)"
    )
    parser.addoption(
        "--tolerance",
        action="store",
        type=int,
        default=None,
        help="Allowed deviation in seconds (default: 20%% of sleep duration, min 2s)"
    )
    parser.addoption(
        "--skip-first-cycle",
        action="store_true",
        default=False,
        help="Exclude first boot cycle from timing analysis"
    )
    parser.addoption(
        "--min-cycles",
        action="store",
        type=int,
        default=3,
        help="Minimum wake cycles required to pass (default: 3)"
    )


@pytest.fixture
def serial_port(request):
    """Fixture to get serial port from command line."""
    return request.config.getoption("--port")


@pytest.fixture
def timeout(request):
    """Fixture to get timeout from command line."""
    return request.config.getoption("--timeout")


@pytest.fixture
def baud_rate(request):
    """Fixture to get baud rate from command line."""
    return request.config.getoption("--baud")


@pytest.fixture
def cycles(request):
    """Fixture to get number of wake cycles from command line."""
    return request.config.getoption("--cycles")


@pytest.fixture
def sleep_duration(request):
    """Fixture to get expected sleep duration from command line."""
    return request.config.getoption("--sleep-duration")


@pytest.fixture
def patterns_file(request):
    """Fixture to get patterns file path from command line."""
    patterns_path = request.config.getoption("--patterns")
    if patterns_path:
        return Path(patterns_path)
    return None


@pytest.fixture
def sd_log_file(request) -> Path | None:
    """Path to the JSONL log file for offline mode tests."""
    path = request.config.getoption("--sd-log-file")
    return Path(path) if path else None


@pytest.fixture
def sd_config_file(request) -> Path | None:
    """Path to config.json for offline mode tests."""
    path = request.config.getoption("--sd-config-file")
    return Path(path) if path else None


@pytest.fixture
def tolerance(request) -> int | None:
    """Tolerance override for sleep timing validation."""
    return request.config.getoption("--tolerance")


@pytest.fixture
def skip_first_cycle(request) -> bool:
    """Whether to skip the first boot cycle."""
    return request.config.getoption("--skip-first-cycle")


@pytest.fixture
def min_cycles(request) -> int:
    """Minimum number of cycles required."""
    return request.config.getoption("--min-cycles")
