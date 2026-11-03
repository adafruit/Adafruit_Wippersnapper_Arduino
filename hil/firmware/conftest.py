"""Fixtures for the firmware HIL tests, which run against the board in the
board file (hil-boards.json by default).

    pytest firmware/provisioning/
    pytest firmware -k io_user
    pytest firmware --log-cli-level=INFO

Fixture chain:

    board (session) -> flashed_board (function) -> serial (function)
                                                -> device_uid (function)
                                                -> protomq (function)
    board, serial, device_uid, protomq -> checkin (function)
    board (session) -> probe (function)

Every system test starts from a clean chip: flashed_board erases the whole
flash (firmware, NVS, filesystem), then flashes the firmware and a LittleFS
image holding the test's secrets scenario, before the test body runs. The
api tests share one flash: the first erases and flashes, and the rest reuse
it while their secrets scenario matches (they power-cycle the board
themselves). The api tests run before the others.

Each scenario's secrets are built here and written to a temp file; none
are read from disk. The valid credentials come from environment variables
set on the HIL host, never from the repo:

    HIL_IO_USERNAME, HIL_IO_KEY      required by the scenarios with valid IO
    HIL_WIFI_SSID, HIL_WIFI_PASSWORD required by the scenarios with valid WiFi
                                     (the password may be empty: open network)
    HIL_PROTOMQ_HOST                 optional, the address the DUT reaches
                                     this host at, default its LAN IP
    HIL_PROTOMQ_PORT                 optional, ProtoMQ's MQTT port, default 1884
    HIL_PROTOMQ_HTTP_PORT            optional, ProtoMQ's API and web UI port,
                                     default a free port
    HIL_PROTOMQ_DIR                  optional, ProtoMQ's checkout, default
                                     ~/ProtoMQ
    HIL_WS_BOARDS_DIR                optional, the Wippersnapper_Boards
                                     checkout, default ~/Wippersnapper_Boards

The valid IO credentials point the DUT at a ProtoMQ broker on this host,
which the protomq fixture starts for the test and stops after it, with
its output in <test>.protomq.log next to the serial log. A test that
needs the broker must request protomq; without it nothing listens.

The scenarios with valid credentials are "good" (the default: valid WiFi
and IO), "valid_io_only" and "valid_wifi_only" (the other half keeps the
placeholders), "ssid_valid" (valid WiFi, fake IO credentials that pass
provisioning), "invalid_password" (as "ssid_valid", but with a wrong
WiFi password), and "invalid_broker_url" and "invalid_broker_port" (as
"good", but with a broker host that doesn't resolve, or the broker host
with a port nothing listens on). A test only needs the variables its
scenario uses. The
scenarios in generated_secrets (e.g. "default", the firmware's placeholder
secrets) need none, and may be invalid on purpose. Two scenarios have no
secrets file at all: "no_secrets_file" flashes an empty LittleFS image, and
"no_fs_image" flashes no image, leaving the fs partition erased. A module
picks a scenario by overriding secrets_name:

    @pytest.fixture
    def secrets_name():
        return "bad_wifi"

and a single test by parametrizing it indirectly:

    @pytest.mark.parametrize("secrets_name", ["default"], indirect=True)

The board entry picks which tests run on it, with selectors relative to
this dir: a folder ("provisioning/"), a file ("provisioning/test_secrets.py")
or a test ("provisioning/test_secrets.py::test_default_secrets"). They're
split in two suites: api_tests drive the firmware's API over ProtoMQ, and
system_tests reprovision and reflash the board, so they're much slower:

    "api_tests": ["checkin/", "digital/"],
    "system_tests": ["provisioning/", "network/", "mqtt/"],
    "skip_tests": []                  # these are reported as skipped

--suite api or --suite system runs only that suite; without it both run.
Omit both lists to run every test. Tests outside the selected lists are
deselected. Selection on the command line (paths, -k) narrows this
further; it can't widen it.

The board entry's wippersnapper_boards_id is the firmware's BOARD_ID,
which names the board's definition in Wippersnapper_Boards (the
board_definition fixture), e.g.:

    "wippersnapper_boards_id": "feather-esp32-v2"

The board entry's digital_gpio probe describes the I2C GPIO expander on
this host that the DUT's pins are wired to, which the probe fixture reads
them through (see hil_probe.py). pin_map maps a DUT pin to an expander pin, GPA0-GPA7
or GPB0-GPB7. The probe makes all 16 expander pins inputs with no pull
when it's opened, before every uhubctl and esptool run while the test
holds it, and when the test ends, so it never drives the DUT's pins
through a power change or flash. Tests that request probe are skipped on
boards without one. The pwm_input probe, a host GPIO wired to a DUT PWM
output, works the same way through the pulse_probe fixture, and the
analog_input probe, an MCP4728 DAC channel wired to a DUT analog input,
through the dac_probe fixture, which also powers down all four channels
before every power change or flash:

    "probes": {
      "digital_gpio": {"type": "mcp23017_gpio", "address": "0x20",
                       "pin_map": {"D14": "GPA0"}},
      "pwm_input": {"type": "pi_gpio", "pin_map": {"D27": "D16"}},
      "analog_input": {"type": "mcp4728_dac", "address": "0x60",
                       "pin_map": {"A34": "A"}}
    }
"""

import copy
import json
import logging
import os
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import pytest

FIRMWARE_DIR = Path(__file__).resolve().parent
HIL_DIR = FIRMWARE_DIR.parent
sys.path.insert(0, str(HIL_DIR))
import hil_board_utils  # noqa: E402
import hil_probe  # noqa: E402
import hil_protomq  # noqa: E402

DEFAULT_BOARDS = HIL_DIR / "hil-boards.json"
DEFAULT_WS_BOARDS_DIR = "~/Wippersnapper_Boards"
SUITES = ("api", "system")  # the board entry's api_tests, system_tests
SECRETS_SUFFIX = ".secrets.json"
GOOD_SECRETS = "good"  # valid WiFi and IO
VALID_IO_ONLY = "valid_io_only"
VALID_WIFI_ONLY = "valid_wifi_only"
SSID_VALID = "ssid_valid"  # valid WiFi, FAKE_IO_CREDS
INVALID_PASSWORD = "invalid_password"  # valid SSID, wrong password, FAKE_IO_CREDS
INVALID_BROKER_URL = "invalid_broker_url"  # as GOOD_SECRETS, MISSING_BROKER_HOST
INVALID_BROKER_PORT = "invalid_broker_port"  # as GOOD_SECRETS, CLOSED_BROKER_PORT
DEFAULT_SECRETS = "default"
NO_SECRETS_FILE = "no_secrets_file"  # an empty LittleFS image
NO_FS_IMAGE = "no_fs_image"  # no image: the fs partition stays erased
DEVICE_UID_RE = re.compile(r"^Device UID: (io-wipper-\S+)$")
# How long checkin's runNetFSM() may take to connect to WiFi and the
# broker, as in test_mqtt.py, and how long after connecting the DUT may
# take to send its checkin.Request, and to finish the checkin once the
# broker answers.
CHECKIN_WIFI_TIMEOUT = 150
CHECKIN_MQTT_TIMEOUT = 120
CHECKIN_TIMEOUT = 30

log = logging.getLogger("hil.firmware")


def pytest_addoption(parser):
    group = parser.getgroup("hil")
    group.addoption("--boards", type=Path, default=DEFAULT_BOARDS,
                    help=f"board file (default: {DEFAULT_BOARDS.name})")
    group.addoption("--board", help="board name, if the board file lists several")
    group.addoption("--firmware-dir", type=Path,
                    help="dir with *.combined.bin and *.partitions.bin "
                         "(default: the board entry's firmware_dir)")
    group.addoption("--uhubctl", help="path to uhubctl, used if not found on PATH")
    group.addoption("--sudo", action="store_true", help="run uhubctl under 'sudo -n'")
    group.addoption("--serial-log-dir", type=Path,
                    help="write each test's serial and ProtoMQ logs here "
                         "(default: tmp_path)")
    group.addoption("--suite", choices=SUITES,
                    help="run only the board entry's api_tests or system_tests "
                         "(default: both)")


def pytest_configure(config):
    config.addinivalue_line(
        "markers", "slow: takes minutes, e.g. waits for the WDT to reboot "
                   "the board (deselect with -m \"not slow\")")


def _log_stem(request):
    """The test's log file name, without extension. A flaky test's reruns
    (pytest-rerunfailures) get a .rerunN suffix so they don't overwrite
    the failed attempt's logs."""
    reruns = getattr(request.node, "execution_count", 1) - 1
    return request.node.name + (f".rerun{reruns}" if reruns > 0 else "")


# --- Board and firmware ---------------------------------------------------


@dataclass(frozen=True)
class Board:
    name: str
    entry: dict
    chip: str
    port: str
    flash_baud: int  # esptool baud rate for flashing
    read_baud: int   # SerialMonitor baud rate for the firmware's output


@dataclass(frozen=True)
class Firmware:
    combined: Path
    partitions: Path


def load_board_entry(path, board=None):
    """Read a board entry from path, which holds one entry or a list of them."""
    try:
        data = json.loads(Path(path).read_text())
    except (OSError, json.JSONDecodeError) as e:
        pytest.fail(f"cannot read board file {path}: {e}", pytrace=False)
    entries = data if isinstance(data, list) else [data]
    if board is None:
        if len(entries) != 1:
            pytest.fail(f"{path} has {len(entries)} boards; pick one with --board",
                        pytrace=False)
        return entries[0]
    matches = [e for e in entries if isinstance(e, dict) and e.get("board") == board]
    if not matches:
        pytest.fail(f"board '{board}' not found in {path}", pytrace=False)
    return matches[0]


def _test_selectors(entry, key):
    """The board entry's api_tests/system_tests/skip_tests list, with trailing '/'
    stripped, or None if it isn't set."""
    selectors = entry.get(key)
    if selectors is None:
        return None
    if (not isinstance(selectors, list)
            or not all(isinstance(s, str) and s.strip("/") for s in selectors)):
        raise pytest.UsageError(
            f"board '{entry.get('board')}': '{key}' must be a list of test paths, "
            f"e.g. [\"provisioning/\"], got {selectors!r}")
    return [s.rstrip("/") for s in selectors]


def _matches(test_id, selectors):
    """Whether test_id (relative to FIRMWARE_DIR) is in, or is, one of
    selectors: a folder, a file, or a test with or without its params."""
    return any(test_id == s or test_id.startswith((s + "/", s + "::", s + "["))
               for s in selectors)


def _run_selectors(entry, suite):
    """The selectors for suite ("api" or "system"), or for both if suite is
    None; None (run everything) if the entry sets neither list."""
    if "run_tests" in entry:
        raise pytest.UsageError(
            f"board '{entry.get('board')}': 'run_tests' is replaced by "
            "'api_tests' and 'system_tests'")
    lists = {s: _test_selectors(entry, f"{s}_tests") for s in SUITES}
    if suite is not None:
        return lists[suite] or []
    if all(v is None for v in lists.values()):
        return None
    return [s for v in lists.values() for s in v or []]


def pytest_collection_modifyitems(config, items):
    """Apply the board entry's api_tests/system_tests (per --suite) and
    skip_tests to the tests under FIRMWARE_DIR."""
    try:
        entry = load_board_entry(config.getoption("boards"),
                                 config.getoption("board"))
    except pytest.fail.Exception as e:
        raise pytest.UsageError(str(e)) from None
    run = _run_selectors(entry, config.getoption("suite"))
    api = _test_selectors(entry, "api_tests") or []
    skip = _test_selectors(entry, "skip_tests")
    selected, deselected = [], []
    for item in items:
        path = Path(item.path)
        if not path.is_relative_to(FIRMWARE_DIR):
            selected.append(item)
            continue
        test_id = path.relative_to(FIRMWARE_DIR).as_posix()
        if "::" in item.nodeid:
            test_id += "::" + item.nodeid.split("::", 1)[1]
        if run is not None and not _matches(test_id, run):
            deselected.append(item)
            continue
        item.stash[SHARED_FLASH] = _matches(test_id, api)
        if skip and _matches(test_id, skip):
            item.add_marker(pytest.mark.skip(
                reason=f"in skip_tests for board '{entry['board']}'"))
        selected.append(item)
    if deselected:
        config.hook.pytest_deselected(items=deselected)
    # The api tests first, so their shared flash happens once, up front.
    items[:] = sorted(selected, key=lambda i: not i.stash.get(SHARED_FLASH, False))


@pytest.fixture(scope="session")
def board(pytestconfig):
    entry = load_board_entry(pytestconfig.getoption("boards"),
                             pytestconfig.getoption("board"))
    try:
        hil_board_utils.validate_board_entry(entry)
        chip = entry.get("chip")
        if not isinstance(chip, str):
            raise hil_board_utils.PortError(
                f"board '{entry['board']}': 'chip' must be set, e.g. \"esp32\"")
        hil_board_utils.esp_bootloader_offset(chip)  # rejects unknown chips
        bauds = {key: _require_baud(entry, key) for key in ("flash_baud", "read_baud")}
        port = hil_board_utils.find_port(entry, refresh=True)
    except hil_board_utils.HilBoardUtilsError as e:
        pytest.fail(str(e), pytrace=False)
    return Board(entry["board"], entry, chip, port, **bauds)


def _require_baud(entry, key):
    baud = entry.get(key)
    if isinstance(baud, bool) or not isinstance(baud, int) or baud <= 0:
        raise hil_board_utils.PortError(
            f"board '{entry['board']}': '{key}' must be a positive integer, "
            f"got {baud!r}")
    return baud


@pytest.fixture(scope="session")
def power(pytestconfig):
    """uhubctl/sudo keyword arguments for port_reset() and the power functions."""
    return {"uhubctl": pytestconfig.getoption("uhubctl"),
            "sudo": pytestconfig.getoption("sudo")}


def _find_one(directory, pattern):
    matches = sorted(directory.glob(pattern))
    if len(matches) != 1:
        pytest.fail(f"expected one {pattern} in {directory}, found {len(matches)}",
                    pytrace=False)
    return matches[0]


@pytest.fixture(scope="session")
def firmware(pytestconfig, board):
    directory = pytestconfig.getoption("firmware_dir") or board.entry.get("firmware_dir")
    if not directory:
        pytest.fail("no firmware dir: pass --firmware-dir or set firmware_dir "
                    "in the board entry", pytrace=False)
    directory = Path(directory)
    return Firmware(_find_one(directory, "*.combined.bin"),
                    _find_one(directory, "*.partitions.bin"))


@pytest.fixture(scope="session")
def board_definition(board):
    """The board's definition.json, boards/<id>/definition.json in the
    Wippersnapper_Boards checkout at $HIL_WS_BOARDS_DIR, where <id> is the
    board entry's wippersnapper_boards_id."""
    boards_dir = _env("HIL_WS_BOARDS_DIR", required=False) or DEFAULT_WS_BOARDS_DIR
    try:
        definition = hil_board_utils.load_board_definition(
            boards_dir, board.entry.get("wippersnapper_boards_id"))
    except hil_board_utils.BoardDefinitionError as e:
        pytest.fail(f"board '{board.name}': {e} (set wippersnapper_boards_id "
                    "in the board entry to the firmware's BOARD_ID)",
                    pytrace=False)
    log.info("board definition: %s at Wippersnapper_Boards %s",
             definition.path, definition.commit)
    return definition



# --- Secrets and credentials ----------------------------------------------


@dataclass(frozen=True)
class BrokerCreds:
    host: str
    port: int
    username: str
    key: str


@dataclass(frozen=True)
class NetworkCreds:
    ssid: str
    password: str


def _env(name, required=True, allow_empty=False):
    """The environment variable name, or None if it's unset and not
    required. Its value is never logged."""
    value = os.environ.get(name)
    if value is None or (not allow_empty and not value.strip()):
        if not required:
            return None
        pytest.fail(f"${name} is not set: export the valid credentials on the "
                    "HIL host (see conftest.py)", pytrace=False)
    return value


def _env_port(name, default=None):
    """The port number in the environment variable name, or default if
    it's unset."""
    port = _env(name, required=False)
    if port is None:
        return default
    if not port.isdigit() or not 1 <= int(port) <= 65535:
        pytest.fail(f"${name} must be a port number, got {port!r}", pytrace=False)
    return int(port)


@pytest.fixture(scope="session")
def broker_creds():
    """The valid IO credentials, from $HIL_IO_*, for the ProtoMQ broker
    the protomq fixture runs on this host: $HIL_PROTOMQ_HOST (default this
    host's LAN IP) and $HIL_PROTOMQ_PORT."""
    host = _env("HIL_PROTOMQ_HOST", required=False)
    if host is None:
        try:
            host = hil_protomq.lan_ip()
        except hil_protomq.ProtoMQError as e:
            pytest.fail(f"{e}: set $HIL_PROTOMQ_HOST", pytrace=False)
    return BrokerCreds(
        host=host,
        port=_env_port("HIL_PROTOMQ_PORT", hil_protomq.DEFAULT_MQTT_PORT),
        username=_env("HIL_IO_USERNAME"),
        key=_env("HIL_IO_KEY"),
    )


@pytest.fixture
def protomq(flashed_board, broker_creds, request, pytestconfig, tmp_path):
    """A ProtoMQ broker for this test, run from $HIL_PROTOMQ_DIR on
    broker_creds' port, and a client for its HTTP API, on
    $HIL_PROTOMQ_HTTP_PORT if set. It starts after
    flashed_board, so nothing from an earlier test is on it, and stops
    when the test ends; its output goes to <test>.protomq.log."""
    log_dir = Path(pytestconfig.getoption("serial_log_dir") or tmp_path)
    server = hil_protomq.ProtoMQServer(
        _env("HIL_PROTOMQ_DIR", required=False) or hil_protomq.DEFAULT_DIR,
        mqtt_port=broker_creds.port,
        http_port=_env_port("HIL_PROTOMQ_HTTP_PORT"),
        log_path=log_dir / f"{_log_stem(request)}.protomq.log")
    try:
        status = server.start()
    except hil_protomq.ProtoMQError as e:
        pytest.fail(f"cannot start ProtoMQ: {e}", pytrace=False)
    source = status.get("protobufs", {}).get("source") or {}
    log.info("protomq %s started from %s: mqtt %s:%d, api %s, protos %s",
             status.get("protomq", {}).get("gitSha"), server.directory,
             broker_creds.host, server.mqtt_port, server.url, source.get("commit"))
    yield server
    server.stop()
    log.info("protomq log: %s", server.log_path)


def _probe_config(board, role):
    """The board entry's probe for role; skips the test if it has none."""
    config = board.entry.get("probes", {}).get(role)
    if config is None:
        pytest.skip(f"board '{board.name}' has no '{role}' probe")
    return config


@pytest.fixture
def probe(board):
    """A hil_probe.Probe reading the DUT's pins through the board entry's
    digital_gpio probe; skips the test if the entry has none. All its pins
    are released before any power change or flash during the test, and
    after the test."""
    config = _probe_config(board, "digital_gpio")
    try:
        probe = hil_probe.Probe(config)
    except hil_probe.ProbeError as e:
        pytest.fail(f"board '{board.name}': {e}", pytrace=False)
    hil_board_utils.add_pre_power_hook(probe.release)
    try:
        yield probe
    finally:
        hil_board_utils.remove_pre_power_hook(probe.release)
        probe.close()


@pytest.fixture(autouse=True)
def release_dac():
    """After every test, power down all four channels of the host's
    MCP4728 DAC at 0x60, outputs at 0V, so none drives the DUT, and save
    that to its EEPROM so it powers up that way."""
    yield
    try:
        hil_probe.release_dac()
    except (hil_probe.ProbeError, OSError) as e:
        pytest.fail(f"cannot release DAC: {e}", pytrace=False)


@pytest.fixture
def onewire_pin(board):
    """The DUT pin the board entry's onewire_sensor probe says a DS18B20
    is wired to, e.g. "D33"; skips the test if the entry has none."""
    config = _probe_config(board, "onewire_sensor")
    try:
        return hil_probe.onewire_sensor_pin(config)
    except hil_probe.ProbeError as e:
        pytest.fail(f"board '{board.name}': {e}", pytrace=False)


@pytest.fixture
def pulse_probe(board):
    """A hil_probe.PulseInProbe timing the DUT's PWM outputs through the
    board entry's pwm_input probe; skips the test if the entry has none."""
    config = _probe_config(board, "pwm_input")
    try:
        probe = hil_probe.PulseInProbe(config)
    except hil_probe.ProbeError as e:
        pytest.fail(f"board '{board.name}': {e}", pytrace=False)
    try:
        yield probe
    finally:
        probe.close()


@pytest.fixture
def dac_probe(board):
    """A hil_probe.DacProbe driving the DUT's analog inputs through the
    board entry's analog_input probe; skips the test if the entry has
    none. All four channels are powered down before any power change or
    flash during the test, and after the test."""
    config = _probe_config(board, "analog_input")
    try:
        probe = hil_probe.DacProbe(config)
    except hil_probe.ProbeError as e:
        pytest.fail(f"board '{board.name}': {e}", pytrace=False)
    hil_board_utils.add_pre_power_hook(probe.release)
    try:
        yield probe
    finally:
        hil_board_utils.remove_pre_power_hook(probe.release)
        probe.close()


@pytest.fixture(scope="session")
def network_creds():
    """The valid WiFi credentials, from $HIL_WIFI_*."""
    return NetworkCreds(ssid=_env("HIL_WIFI_SSID"),
                        password=_env("HIL_WIFI_PASSWORD", allow_empty=True))


@pytest.fixture(scope="session")
def default_secrets():
    """The placeholder secrets the firmware rejects, as in
    examples/secrets-examples/secrets-wifi.json."""
    return {
        "io_username": "YOUR_IO_USERNAME_HERE",
        "io_key": "YOUR_IO_KEY_HERE",
        "network_type_wifi": {
            "network_ssid": "YOUR_WIFI_SSID_HERE",
            "network_password": "YOUR_WIFI_PASS_HERE",
        },
    }


def _with_io(secrets, creds):
    """A copy of secrets with the IO credentials in creds."""
    secrets = copy.deepcopy(secrets)
    secrets.update(io_username=creds.username, io_key=creds.key,
                   io_url=creds.host, io_port=creds.port)
    return secrets


# Credentials that pass the firmware's placeholder checks but are valid
# nowhere, for scenarios that must fail later, e.g. at the WiFi scan.
FAKE_IO_CREDS = BrokerCreds(host="io.adafruit.com", port=8883,
                            username="hil_fake_user", key="hil_fake_key")
MISSING_SSID = "hil-no-such-network"
WRONG_WIFI_PASSWORD = "invalidpassword"
# A broker host that never resolves (RFC 2606 reserves .invalid), and a
# port nothing listens on at the broker's host.
MISSING_BROKER_HOST = "hil-no-such-broker.invalid"
CLOSED_BROKER_PORT = 1


def _with_wifi(secrets, **changes):
    """A copy of secrets with network_type_wifi updated with changes."""
    secrets = copy.deepcopy(secrets)
    secrets["network_type_wifi"].update(changes)
    return secrets


@pytest.fixture(scope="session")
def generated_secrets(default_secrets):
    """Scenarios built here instead of read from the secrets dir, by name.
    The structure scenarios keep the placeholder credentials: the firmware
    checks structure first, so it must halt before it gets to them. The
    network scenarios use FAKE_IO_CREDS, so they need no $HIL_* variables."""
    no_wifi = {k: v for k, v in default_secrets.items() if k != "network_type_wifi"}
    return {
        DEFAULT_SECRETS: default_secrets,
        "no_network_type_wifi": no_wifi,
        "empty_alternative_networks":
            _with_wifi(default_secrets, alternative_networks=[]),
        "string_alternative_networks":
            _with_wifi(default_secrets, alternative_networks="YOUR_WIFI_SSID_HERE"),
        "object_alternative_networks":
            _with_wifi(default_secrets, alternative_networks={
                "network_ssid": "YOUR_WIFI_SSID_HERE",
                "network_password": "YOUR_WIFI_PASS_HERE",
            }),
        "ssid_not_found":
            _with_wifi(_with_io(default_secrets, FAKE_IO_CREDS),
                       network_ssid=MISSING_SSID,
                       network_password="hil_fake_password"),
    }


@pytest.fixture(scope="session")
def valid_secrets(default_secrets, broker_creds, network_creds):
    """Secrets with valid WiFi and IO credentials: the "good" scenario."""
    return _with_wifi(_with_io(default_secrets, broker_creds),
                      network_ssid=network_creds.ssid,
                      network_password=network_creds.password)


@pytest.fixture(scope="session")
def valid_io_secrets(default_secrets, broker_creds):
    """Secrets with valid IO credentials and placeholder WiFi."""
    return _with_io(default_secrets, broker_creds)


@pytest.fixture(scope="session")
def valid_wifi_secrets(default_secrets, network_creds):
    """Secrets with valid WiFi credentials and placeholder IO."""
    return _with_wifi(default_secrets, network_ssid=network_creds.ssid,
                      network_password=network_creds.password)


@pytest.fixture(scope="session")
def ssid_valid_secrets(default_secrets, network_creds):
    """Secrets with valid WiFi credentials and FAKE_IO_CREDS, which pass
    provisioning so the firmware gets to the WiFi scan."""
    return _with_wifi(_with_io(default_secrets, FAKE_IO_CREDS),
                      network_ssid=network_creds.ssid,
                      network_password=network_creds.password)


@pytest.fixture(scope="session")
def invalid_password_secrets(default_secrets, network_creds):
    """Secrets naming the HIL WiFi network with a wrong password, and
    FAKE_IO_CREDS, so the scan finds the SSID but the connect fails."""
    return _with_wifi(_with_io(default_secrets, FAKE_IO_CREDS),
                      network_ssid=network_creds.ssid,
                      network_password=WRONG_WIFI_PASSWORD)


@pytest.fixture(scope="session")
def invalid_broker_url_secrets(valid_secrets):
    """The good secrets, with io_url set to MISSING_BROKER_HOST."""
    return {**valid_secrets, "io_url": MISSING_BROKER_HOST}


@pytest.fixture(scope="session")
def invalid_broker_port_secrets(valid_secrets):
    """The good secrets, with io_port set to CLOSED_BROKER_PORT."""
    return {**valid_secrets, "io_port": CLOSED_BROKER_PORT}


# The scenarios with valid credentials, and their fixtures. They're looked
# up only when a test uses them, so the other scenarios run without the
# $HIL_* variables.
VALID_SCENARIOS = {
    GOOD_SECRETS: "valid_secrets",
    VALID_IO_ONLY: "valid_io_secrets",
    VALID_WIFI_ONLY: "valid_wifi_secrets",
    SSID_VALID: "ssid_valid_secrets",
    INVALID_PASSWORD: "invalid_password_secrets",
    INVALID_BROKER_URL: "invalid_broker_url_secrets",
    INVALID_BROKER_PORT: "invalid_broker_port_secrets",
}


# --- Flashing -------------------------------------------------------------


@pytest.fixture
def secrets_name(request):
    """The secrets scenario the test boots with: the test's indirect
    parameter, else the good secrets."""
    return getattr(request, "param", GOOD_SECRETS)


@pytest.fixture
def secrets_path(request, secrets_name, tmp_path):
    """secrets_name's secrets, written to a temp file: None for
    no_secrets_file and no_fs_image."""
    if secrets_name in (NO_SECRETS_FILE, NO_FS_IMAGE):
        return None
    if secrets_name in VALID_SCENARIOS:
        secrets = request.getfixturevalue(VALID_SCENARIOS[secrets_name])
    else:
        generated = request.getfixturevalue("generated_secrets")
        if secrets_name not in generated:
            pytest.fail(f"unknown secrets scenario '{secrets_name}'", pytrace=False)
        secrets = generated[secrets_name]
    path = tmp_path / f"{secrets_name}{SECRETS_SUFFIX}"
    path.write_text(json.dumps(secrets, indent=4))
    return path


@dataclass(frozen=True)
class FlashState:
    """What flashed_board did to the board before the test."""
    secrets_path: Path  # None if no secrets file was packed
    erase: hil_board_utils.EraseResult
    build: hil_board_utils.BuildResult  # None if no fs image was flashed
    flash: hil_board_utils.FlashResult
    erase_s: float  # seconds to erase the chip
    build_s: float  # seconds to build the LittleFS image
    flash_s: float  # seconds to flash the firmware and image


# Whether a test is in the board entry's api_tests, which share one flash.
SHARED_FLASH = pytest.StashKey[bool]()
# The api tests' flash: ((firmware, scenario, secrets text), FlashState),
# or None once anything else flashed the board.
_shared_flash = None


@pytest.fixture
def flashed_board(request, board, firmware, secrets_name, secrets_path, tmp_path):
    """Erase the whole chip, then flash the firmware and a LittleFS image
    holding secrets_path. The file is packed as-is, valid or not. For
    no_fs_image only the firmware is flashed. An api test reuses the
    board's flash if an earlier api test flashed the same secrets."""
    global _shared_flash
    shared = request.node.stash.get(SHARED_FLASH, False)
    key = (firmware.combined, secrets_name,
           secrets_path.read_text() if secrets_path else None)
    if shared and _shared_flash is not None and _shared_flash[0] == key:
        log.info("reusing the api tests' flash on %s with scenario %s",
                 board.port, secrets_name)
        return _shared_flash[1]
    _shared_flash = None
    state = _flash_board(board, firmware, secrets_name, secrets_path, tmp_path)
    if shared:
        _shared_flash = (key, state)
    return state


def _current_port(board):
    """The board's tty now. A power cycle can re-enumerate it under another
    name (e.g. /dev/ttyACM2 -> /dev/ttyACM0) partway through a session, so
    board.port, resolved at session start, may be stale."""
    try:
        return hil_board_utils.wait_for_port(board.entry)
    except hil_board_utils.HilBoardUtilsError as e:
        pytest.fail(str(e), pytrace=False)


def _flash_board(board, firmware, secrets_name, secrets_path, tmp_path):
    image = None if secrets_name == NO_FS_IMAGE else tmp_path / "littlefs.bin"
    build = None
    port = _current_port(board)
    try:
        start = time.perf_counter()
        erase = hil_board_utils.esp_erase_flash(port, board.chip,
                                                baud=board.flash_baud)
        erased = time.perf_counter()
        if image is not None:
            build = hil_board_utils.esp_make_littlefs(firmware.partitions, image,
                                                      secrets_path)
        built = time.perf_counter()
        flash = hil_board_utils.esp_flash(port, board.chip, firmware.partitions,
                                          firmware.combined, image,
                                          baud=board.flash_baud)
        flashed = time.perf_counter()
    except hil_board_utils.HilBoardUtilsError as e:
        pytest.fail(str(e), pytrace=False)
    log.info("erased %s, flashed %s with scenario %s: erase %.2fs, build %.2fs, "
             "flash %.2fs", port, firmware.combined.name, secrets_name,
             erased - start, built - erased, flashed - built)
    return FlashState(secrets_path, erase, build, flash, erase_s=erased - start,
                      build_s=built - erased, flash_s=flashed - built)


# --- Serial ---------------------------------------------------------------


@pytest.fixture
def serial(board, flashed_board, request, pytestconfig, tmp_path):
    """A SerialMonitor on the board's port. It doesn't reset the board; the
    test does (board_reset() or port_reset()), as its Act step. The capture
    is dumped when the test ends."""
    mon = hil_board_utils.SerialMonitor(_current_port(board), board.read_baud)
    yield mon
    mon.close()
    log_dir = pytestconfig.getoption("serial_log_dir") or tmp_path
    path = mon.dump(Path(log_dir) / f"{_log_stem(request)}.serial.log")
    log.info("serial log: %s", path)


@pytest.fixture
def device_uid(serial):
    """device_uid(timeout) -> the Device UID the firmware prints at the start
    of connect(), io-wipper-<uid>, the client ID it uses on the broker.
    Call it after the test's Act step, and after expecting any earlier boot
    lines: it moves the serial cursor past the UID line. Scenarios that halt in provision() never
    print it."""
    def read(timeout=30):
        uid = serial.expect(DEVICE_UID_RE, timeout=timeout).group(1)
        log.info("device uid: %s", uid)
        return uid
    return read


@pytest.fixture
def checkin(board, board_definition, serial, power, device_uid, protomq):
    """checkin() resets the DUT and sees it through the checkin to its app
    loop, the broker answering R_OK for its board definition. Returns its
    UID. Call it in the test's Arrange step, after anything that must
    happen before the reset."""
    def run():
        # Create the expected checkin.Response based on the board definition
        definition = board_definition.definition
        response = {
            "response": "R_OK",
            "totalGpioPins": len(definition["components"]["digitalPins"]),
            "totalAnalogPins": len(definition["components"]["analogPins"]),
            "referenceVoltage": definition["mcuRefVoltage"],
        }
        protomq.enable_checkin_fallback(False)
        protomq.set_autorespond(
            "checkin.request", {"checkin": {"response": response}},
            match={"hardwareUid": board_definition.board_id})

        serial.port_reset(board.entry, timeout=15, confirm_off=True, **power)

        # Capture the DUT's UID and tell the broker to track its messages.
        # Not asserting that it isn't on the broker yet: if the DUT
        # reconnected to this test's broker before the port reset cut its
        # power, the broker keeps that dead session until the DUT
        # reconnects and takes it over. The checkin.Request below shows the
        # connection is fresh. If this race causes trouble again, power the
        # DUT off before ProtoMQ starts.
        uid = device_uid(timeout=60)
        protomq.track(uid)

        # Ensure runNetFSM() connects to WiFi and the broker
        serial.expect(r"^Connected to WiFi!$", timeout=CHECKIN_WIFI_TIMEOUT)
        serial.expect(r"^Connected to Adafruit IO$",
                      timeout=CHECKIN_MQTT_TIMEOUT)
        protomq.expect_client(uid, timeout=5)

        # DUT checks in with the broker
        protomq.expect_d2b_message(
            uid, "checkin.request",
            match={"hardwareUid": board_definition.board_id},
            timeout=CHECKIN_TIMEOUT)
        protomq.expect_b2d_message(uid, "checkin.response",
                                   match=response, timeout=5)
        protomq.expect_d2b_message(uid, "checkin.complete",
                                   timeout=CHECKIN_TIMEOUT)
        serial.expect(r"^Completed checkin process!$",
                      timeout=CHECKIN_TIMEOUT)
        serial.expect(r"^Running app loop\.\.\.$", timeout=CHECKIN_TIMEOUT)
        return uid
    return run
