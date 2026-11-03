"""
HIL tests for Adafruit Wippersnapper firmware's digital component's input handling.

Exercises:
- `src/components/digitalIO/controller.cpp::Handle_DigitalIO_Add()`
- `src/components/digitalIO/controller.cpp::update()`
- `src/components/digitalIO/controller.cpp::EncodePublishPinEvent()`
- `src/components/digitalIO/controller.cpp::Handle_DigitalIO_Remove()`
- `src/components/digitalIO/controller.cpp::Router()` (digital.Modify)
- `src/components/digitalIO/hardware.cpp::CheckEvent()`
- `src/components/digitalIO/hardware.cpp::CheckTimer()`

Each test leaves the DUT powered off, so it doesn't keep sampling (and
trying to publish to a stopped broker) into the next test. The next test
powers it back on before its serial monitor opens, and the DUT is powered
on again once this module's tests are done, for the tests after them.

TODO: test D_INPUT with a pull-down resistor (high-impedance probe reads 0,
probe driving HIGH gives 1) once the firmware and digital.proto support
pull-down inputs; today digital.Direction only has D_INPUT_PULL_UP.
"""

import re
import time

import pytest

import hil_board_utils
from dio_helpers import CHECKIN_TIMEOUT, remove

# digital.Direction values, as Handle_DigitalIO_Add() prints them
DIRECTIONS = {"D_INPUT": 1, "D_INPUT_PULL_UP": 2}
# How long the DUT may take to publish a pin's first value after adding it
FIRST_EVENT_TIMEOUT = 5
# How long the pin is left at its starting level after the first event, for
# the DUT to settle; no events may come in meanwhile
SETTLE_TIME = 0.5
# How long the DUT may take to publish an event after the probe drives an
# edge, and the window that event must be the only one in
EVENT_TIMEOUT = 1
# SM_TIMER: the Add.period the timer tests set, how long the reporting test
# samples for, and the shorter period the Modify sets and how many of its
# publishes are checked (seconds, except MODIFY_SAMPLES)
TIMER_PERIOD = 10
TIMER_DURATION = 90
MODIFY_PERIOD = 5
MODIFY_SAMPLES = 10
# How far each gap between timer publishes may be from the period: as the
# DUT logs them (serial), and as the broker receives them (adds WiFi jitter)
TIMER_TOLERANCE = 0.35
DELIVERY_TOLERANCE = 0.35
# After a remove, how many times the probe toggles the pin and over how long
# (seconds); the DUT must publish nothing for it
REMOVED_TOGGLES = 20
REMOVED_WINDOW = 2


def _power_on(board, power):
    """Power the DUT on, if it isn't, and wait for its serial port."""
    loc = board.entry["location"]
    hil_board_utils._power_on(loc["hub"], loc["port"], **power)
    port = str(hil_board_utils.wait_for_port(board.entry, timeout=15))
    if port != board.port:
        pytest.fail(f"board '{board.name}' came back on {port}, not "
                    f"{board.port}", pytrace=False)


@pytest.fixture(scope="module", autouse=True)
def powered_after_module(board, power):
    """Leave the DUT powered on for the tests after this module's."""
    yield
    _power_on(board, power)


@pytest.fixture(autouse=True)
def powered_between_tests(board, power, powered_after_module):
    """Power the DUT on before the test (a previous test left it off) and
    off after it. Autouse, so it's set up before the serial monitor opens
    the port and torn down after it closes."""
    _power_on(board, power)
    loc = board.entry["location"]
    yield
    hil_board_utils._power_off(loc["hub"], loc["port"], **power)
    hil_board_utils.wait_for_port_gone(board.entry, timeout=10)
    hil_board_utils.clear_port(board.name)


def _event_re(pin, value=None):
    """Matches EncodePublishPinEvent()'s log line for pin, with value if
    given."""
    value = r"[01]" if value is None else int(value)
    return re.compile(rf"^\[digital\] Publish Event: {pin} \| value: {value}$")


def _serial_events(serial, pin, since):
    """The SerialLines from since on that log an event for pin."""
    regex = _event_re(pin)
    return [l for l in serial.lines(since) if regex.search(l.text)]


def _input_add(pin, direction, sample_mode, period):
    """A digital.Add for pin as an input, with period if given."""
    add = {"pin": pin, "gpioDirection": direction, "sampleMode": sample_mode}
    if period is not None:
        add["period"] = period
    return add


def _expect_added(serial, pin, direction):
    """Expect Handle_DigitalIO_Add() to log adding pin as direction."""
    serial.expect(r"^\[digital\] Added new pin:$", timeout=CHECKIN_TIMEOUT)
    serial.expect(rf"^Pin Name: {pin.lstrip('D')}$", timeout=5)
    serial.expect(rf"^Direction: {DIRECTIONS[direction]}$", timeout=5)


def _add_input(serial, protomq, uid, pin, direction="D_INPUT",
               sample_mode="SM_EVENT", period=None):
    """Send a digital.Add for pin as an input with sample_mode (and period,
    in seconds, if given) and expect the DUT to add it."""
    add = _input_add(pin, direction, sample_mode, period)
    protomq.publish(uid, {"digital": {"add": add}})
    protomq.expect_b2d_message(uid, "digital.add", match=add, timeout=5)
    _expect_added(serial, pin, direction)


def _arrange_input(checkin, probe, pin, level):
    """Check the DUT in with the probe not driving pin, then drive pin to
    level, or leave it high-impedance if level is None. Returns the UID."""
    probe.release(pin)
    uid = checkin()
    if level is not None:
        probe.drive(pin, level)
    return uid


def _expect_event(serial, protomq, uid, pin, value, timeout):
    """Expect the DUT to log and publish an event of value for pin, and
    that the next digital.Event the broker gets is that one."""
    serial.expect(_event_re(pin, value), timeout=timeout)
    serial.expect(r"^\[digital\] Published!$", timeout=timeout)

    # No match: the next event of any pin or value, so a stray one fails
    event = protomq.expect_d2b_message(uid, "digital.event", timeout=timeout)
    assert event.get("pin") == pin, f"expected an event for {pin}: {event}"
    sensor_event = event.get("value", {})
    assert sensor_event.get("type") == "T_BOOLEAN", (
        f"expected a T_BOOLEAN sensor event: {event}")
    # proto3 JSON leaves out a false bool
    assert sensor_event.get("boolValue", False) == value, (
        f"expected value {int(value)}: {event}")


def _edge_test(checkin, serial, protomq, probe, pin, start, end,
               direction="D_INPUT", first=None):
    """Check the DUT in, drive pin to start, add it as a direction SM_EVENT
    input, then drive it to end and expect exactly one event of end.

    start None leaves the probe high-impedance before the edge, so the DUT
    reads what its pull resistor gives; first is then the value it must
    report first. Otherwise first is start."""
    if first is None:
        first = start
    try:
        # --- Arrange --------------------------------------------------
        # The probe doesn't drive pin through the power cut and checkin,
        # then holds it at start (or leaves it high-impedance) and the
        # broker adds it. CheckEvent() publishes the first value it reads
        uid = _arrange_input(checkin, probe, pin, start)
        added = serial.mark()
        _add_input(serial, protomq, uid, pin, direction, "SM_EVENT")
        _expect_event(serial, protomq, uid, pin, first,
                      timeout=FIRST_EVENT_TIMEOUT)

        # Nothing else is published while pin stays at start
        time.sleep(SETTLE_TIME)
        events = _serial_events(serial, pin, added)
        assert len(events) == 1, (
            f"expected only the first event for {pin} before driving it: "
            f"{[l.text for l in events]}")
        protomq.assert_no_d2b_message(uid, "digital.event", within=0.1)

        # --- Act ------------------------------------------------------
        driven = serial.mark()
        t0 = time.monotonic()
        probe.drive(pin, end)

        # --- Assert ---------------------------------------------------
        # One event of end within EVENT_TIMEOUT, and no others
        _expect_event(serial, protomq, uid, pin, end, timeout=EVENT_TIMEOUT)
        time.sleep(max(EVENT_TIMEOUT - (time.monotonic() - t0), 0))
        events = _serial_events(serial, pin, driven)
        assert len(events) == 1, (
            f"expected exactly one event for {pin} after driving it "
            f"{int(first)} -> {int(end)}: {[l.text for l in events]}")
        protomq.assert_no_d2b_message(uid, "digital.event", within=0.1)
    finally:
        probe.release(pin)


@pytest.mark.parametrize("secrets_name", ["good"], indirect=True)
def test_digital_input_event_rising_edge(board, board_definition, secrets_name,
                                         flashed_board, serial, power,
                                         device_uid, protomq, probe, checkin):
    _edge_test(checkin, serial, protomq, probe, "D32", start=False, end=True)


@pytest.mark.parametrize("secrets_name", ["good"], indirect=True)
def test_digital_input_event_falling_edge(board, board_definition,
                                          secrets_name, flashed_board, serial,
                                          power, device_uid, protomq, probe,
                                          checkin):
    _edge_test(checkin, serial, protomq, probe, "D32", start=True, end=False)


@pytest.mark.parametrize("secrets_name", ["good"], indirect=True)
def test_digital_input_pull_up(board, board_definition, secrets_name,
                               flashed_board, serial, power, device_uid,
                               protomq, probe, checkin):
    # The probe stays high-impedance, so D32's pull-up makes it read 1,
    # until the probe drives it LOW, which must read 0
    _edge_test(checkin, serial, protomq, probe, "D32", start=None, end=False,
               direction="D_INPUT_PULL_UP", first=True)


def _assert_silent_after_remove(serial, protomq, uid, probe, pin, level,
                                window):
    """Toggle pin REMOVED_TOGGLES times over window seconds, starting from
    level, then expect no event for pin on serial and no digital.event at
    the broker (whose cursor is on the last expected event) since the
    first toggle, allowing SETTLE_TIME for a late one."""
    since = serial.mark()
    for _ in range(REMOVED_TOGGLES):
        level = not level
        probe.drive(pin, level)
        time.sleep(window / REMOVED_TOGGLES)
    serial.assert_none(_event_re(pin), since=since, duration=SETTLE_TIME)
    protomq.assert_no_d2b_message(uid, "digital.event", within=0.1)


@pytest.mark.parametrize("secrets_name", ["good"], indirect=True)
def test_digital_input_event_remove(board, board_definition, secrets_name,
                                    flashed_board, serial, power, device_uid,
                                    protomq, probe, checkin):
    try:
        # --- Arrange --------------------------------------------------
        # D32 is an SM_EVENT input: its first event, then a rising edge
        uid = _arrange_input(checkin, probe, "D32", False)
        _add_input(serial, protomq, uid, "D32", "D_INPUT", "SM_EVENT")
        _expect_event(serial, protomq, uid, "D32", False,
                      timeout=FIRST_EVENT_TIMEOUT)
        probe.drive("D32", True)
        _expect_event(serial, protomq, uid, "D32", True, timeout=EVENT_TIMEOUT)

        # --- Act ------------------------------------------------------
        remove(serial, protomq, uid, "D32")

        # --- Assert ---------------------------------------------------
        # The probe toggles D32 over and over, and none of it is published
        _assert_silent_after_remove(serial, protomq, uid, probe, "D32", True,
                                    REMOVED_WINDOW)
    finally:
        probe.release("D32")


def _sample_timer(serial, protomq, uid, probe, pin, level, period, count):
    """Expect count SM_TIMER publishes of pin, each of the level the probe
    drives, flipping that level halfway between publishes so the values
    must follow it. Returns the last level driven and when the broker
    received each publish."""
    received = []
    for i in range(count):
        _expect_event(serial, protomq, uid, pin, level,
                      timeout=period + TIMER_TOLERANCE + 1)
        received.append(protomq.last_d2b_time(uid))
        if i < count - 1:
            time.sleep(period / 2)
            level = not level
            probe.drive(pin, level)
    assert None not in received, (
        "ProtoMQ doesn't timestamp deliveries ('time' in /dump_deliveries); "
        "update its checkout")
    return level, received


def _assert_gaps(times, period, tolerance, what):
    """Every gap between consecutive times is period +/- tolerance."""
    gaps = [b - a for a, b in zip(times, times[1:])]
    shown = ", ".join(f"{g:.3f}" for g in gaps)
    assert all(abs(g - period) <= tolerance for g in gaps), (
        f"{what} gaps should be {period}s +/- {tolerance}s: [{shown}]")


def _assert_timer(serial, pin, since, received, period):
    """Assert the publishes since since are the len(received) expected
    ones, a period apart on serial and at the broker."""
    lines = _serial_events(serial, pin, since)
    assert len(lines) == len(received), (
        f"expected {len(received)} timer publishes for {pin}: "
        f"{[l.text for l in lines]}")
    _assert_gaps([l.time for l in lines], period, TIMER_TOLERANCE,
                 "serial publish")
    _assert_gaps(received, period, DELIVERY_TOLERANCE, "broker delivery")


@pytest.mark.parametrize("secrets_name", ["good"], indirect=True)
def test_digital_input_timer_reporting(board, board_definition, secrets_name,
                                       flashed_board, serial, power,
                                       device_uid, protomq, probe, checkin):
    try:
        # --- Arrange --------------------------------------------------
        # The probe holds D32 low once the DUT is checked in
        uid = _arrange_input(checkin, probe, "D32", False)

        # --- Act ------------------------------------------------------
        # The broker adds D32 as a timer input. CheckTimer() publishes at
        # once, then every TIMER_PERIOD, while the probe flips D32 between
        # publishes
        added = serial.mark()
        _add_input(serial, protomq, uid, "D32", "D_INPUT", "SM_TIMER",
                   period=TIMER_PERIOD)
        _, received = _sample_timer(serial, protomq, uid, probe, "D32", False,
                                    TIMER_PERIOD,
                                    TIMER_DURATION // TIMER_PERIOD + 1)

        # --- Assert ---------------------------------------------------
        _assert_timer(serial, "D32", added, received, TIMER_PERIOD)
    finally:
        probe.release("D32")


@pytest.mark.parametrize("secrets_name", ["good"], indirect=True)
def test_digital_input_timer_remove(board, board_definition, secrets_name,
                                    flashed_board, serial, power, device_uid,
                                    protomq, probe, checkin):
    try:
        # --- Arrange --------------------------------------------------
        # D32 is a timer input publishing every TIMER_PERIOD
        uid = _arrange_input(checkin, probe, "D32", False)
        added = serial.mark()
        _add_input(serial, protomq, uid, "D32", "D_INPUT", "SM_TIMER",
                   period=TIMER_PERIOD)
        level, received = _sample_timer(serial, protomq, uid, probe, "D32",
                                        False, TIMER_PERIOD, 3)
        _assert_timer(serial, "D32", added, received, TIMER_PERIOD)

        # --- Act ------------------------------------------------------
        remove(serial, protomq, uid, "D32")

        # --- Assert ---------------------------------------------------
        # The probe toggles D32 for a whole period after the remove, and
        # nothing is published
        _assert_silent_after_remove(serial, protomq, uid, probe, "D32", level,
                                    TIMER_PERIOD + TIMER_TOLERANCE)
    finally:
        probe.release("D32")


@pytest.mark.parametrize("secrets_name", ["good"], indirect=True)
def test_digital_input_timer_modify_period(board, board_definition,
                                           secrets_name, flashed_board, serial,
                                           power, device_uid, protomq, probe,
                                           checkin):
    try:
        # --- Arrange --------------------------------------------------
        # D32 is a timer input publishing every TIMER_PERIOD, then the
        # broker removes it
        uid = _arrange_input(checkin, probe, "D32", False)
        added = serial.mark()
        _add_input(serial, protomq, uid, "D32", "D_INPUT", "SM_TIMER",
                   period=TIMER_PERIOD)
        level, received = _sample_timer(serial, protomq, uid, probe, "D32",
                                        False, TIMER_PERIOD, 3)
        _assert_timer(serial, "D32", added, received, TIMER_PERIOD)
        remove(serial, protomq, uid, "D32")

        # --- Act ------------------------------------------------------
        # A digital.Modify re-adds D32 with the shorter MODIFY_PERIOD
        modified = serial.mark()
        add = _input_add("D32", "D_INPUT", "SM_TIMER", MODIFY_PERIOD)
        protomq.publish(uid, {"digital": {"modify": {"add": add}}})
        protomq.expect_b2d_message(uid, "digital.modify", match={"add": add},
                                   timeout=5)
        _expect_added(serial, "D32", "D_INPUT")

        # --- Assert ---------------------------------------------------
        # The new pin publishes at once, then every MODIFY_PERIOD
        _, received = _sample_timer(serial, protomq, uid, probe, "D32", level,
                                    MODIFY_PERIOD, MODIFY_SAMPLES)
        _assert_timer(serial, "D32", modified, received, MODIFY_PERIOD)
    finally:
        probe.release("D32")
