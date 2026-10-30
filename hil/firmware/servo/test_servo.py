"""
HIL tests for Adafruit Wippersnapper firmware's servo component.

Exercises:
- `src/components/servo/controller.cpp::Router()`
- `src/components/servo/controller.cpp::Handle_Servo_Add()`
- `src/components/servo/hardware.cpp::ServoAttach()` (ESP32's ledcAttach())
- `src/components/servo/hardware.cpp::ServoWrite()`
- `src/components/servo/hardware.cpp::writeMicroseconds()` (ESP32's ledcWrite())
- `src/components/servo/controller.cpp::Handle_Servo_Write()`
- `src/components/servo/controller.cpp::Handle_Servo_Remove()`
- `src/components/servo/hardware.cpp::ServoDetach()` (ESP32's ledcDetach())
"""

import logging
import re

import pytest


log = logging.getLogger("hil.servo")

# Maximum time to wait for the DUT to check in, in seconds.
CHECKIN_TIMEOUT = 30
# The broker's default servo PWM frequency, in Hz.
DEFAULT_FREQUENCY = 50
# The broker's default minimum and maximum pulse widths, in us.
DEFAULT_MIN_PULSE_WIDTH = 500
DEFAULT_MAX_PULSE_WIDTH = 2500
# Maximum time to wait for the DUT to report an error after handling a message, in seconds.
ERROR_WINDOW = 3
# Number of PWM periods the pulse probe measures over: 0.5s at 50 Hz.
SERVO_PERIODS = 25
# Maximum time for the pulse probe to capture SERVO_PERIODS periods, in seconds.
CAPTURE_TIMEOUT = 2
# Maximum acceptable error for the pulse probe's measured pulse width, in us.
PULSE_WIDTH_TOLERANCE = 7
# Period tolerance for the pulse probe's measured period.
PERIOD_TOLERANCE = 0.002
# Quiet window during which no edges should be detected.
QUIET_WINDOW = 0.5
# A LEDC frequency beyond what the servo's LEDC timer width reaches.
MAX_LEDC_FREQ = 1_000_000
# Pulse widths Adafruit IO sends as a servo position, in us.
IO_MIN_PULSE_WIDTH = 0
IO_MAX_PULSE_WIDTH = 2500
# Servo positions to sweep through, in degrees: up to full travel and back.
SWEEP_UP = (0, 25, 50, 90, 100, 125, 150, 180)
SWEEP_POSITIONS = SWEEP_UP + SWEEP_UP[-2::-1]
# Full travel of the servo, in degrees.
FULL_TRAVEL = 180
# Time to wait after a write before measuring, in seconds.
WRITE_SETTLE = 1
# Pulse width range a servo.Modify changes the servo to, in us.
MODIFY_MIN_PULSE_WIDTH = 1000
MODIFY_MAX_PULSE_WIDTH = 2000
# Lines the DUT prints when it crashes or reboots.
CRASH_RE = re.compile(r"Guru Meditation|Backtrace:|rst:0x|ESP Reset Reason")


@pytest.fixture
def servo_pin(pulse_probe):
    """The DUT's servo output pin under test: the first pin in the board
    entry's pwm_input probe's pin_map, which the pulse probe measures."""
    return next(iter(pulse_probe.pin_map))


def _add(serial, protomq, uid, pin, frequency=DEFAULT_FREQUENCY,
         min_pulse_width=DEFAULT_MIN_PULSE_WIDTH,
         max_pulse_width=DEFAULT_MAX_PULSE_WIDTH):
    """Send a servo.Add for pin at frequency and pulse width range, leaving
    the initial write empty, and expect the DUT to attach it."""
    add = {"servoPin": pin, "freq": frequency,
           "minPulseWidth": min_pulse_width,
           "maxPulseWidth": max_pulse_width}
    protomq.publish(uid, {"servo": {"add": add}})
    protomq.expect_b2d_message(uid, "servo.add", match=add, timeout=5)

    # Handle_Servo_Add() writes min_pulse_width once ServoAttach()
    # succeeds; writeMicroseconds() logs it, and the GPIO number, without a
    # newline, so it shares a line with the attach log
    gpio = int(pin[1:])
    serial.expect(rf"\[servo\] Set Pulse Width: {min_pulse_width} uS on "
                  rf"pin: {gpio}\[servo\] Servo attached to pin: {pin}",
                  timeout=CHECKIN_TIMEOUT)
    protomq.assert_no_d2b_message(uid, "error.component",
                                  match={"pin": pin}, within=ERROR_WINDOW)


def _assert_pulse(pulse_probe, pin, pulse_width, frequency):
    """Measure pin over SERVO_PERIODS periods and check it runs at
    frequency with a pulse_width us high pulse."""
    duty, period_us = pulse_probe.measure(pin, periods=SERVO_PERIODS,
                                          timeout=CAPTURE_TIMEOUT)
    measured_width = duty * period_us
    log.info("%s measured %.1fus high every %.1fus", pin, measured_width,
             period_us)

    assert abs(measured_width - pulse_width) <= PULSE_WIDTH_TOLERANCE, (
        f"{pin}: measured pulse width {measured_width:.1f}us, expected "
        f"{pulse_width}us +/- {PULSE_WIDTH_TOLERANCE}us "
        f"(period {period_us:.1f}us)")
    expected_period = 1e6 / frequency
    assert abs(period_us - expected_period) <= (
        PERIOD_TOLERANCE * expected_period), (
        f"{pin}: measured period {period_us:.1f}us, expected "
        f"{expected_period:.0f}us +/- {PERIOD_TOLERANCE:.1%}")


@pytest.mark.parametrize("secrets_name", ["good"], indirect=True)
def test_servo_add(board, board_definition, secrets_name, flashed_board,
                   serial, power, device_uid, protomq, pulse_probe,
                   servo_pin, checkin):
    """A servo.Add at the broker's defaults attaches the pin and drives it
    at the minimum pulse width."""
    # --- Arrange ------------------------------------------------------
    # DUT checks in
    uid = checkin()

    # --- Act ----------------------------------------------------------
    # The broker adds the servo pin at 50 Hz, 500-2500us
    _add(serial, protomq, uid, servo_pin)

    # --- Assert -------------------------------------------------------
    # The pin runs at 50 Hz with the minimum, 500us, pulse
    _assert_pulse(pulse_probe, servo_pin, DEFAULT_MIN_PULSE_WIDTH,
                  DEFAULT_FREQUENCY)


def _expect_add_rejected(serial, protomq, uid, pin, frequency):
    """Send a servo.Add for pin at frequency, with the default pulse width
    range, and expect the DUT to reject it: ServoAttach() fails and it
    publishes a component error."""
    add = {"servoPin": pin, "freq": frequency,
           "minPulseWidth": DEFAULT_MIN_PULSE_WIDTH,
           "maxPulseWidth": DEFAULT_MAX_PULSE_WIDTH}
    protomq.publish(uid, {"servo": {"add": add}})
    protomq.expect_b2d_message(uid, "servo.add", match=add, timeout=5)

    # ServoAttach() logs the GPIO number it failed on
    gpio = int(pin[1:])
    serial.expect(rf"\[servo\] Error: Failed to attach servo to pin: {gpio}",
                  timeout=CHECKIN_TIMEOUT)
    protomq.expect_d2b_message(
        uid, "error.component",
        match={"pin": pin, "message": "Failed to attach servo"},
        timeout=CHECKIN_TIMEOUT)


def _assert_quiet(pulse_probe, pin, level):
    """Check pin shows no edge for QUIET_WINDOW and reads level."""
    count = pulse_probe.count_pulses(pin, QUIET_WINDOW)
    assert count == 0, (
        f"{pin} should be still but had {count} pulses in {QUIET_WINDOW}s")
    assert pulse_probe.level(pin) == level, (
        f"{pin} should read {'HIGH' if level else 'LOW'}")


@pytest.mark.parametrize("secrets_name", ["good"], indirect=True)
@pytest.mark.parametrize("frequency", [0, MAX_LEDC_FREQ])
def test_servo_add_rejected(board, board_definition, secrets_name,
                            flashed_board, serial, power, device_uid,
                            protomq, pulse_probe, servo_pin, checkin,
                            frequency):
    """A servo.Add at a frequency ledcAttach() rejects, 0 or beyond what
    the servo's LEDC timer reaches, is reported as a component error, leaves
    the pin undriven, and doesn't stop a valid Add afterward."""
    # --- Arrange ------------------------------------------------------
    # DUT checks in
    uid = checkin()
    since = serial.mark()

    # --- Act ----------------------------------------------------------
    # The broker adds the servo pin at the invalid frequency
    _expect_add_rejected(serial, protomq, uid, servo_pin, frequency)

    # --- Assert -------------------------------------------------------
    # The DUT never reports the servo attached, and the pin stays undriven,
    # reading LOW with the probe's pull-down
    serial.assert_none(r"\[servo\] Servo attached to pin:", since=since,
                       duration=ERROR_WINDOW)
    _assert_quiet(pulse_probe, servo_pin, level=False)

    # The DUT still takes a valid Add at the broker's defaults
    _add(serial, protomq, uid, servo_pin)
    _assert_pulse(pulse_probe, servo_pin, DEFAULT_MIN_PULSE_WIDTH,
                  DEFAULT_FREQUENCY)

    # The DUT didn't crash or reboot throughout
    serial.assert_none(CRASH_RE, since=since)


def _pulse_width(degrees, min_pulse_width=DEFAULT_MIN_PULSE_WIDTH,
                 max_pulse_width=DEFAULT_MAX_PULSE_WIDTH):
    """The pulse width, in us, for a servo position in degrees across the
    pulse width range, filtered to what Adafruit IO sends."""
    width = round(min_pulse_width + degrees / FULL_TRAVEL
                  * (max_pulse_width - min_pulse_width))
    return max(IO_MIN_PULSE_WIDTH, min(IO_MAX_PULSE_WIDTH, width))


def _write(serial, protomq, uid, pin, pulse_width, logged=None):
    """Send a servo.Write of pulse_width to pin and expect the DUT to
    apply it, logging logged (default pulse_width, the value clamped to
    the servo's range), and report no error for it."""
    write = {"servoPin": pin, "pulseWidth": pulse_width}
    protomq.publish(uid, {"servo": {"write": write}})
    protomq.expect_b2d_message(uid, "servo.write", match=write, timeout=5)
    # writeMicroseconds() logs the clamped pulse width and GPIO number once
    # ledcWrite() returns, without a newline
    gpio = int(pin[1:])
    logged = pulse_width if logged is None else logged
    serial.expect(rf"\[servo\] Set Pulse Width: {logged} uS on pin: "
                  rf"{gpio}\b", timeout=CHECKIN_TIMEOUT, partial=True)
    # Handle_Servo_Write() publishes a component error on failure. This
    # also gives the DUT time to apply the write.
    protomq.assert_no_d2b_message(uid, "error.component", match={"pin": pin},
                                  within=WRITE_SETTLE)


# NOTE: Each position waits for the DUT to log its write, then
# WRITE_SETTLE, then measures
# SERVO_PERIODS periods. This sweep can be sped up by shortening those, or
# slowed down by adding pauses before checking for pulses, e.g. if the
# probe catches the transition period between two positions.
@pytest.mark.parametrize("secrets_name", ["good"], indirect=True)
def test_servo_write_sweep(board, board_definition, secrets_name,
                           flashed_board, serial, power, device_uid,
                           protomq, pulse_probe, servo_pin, checkin,
                           capsys):
    """Writing the servo through SWEEP_POSITIONS, up to full travel and
    back to 0, drives the pin at each position's pulse width."""
    # --- Arrange ------------------------------------------------------
    # DUT checks in and adds the servo pin at 50 Hz, 500-2500us
    uid = checkin()
    since = serial.mark()
    _add(serial, protomq, uid, servo_pin)

    # --- Act / Assert -------------------------------------------------
    # The broker writes each position as a pulse width, and the pulse
    # probe measures it
    for degrees in SWEEP_POSITIONS:
        pulse_width = _pulse_width(degrees)
        with capsys.disabled():
            print(f"\n{servo_pin} servo at {degrees} "
                  f"({degrees / FULL_TRAVEL:.0%} of travel): "
                  f"{pulse_width}us", flush=True)
        _write(serial, protomq, uid, servo_pin, pulse_width)
        _assert_pulse(pulse_probe, servo_pin, pulse_width, DEFAULT_FREQUENCY)

    # The DUT didn't crash or reboot throughout
    serial.assert_none(CRASH_RE, since=since)


def _remove(serial, protomq, uid, pin):
    """Send a servo.Remove for pin and expect the DUT to remove it."""
    remove = {"servoPin": pin}
    protomq.publish(uid, {"servo": {"remove": remove}})
    protomq.expect_b2d_message(uid, "servo.remove", match=remove, timeout=5)
    # Handle_Servo_Remove() logs the pin once the servo is detached; an
    # earlier write's log, which has no newline, may share its line
    serial.expect(rf"\[servo\] Servo removed from pin: {pin}$",
                  timeout=CHECKIN_TIMEOUT)
    protomq.assert_no_d2b_message(uid, "error.component",
                                  match={"pin": pin}, within=ERROR_WINDOW)


@pytest.mark.parametrize("secrets_name", ["good"], indirect=True)
def test_servo_remove(board, board_definition, secrets_name, flashed_board,
                      serial, power, device_uid, protomq, pulse_probe,
                      servo_pin, checkin):
    """Removing a servo stops its output and leaves the pin LOW."""
    # --- Arrange ------------------------------------------------------
    # The servo pin runs at 50 Hz, 1500us
    uid = checkin()
    since = serial.mark()
    _add(serial, protomq, uid, servo_pin)
    _write(serial, protomq, uid, servo_pin, 1500)
    _assert_pulse(pulse_probe, servo_pin, 1500, DEFAULT_FREQUENCY)

    # --- Act ----------------------------------------------------------
    _remove(serial, protomq, uid, servo_pin)

    # --- Assert -------------------------------------------------------
    # The destructor detached the servo without error, and the pin has no
    # edges afterward, reading LOW with the probe's pull-down
    serial.assert_none(r"\[servo\] (Detach Error|Error: Failed to detach)",
                       since=since)
    _assert_quiet(pulse_probe, servo_pin, level=False)

    # The DUT didn't crash or reboot throughout
    serial.assert_none(CRASH_RE, since=since)


def _modify(serial, protomq, uid, pin, min_pulse_width, max_pulse_width,
            frequency=DEFAULT_FREQUENCY):
    """Send a servo.Modify of pin to frequency and pulse width range,
    leaving the Add's initial write empty, and expect the DUT to remove
    and re-add it with no error."""
    add = {"servoPin": pin, "freq": frequency,
           "minPulseWidth": min_pulse_width,
           "maxPulseWidth": max_pulse_width}
    modify = {"remove": {"servoPin": pin}, "add": add}
    protomq.publish(uid, {"servo": {"modify": modify}})
    protomq.expect_b2d_message(uid, "servo.modify", match=modify, timeout=5)

    # The Router removes the servo, then the add writes min_pulse_width
    # and logs it, without a newline, on the attach log's line
    serial.expect(rf"\[servo\] Servo removed from pin: {pin}$",
                  timeout=CHECKIN_TIMEOUT)
    gpio = int(pin[1:])
    serial.expect(rf"\[servo\] Set Pulse Width: {min_pulse_width} uS on "
                  rf"pin: {gpio}\[servo\] Servo attached to pin: {pin}",
                  timeout=CHECKIN_TIMEOUT)
    protomq.assert_no_d2b_message(uid, "error.component",
                                  match={"pin": pin}, within=ERROR_WINDOW)


@pytest.mark.parametrize("secrets_name", ["good"], indirect=True)
def test_servo_modify(board, board_definition, secrets_name, flashed_board,
                      serial, power, device_uid, protomq, pulse_probe,
                      servo_pin, checkin):
    """A servo.Modify of the servo's pulse width range takes effect: the
    pin restarts at the new minimum, and writes clamp to the new range."""
    # --- Arrange ------------------------------------------------------
    # The servo pin runs at 50 Hz, 1500us, with the default 500-2500us
    # range
    uid = checkin()
    since = serial.mark()
    _add(serial, protomq, uid, servo_pin)
    _write(serial, protomq, uid, servo_pin, 1500)
    _assert_pulse(pulse_probe, servo_pin, 1500, DEFAULT_FREQUENCY)

    # --- Act ----------------------------------------------------------
    # The broker modifies the servo to a 1000-2000us range
    _modify(serial, protomq, uid, servo_pin, MODIFY_MIN_PULSE_WIDTH,
            MODIFY_MAX_PULSE_WIDTH)

    # --- Assert -------------------------------------------------------
    # The pin restarts at the new minimum
    _assert_pulse(pulse_probe, servo_pin, MODIFY_MIN_PULSE_WIDTH,
                  DEFAULT_FREQUENCY)

    # Writes at the old range's ends, valid before the modify, clamp to
    # the new range's
    for pulse_width, clamped in ((DEFAULT_MAX_PULSE_WIDTH,
                                  MODIFY_MAX_PULSE_WIDTH),
                                 (DEFAULT_MIN_PULSE_WIDTH,
                                  MODIFY_MIN_PULSE_WIDTH)):
        _write(serial, protomq, uid, servo_pin, pulse_width,
               logged=clamped)
        _assert_pulse(pulse_probe, servo_pin, clamped, DEFAULT_FREQUENCY)

    # The DUT didn't crash or reboot throughout
    serial.assert_none(CRASH_RE, since=since)
