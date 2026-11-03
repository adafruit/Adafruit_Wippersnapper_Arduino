"""
HIL tests for Adafruit Wippersnapper firmware's digitalIO component driving
output pins: what the DUT does with the broker's digital.B2D messages.

Exercises:
- `src/components/digitalIO/controller.cpp::Router()`
- `src/components/digitalIO/controller.cpp::Handle_DigitalIO_Add()`
- `src/components/digitalIO/controller.cpp::Handle_DigitalIO_Write()`
- `src/components/digitalIO/controller.cpp::Handle_DigitalIO_Remove()`
"""

import pytest

from dio_helpers import CHECKIN_TIMEOUT, remove

# How long the DUT's pin may take to reach a written value, read by the
# probe, from entering expect_digitalout_transition().
PIN_TRANSITION_TIMEOUT = 2


def _add_output(serial, protomq, uid, pin, value=None):
    """Send a digital.Add for pin as an output, with value as its initial
    write if given, leaving the sample mode, period and inversion empty,
    and expect the DUT to add it."""
    add = {"pin": pin, "gpioDirection": "D_OUTPUT"}
    if value is not None:
        add["write"] = {"pin": pin, "value": {"boolValue": value}}
    protomq.publish(uid, {"digital": {"add": add}})
    protomq.expect_b2d_message(uid, "digital.add", match=add, timeout=5)

    # DUT adds the pin as an output (D_OUTPUT = 3)
    serial.expect(r"^\[digital\] Added new pin:$", timeout=CHECKIN_TIMEOUT)
    serial.expect(rf"^Pin Name: {pin.lstrip('D')}$", timeout=5)
    serial.expect(r"^Direction: 3$", timeout=5)


@pytest.mark.parametrize("secrets_name", ["good"], indirect=True)
def test_digital_add_output(board, board_definition, secrets_name,
                            flashed_board, serial, power, device_uid, protomq,
                            checkin):
    # DUT checks in, then the broker sends the digital.Add for D14 once
    # the DUT is fully checked in
    uid = checkin()
    _add_output(serial, protomq, uid, "D14")


def _write(serial, protomq, probe, uid, pin, value):
    """Send a digital.Write of value to pin, an output, and expect the probe
    to see it go from the other value to value."""
    write = {"pin": pin, "value": {"boolValue": value}}
    # The probe first checks that pin is at the other value, then samples
    # it while the broker sends the digital.Write, and finally asserts it
    # reached value
    with probe.expect_digitalout_transition(
            pin, not value, value, timeout=PIN_TRANSITION_TIMEOUT):
        protomq.publish(uid, {"digital": {"write": write}})
        serial.expect(rf"^\[digital\] Writing: {int(value)} to Pin {pin.lstrip('D')}$",
                      timeout=5)


@pytest.mark.parametrize("secrets_name", ["good"], indirect=True)
def test_digital_write_output_low_high(board, board_definition, secrets_name,
                                       flashed_board, serial, power, device_uid,
                                       protomq, probe, checkin):
    # --- Arrange ------------------------------------------------------
    # DUT checks in and adds D14 as an output, which starts low
    uid = checkin()
    _add_output(serial, protomq, uid, "D14")

    # --- Act / Assert -------------------------------------------------
    # D14 goes low -> high
    _write(serial, protomq, probe, uid, "D14", True)


@pytest.mark.parametrize("secrets_name", ["good"], indirect=True)
def test_digital_write_output_high_low(board, board_definition, secrets_name,
                                       flashed_board, serial, power, device_uid,
                                       protomq, probe, checkin):
    # --- Arrange ------------------------------------------------------
    # DUT checks in and adds D14 as an output, which starts low, then the
    # broker drives it high
    uid = checkin()
    _add_output(serial, protomq, uid, "D14")
    _write(serial, protomq, probe, uid, "D14", True)

    # --- Act / Assert -------------------------------------------------
    # D14 goes high -> low
    _write(serial, protomq, probe, uid, "D14", False)


@pytest.mark.parametrize("secrets_name", ["good"], indirect=True)
def test_digital_add_output_initial_high(board, board_definition, secrets_name,
                                         flashed_board, serial, power,
                                         device_uid, protomq, probe, checkin):
    # --- Arrange ------------------------------------------------------
    # DUT checks in and adds D14 as an output, which drives it low. Until
    # then D14 reads high: after reset GPIO14 is an input with its weak
    # pull-up on
    uid = checkin()
    _add_output(serial, protomq, uid, "D14")

    # --- Act / Assert -------------------------------------------------
    # The probe checks D14 is low and samples it while the broker re-adds
    # D14 with an initial write of high, which Handle_DigitalIO_Add()
    # applies to the new pin (initial_value), then expects it to go high
    with probe.expect_digitalout_transition(
            "D14", False, True, timeout=PIN_TRANSITION_TIMEOUT):
        _add_output(serial, protomq, uid, "D14", value=True)


@pytest.mark.parametrize("secrets_name", ["good"], indirect=True)
def test_digital_add_write_remove(board, board_definition, secrets_name,
                                  flashed_board, serial, power, device_uid,
                                  protomq, probe, checkin):
    # --- Arrange ------------------------------------------------------
    # DUT checks in, adds D14 as an output and the broker drives it high
    uid = checkin()
    _add_output(serial, protomq, uid, "D14")
    _write(serial, protomq, probe, uid, "D14", True)

    # --- Act / Assert -------------------------------------------------
    # DUT removes D14
    remove(serial, protomq, uid, "D14")


@pytest.mark.parametrize("secrets_name", ["good"], indirect=True)
def test_digital_add_write_remove_write(board, board_definition, secrets_name,
                                        flashed_board, serial, power,
                                        device_uid, protomq, probe, checkin):
    # --- Arrange ------------------------------------------------------
    # DUT checks in, adds D14 as an output, the broker drives it high and
    # then removes it
    uid = checkin()
    _add_output(serial, protomq, uid, "D14")
    _write(serial, protomq, probe, uid, "D14", True)
    remove(serial, protomq, uid, "D14")

    # --- Act ----------------------------------------------------------
    # The broker writes to the removed D14
    write = {"pin": "D14", "value": {"boolValue": True}}
    protomq.publish(uid, {"digital": {"write": write}})

    # --- Assert -------------------------------------------------------
    # D14's DigitalIOHardware was deleted by the remove, so GetPin() finds
    # nothing and Handle_DigitalIO_Write() rejects the write
    protomq.expect_d2b_message(
        uid, "error.component",
        match={"pin": "D14", "message": "Failed to find pin"}, timeout=5)
