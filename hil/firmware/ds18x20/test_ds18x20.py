"""
HIL tests for Adafruit Wippersnapper firmware's DS18x20 component.

How Does This Work?
A DS18B20 is wired to a DUT pin with a 4.7k pull-up to 3V; the board
entry's onewire_sensor probe names the pin (the onewire_pin fixture).
The broker adds, modifies and removes it, and the tests check the
temperature events the DUT publishes, their timing, and the component
errors it publishes for bad requests.

Exercises:
- `src/components/ds18x20/controller.cpp::Router()`
- `src/components/ds18x20/controller.cpp::Router()` (ds18x20.Modify)
- `src/components/ds18x20/controller.cpp::Handle_Ds18x20Add()`
- `src/components/ds18x20/controller.cpp::Handle_Ds18x20Remove()`
- `src/components/ds18x20/controller.cpp::update()`
- `src/components/ds18x20/hardware.cpp::GetSensor()` (and with no sensor)
- `src/components/ds18x20/hardware.cpp::SetPeriod()`
- `src/components/ds18x20/hardware.cpp::IsTimerExpired()`
- `src/components/ds18x20/hardware.cpp::ReadTemperatureC()`
- `src/components/ds18x20/hardware.cpp::GetTemperatureF()`
- `src/components/ds18x20/model.cpp::addSensorEvent()`
- `src/components/ds18x20/model.cpp::EncodeDs18x20Event()`
"""

import logging

import pytest


log = logging.getLogger("hil.ds18x20")

# A DUT pin with nothing wired to it (no sensor, no pull-up), for adding a
# sensor where there's none
EMPTY_PIN = "D15"
# ws.sensor.Type values for each unit
CELSIUS = "T_OBJECT_TEMPERATURE"
FAHRENHEIT = "T_OBJECT_TEMPERATURE_FAHRENHEIT"
# The Add's settings: 12-bit resolution, read every PERIOD seconds, in
# Celsius unless a test says otherwise
RESOLUTION = 12
PERIOD = 10
# The Modify's shorter period, in seconds
MODIFIED_PERIOD = 3
# How many intervals between events to time after the Modify
MODIFIED_INTERVALS = 3
# How long after checkin the DUT may take to handle a message, as the
# checkin fixture allows for the checkin itself.
CHECKIN_TIMEOUT = 30
# SetPeriod() zeroes the timer, so the first read follows an Add (or
# Modify) at once; allow for the 750 ms 12-bit conversion and the publish.
FIRST_EVENT_TIMEOUT = 10
# How far an interval between events may be from the period, in seconds.
# ReadTemperatureC() restarts the timer after the conversion, so an
# interval is the period plus up to MAX_CONV_TIME (750 ms), plus the
# loop's and the network's latency.
INTERVAL_EARLY = 0.5
INTERVAL_LATE = 2.0
# How many periods to watch for events after a Remove
REMOVED_PERIODS = 2.5
# A room-temperature reading, per unit. Rules out the 85.0 C power-on
# value and the 0.0 C of an all-zero scratchpad.
TEMP_RANGE = {CELSIUS: (5.0, 50.0), FAHRENHEIT: (41.0, 122.0)}
# addSensorEvent() rounds each value to 2 decimals, so a Fahrenheit value
# may be this far from its Celsius value converted
ROUNDING = 0.011


def _add_msg(pin, period, types=(CELSIUS,)):
    """A ds18x20.Add for pin, reading types every period."""
    return {"onewirePin": pin, "sensorResolution": RESOLUTION,
            "period": period, "types": list(types)}


def _expect_added(serial):
    """Expect Handle_Ds18x20Add() to find and add the sensor."""
    serial.expect(r"^\[ds18x20\] Handle_Ds18x20Add MESSAGE\.\.\.$",
                  timeout=CHECKIN_TIMEOUT)
    serial.expect(r"^\[ds18x\] New Sensor Added!$", timeout=5)


def _expect_removed(serial):
    """Expect Handle_Ds18x20Remove() to remove the sensor. It logs both
    halves on one line."""
    serial.expect(r"^\[ds18x20\] Handle_Ds18x20Remove MESSAGE\.\.\.Removed!$",
                  timeout=CHECKIN_TIMEOUT)


def _add(serial, protomq, uid, pin, period, types=(CELSIUS,)):
    """Send a ds18x20.Add for pin reading types, and expect the DUT to add
    it."""
    add = _add_msg(pin, period, types)
    protomq.publish(uid, {"ds18x20": {"add": add}})
    protomq.expect_b2d_message(uid, "ds18x20.add", match=add, timeout=5)
    _expect_added(serial)


def _modify(serial, protomq, uid, pin, period):
    """Send a ds18x20.Modify that replaces pin's sensor with one reading
    every period, and expect the DUT to remove then re-add it."""
    modify = {"remove": {"onewirePin": pin}, "add": _add_msg(pin, period)}
    protomq.publish(uid, {"ds18x20": {"modify": modify}})
    protomq.expect_b2d_message(uid, "ds18x20.modify", match=modify, timeout=5)
    _expect_removed(serial)
    _expect_added(serial)


def _remove(serial, protomq, uid, pin):
    """Send a ds18x20.Remove for pin and expect the DUT to remove it."""
    remove = {"onewirePin": pin}
    protomq.publish(uid, {"ds18x20": {"remove": remove}})
    protomq.expect_b2d_message(uid, "ds18x20.remove", match=remove, timeout=5)
    _expect_removed(serial)


def _expect_event(protomq, uid, pin, timeout, types=(CELSIUS,)):
    """Expect the broker to get a ds18x20.event for pin holding a
    sensor event per type in types, in that order (update()'s: Celsius,
    then Fahrenheit), each a room temperature. Returns when the broker
    received it, in seconds since the epoch, and {type: value}."""
    event = protomq.expect_d2b_message(
        uid, "ds18x20.event", match={"onewirePin": pin},
        timeout=timeout)
    events = event.get("events", [])
    got = [e.get("type") for e in events]
    assert got == list(types), (
        f"expected sensor events of {list(types)}, got {got}: {event}")
    values = {}
    for sensor_event in events:
        kind = sensor_event["type"]
        value = sensor_event.get("floatValue")
        assert value is not None, f"expected a floatValue: {event}"
        low, high = TEMP_RANGE[kind]
        assert low <= value <= high, (
            f"expected {kind} {low}-{high}, got {value}: {event}")
        values[kind] = value
    received = protomq.last_d2b_time(uid)
    log.info("%s: %s at %s", pin, values, received)
    return received, values


def _expect_component_error(protomq, uid, pin, message):
    """Expect the DUT to publish an error.component for pin with message.
    Handle_Ds18x20Remove() and Router() don't log it."""
    protomq.expect_d2b_message(uid, "error.component",
                               match={"pin": pin, "message": message},
                               timeout=CHECKIN_TIMEOUT)


def _assert_no_events(protomq, uid, period):
    """Expect no ds18x20.event for REMOVED_PERIODS of period."""
    protomq.assert_no_d2b_message(uid, "ds18x20.event",
                                  within=REMOVED_PERIODS * period)


@pytest.mark.parametrize("secrets_name", ["good"], indirect=True)
def test_ds18x20_add_event_celsius(board, secrets_name, flashed_board,
                                   serial, protomq, checkin, onewire_pin):
    # --- Arrange ------------------------------------------------------
    uid = checkin()

    # --- Act ----------------------------------------------------------
    _add(serial, protomq, uid, onewire_pin, PERIOD)

    # --- Assert -------------------------------------------------------
    _expect_event(protomq, uid, onewire_pin, FIRST_EVENT_TIMEOUT)


@pytest.mark.parametrize("secrets_name", ["good"], indirect=True)
def test_ds18x20_modify_period(board, secrets_name, flashed_board, serial,
                               protomq, checkin, onewire_pin):
    # --- Arrange ------------------------------------------------------
    uid = checkin()
    _add(serial, protomq, uid, onewire_pin, PERIOD)
    _expect_event(protomq, uid, onewire_pin, FIRST_EVENT_TIMEOUT)

    # --- Act ----------------------------------------------------------
    # Sent just after an event, so none at the old period is in flight
    _modify(serial, protomq, uid, onewire_pin, MODIFIED_PERIOD)

    # --- Assert -------------------------------------------------------
    # The re-added sensor reads at once, then every MODIFIED_PERIOD
    times = [_expect_event(protomq, uid, onewire_pin,
                           FIRST_EVENT_TIMEOUT)[0]]
    for _ in range(MODIFIED_INTERVALS):
        times.append(_expect_event(protomq, uid, onewire_pin,
                                   MODIFIED_PERIOD + INTERVAL_LATE + 5)[0])
    if None in times:
        pytest.fail("ProtoMQ doesn't timestamp deliveries, can't time the "
                    "events", pytrace=False)
    intervals = [b - a for a, b in zip(times, times[1:])]
    log.info("intervals after modify to %ss: %s", MODIFIED_PERIOD,
             ", ".join(f"{i:.2f}s" for i in intervals))
    for interval in intervals:
        assert (MODIFIED_PERIOD - INTERVAL_EARLY <= interval
                <= MODIFIED_PERIOD + INTERVAL_LATE), (
            f"expected events every {MODIFIED_PERIOD}s "
            f"(-{INTERVAL_EARLY}/+{INTERVAL_LATE}s), got intervals "
            f"{[round(i, 2) for i in intervals]}")

    # Remove it, just after an event so none is in flight, and expect no
    # more events
    _remove(serial, protomq, uid, onewire_pin)
    _assert_no_events(protomq, uid, MODIFIED_PERIOD)


@pytest.mark.parametrize("secrets_name", ["good"], indirect=True)
def test_ds18x20_remove_stops_events(board, secrets_name, flashed_board,
                                     serial, protomq, checkin, onewire_pin):
    # --- Arrange ------------------------------------------------------
    uid = checkin()
    _add(serial, protomq, uid, onewire_pin, PERIOD)
    _expect_event(protomq, uid, onewire_pin, FIRST_EVENT_TIMEOUT)

    # --- Act ----------------------------------------------------------
    # Sent just after an event, so none is in flight
    _remove(serial, protomq, uid, onewire_pin)

    # --- Assert -------------------------------------------------------
    _assert_no_events(protomq, uid, PERIOD)


@pytest.mark.parametrize("secrets_name", ["good"], indirect=True)
def test_ds18x20_add_event_fahrenheit(board, secrets_name, flashed_board,
                                      serial, protomq, checkin, onewire_pin):
    # --- Arrange ------------------------------------------------------
    uid = checkin()

    # --- Act ----------------------------------------------------------
    _add(serial, protomq, uid, onewire_pin, PERIOD, types=(FAHRENHEIT,))

    # --- Assert -------------------------------------------------------
    _expect_event(protomq, uid, onewire_pin, FIRST_EVENT_TIMEOUT,
                  types=(FAHRENHEIT,))


@pytest.mark.parametrize("secrets_name", ["good"], indirect=True)
def test_ds18x20_add_event_celsius_and_fahrenheit(board, secrets_name,
                                                  flashed_board, serial,
                                                  protomq, checkin,
                                                  onewire_pin):
    # --- Arrange ------------------------------------------------------
    uid = checkin()
    types = (CELSIUS, FAHRENHEIT)

    # --- Act ----------------------------------------------------------
    _add(serial, protomq, uid, onewire_pin, PERIOD, types=types)

    # --- Assert -------------------------------------------------------
    # One event holding both units, of the same reading
    _, values = _expect_event(protomq, uid, onewire_pin, FIRST_EVENT_TIMEOUT,
                              types=types)
    expected_f = values[CELSIUS] * 9 / 5 + 32
    assert abs(values[FAHRENHEIT] - expected_f) <= ROUNDING, (
        f"{values[FAHRENHEIT]} F isn't {values[CELSIUS]} C "
        f"({expected_f:.3f} F)")


@pytest.mark.parametrize("secrets_name", ["good"], indirect=True)
def test_ds18x20_remove_no_sensor(board, secrets_name, flashed_board,
                                  serial, protomq, checkin, onewire_pin):
    # --- Arrange ------------------------------------------------------
    # Nothing added on the sensor's pin, so there's no sensor to remove
    uid = checkin()
    remove = {"onewirePin": onewire_pin}

    # --- Act ----------------------------------------------------------
    protomq.publish(uid, {"ds18x20": {"remove": remove}})

    # --- Assert -------------------------------------------------------
    protomq.expect_b2d_message(uid, "ds18x20.remove", match=remove, timeout=5)
    serial.expect(r"^\[ds18x20\] Handle_Ds18x20Remove MESSAGE\.\.\.",
                  timeout=CHECKIN_TIMEOUT)
    _expect_component_error(protomq, uid, onewire_pin, "Sensor not found")


@pytest.mark.parametrize("secrets_name", ["good"], indirect=True)
def test_ds18x20_modify_without_add(board, secrets_name, flashed_board,
                                    serial, protomq, checkin, onewire_pin):
    # --- Arrange ------------------------------------------------------
    uid = checkin()
    _add(serial, protomq, uid, onewire_pin, MODIFIED_PERIOD)
    _expect_event(protomq, uid, onewire_pin, FIRST_EVENT_TIMEOUT)
    modify = {"remove": {"onewirePin": onewire_pin}}

    # --- Act ----------------------------------------------------------
    protomq.publish(uid, {"ds18x20": {"modify": modify}})

    # --- Assert -------------------------------------------------------
    # Router() rejects it before removing anything...
    protomq.expect_b2d_message(uid, "ds18x20.modify", match=modify, timeout=5)
    _expect_component_error(protomq, uid, onewire_pin,
                            "Modify requires an Add, nothing was removed")
    serial.assert_none(r"Handle_Ds18x20Remove", duration=1)
    # ...so the sensor keeps publishing at its period
    _expect_event(protomq, uid, onewire_pin,
                  MODIFIED_PERIOD + INTERVAL_LATE + 5)


@pytest.mark.parametrize("secrets_name", ["good"], indirect=True)
def test_ds18x20_add_no_sensor(board, secrets_name, flashed_board, serial,
                               protomq, checkin):
    # --- Arrange ------------------------------------------------------
    uid = checkin()
    add = _add_msg(EMPTY_PIN, PERIOD)

    # --- Act ----------------------------------------------------------
    protomq.publish(uid, {"ds18x20": {"add": add}})

    # --- Assert -------------------------------------------------------
    # Handle_Ds18x20Add() finds no sensor on the bus and rejects the Add
    protomq.expect_b2d_message(uid, "ds18x20.add", match=add, timeout=5)
    serial.expect(r"^\[ds18x20\] Handle_Ds18x20Add MESSAGE\.\.\.$",
                  timeout=CHECKIN_TIMEOUT)
    _expect_component_error(protomq, uid, EMPTY_PIN,
                            "Unable to initialize sensor on OneWire bus")
    serial.assert_none(r"^\[ds18x\] New Sensor Added!$")
    # ...so it never publishes an event for it
    protomq.assert_no_d2b_message(uid, "ds18x20.event",
                                  within=REMOVED_PERIODS * PERIOD)
