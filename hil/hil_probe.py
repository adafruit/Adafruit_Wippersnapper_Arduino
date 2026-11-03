"""Read and drive the DUT's pins from the HIL host, through an I2C GPIO
expander wired to them, for HIL tests that check what the DUT drives on a
pin, or what it reads from one.

The board entry's probes describe the wiring, one per role:

    "probes": {
      "digital_gpio": {"type": "mcp23017_gpio", "address": "0x20",
                       "pin_map": {"D14": "GPA0", "D32": "GPA1"}},
      "pwm_input": {"type": "pi_gpio", "pin_map": {"D27": "D16"}},
      "analog_input": {"type": "mcp4728_dac", "address": "0x60",
                       "pin_map": {"A34": "A"}}
    }

digital_gpio is a Probe: pin_map maps a DUT pin name to the expander pin
it's wired to, GPA0-GPA7 or GPB0-GPB7.

    probe = Probe(entry["probes"]["digital_gpio"])
    probe.value("D14")                       # read it once, now
    with probe.expect_digitalout_transition("D14", False, True, timeout=2):
        pmq.publish(uid, {"digital": {"write": ...}})   # make it go high
    probe.drive("D32", True)                 # drive the DUT's input high
    probe.release("D32")                     # back to an input, not driving
    probe.release()                          # every expander pin, mapped or not

The MCP23017 keeps driving its outputs when the DUT loses power, which can
damage both, so all 16 expander pins are made inputs with no pull when a
Probe is made, by release() with no pin, and by close().

pwm_input is a PulseInProbe: it times the DUT's PWM output on a host GPIO
wired to the DUT pin, with pulseio.PulseIn (an I2C expander can't time
edges). pin_map maps a DUT pin name to the host's Blinka board pin name.

    pulse_probe = PulseInProbe(entry["probes"]["pwm_input"])
    duty, period_us = pulse_probe.measure("D27", periods=100)

analog_input is a DacProbe: it drives a voltage into the DUT's analog
input from a channel of an MCP4728 quad DAC on the host's I2C bus.
pin_map maps a DUT pin name to the DAC channel it's wired to, A-D. The
channels use the DAC's internal 2.048V reference at 2x gain, so 0-4.095V
in 1mV steps (outputs can't go above the DAC's VDD). The optional
max_voltage, 3.3V if not given, is the most the probe will drive, to keep
within the DUT's logic level.

    dac_probe = DacProbe(entry["probes"]["analog_input"])
    dac_probe.set_voltage("A34", 1.65)       # or .write("A34", 2048)
    dac_probe.release()                      # all 4 channels to 0V, powered down

Blinka and adafruit_mcp230xx are imported when a Probe, PulseInProbe or
DacProbe is made, so this module imports on hosts without them.
"""

import contextlib
import functools
import logging
import re
import statistics
import threading
import time

# The probe roles a board entry's probes may have, and the hardware types
# each supports.
PROBE_TYPES = {"digital_gpio": ("mcp23017_gpio",), "pwm_input": ("pi_gpio",),
               "analog_input": ("mcp4728_dac",)}
EXPANDER_PINS = 16
EXPANDER_PIN_RE = re.compile(r"^GP([AB])([0-7])$")
TRACE_SAMPLES = 10  # samples shown when a transition times out

log = logging.getLogger("hil.probe")


class ProbeError(Exception):
    """The probe's config is invalid, or a pin isn't mapped."""


class ProbeTimeout(ProbeError, AssertionError):
    """A pin didn't reach the expected value in time."""


def resolve_expander_pin(name):
    """The MCP23017 pin number for name: GPA0-GPA7 -> 0-7, GPB0-GPB7 -> 8-15."""
    m = EXPANDER_PIN_RE.match(name) if isinstance(name, str) else None
    if not m:
        raise ValueError(f"expander pin must be GPA0-GPA7 or GPB0-GPB7, got {name!r}")
    bank, pin = m.groups()
    return (8 if bank == "B" else 0) + int(pin)


def _check_type(role, config):
    """The config's type, if role supports it."""
    probe_type = config.get("type")
    if probe_type not in PROBE_TYPES[role]:
        raise ProbeError(f"{role} probe 'type' must be one of "
                         f"{PROBE_TYPES[role]}, got {probe_type!r}")
    return probe_type


class Probe:
    def __init__(self, config):
        probe_type = _check_type("digital_gpio", config)
        try:
            self.address = int(str(config.get("address")), 0)
        except ValueError:
            raise ProbeError(f"digital_gpio probe 'address' must be an I2C "
                             f"address like \"0x20\", got "
                             f"{config.get('address')!r}") from None
        pin_map = config.get("pin_map")
        if not isinstance(pin_map, dict):
            raise ProbeError("digital_gpio probe 'pin_map' must be an object, "
                             "e.g. {\"D14\": \"GPA0\"}")
        # Resolve now, so a typo fails before the test runs
        self.pin_map = {dut: resolve_expander_pin(exp) for dut, exp in pin_map.items()}

        import board
        import busio
        import digitalio
        from adafruit_mcp230xx.mcp23017 import MCP23017

        self._direction_input = digitalio.Direction.INPUT
        self._direction_output = digitalio.Direction.OUTPUT
        self._i2c = busio.I2C(board.SCL, board.SDA)
        self._mcp = MCP23017(self._i2c, address=self.address)
        self._pins = {}
        log.info("probe %s at 0x%02x: %s", probe_type, self.address, pin_map)
        self.release()  # an earlier run may have left pins driving

    def _pin(self, dut_pin):
        if dut_pin not in self._pins:
            if dut_pin not in self.pin_map:
                raise ProbeError(f"DUT pin {dut_pin!r} isn't in the probe's "
                                 f"pin_map {sorted(self.pin_map)}")
            pin = self._mcp.get_pin(self.pin_map[dut_pin])
            pin.direction = self._direction_input
            pin.pull = None
            self._pins[dut_pin] = pin
        return self._pins[dut_pin]

    def drive(self, dut_pin, value):
        """Drive the DUT pin to value. Sets the output latch before making
        the expander pin an output, so it doesn't glitch to the old latch."""
        pin = self._pin(dut_pin)
        pin.value = value
        pin.direction = self._direction_output
        log.info("probe driving %s %d", dut_pin, value)

    def release(self, dut_pin=None):
        """Stop driving the DUT pin: make the expander pin an input with no
        pull. With no pin, do that to all 16 expander pins."""
        if dut_pin is not None:
            pins = [self._pin(dut_pin)]
        else:
            pins = [self._mcp.get_pin(n) for n in range(EXPANDER_PINS)]
        for pin in pins:
            pin.direction = self._direction_input
            pin.pull = None
        if dut_pin is None:
            log.info("released all probe pins")

    def value(self, dut_pin):
        """The DUT pin's level right now."""
        return bool(self._pin(dut_pin).value)

    @contextlib.contextmanager
    def expect_digitalout_transition(self, dut_pin, from_value, to_value, *,
                                     frequency=200, timeout=2):
        """Check dut_pin reads from_value now, sample it at frequency Hz while
        the block runs (the block sends the command), then wait until it
        reads to_value, at most timeout seconds after entering."""
        start = time.monotonic()
        initial = self.value(dut_pin)
        assert initial == from_value, (
            f"{dut_pin} should start at {int(from_value)} but reads {int(initial)}")

        samples = [(0.0, initial)]
        reached = threading.Event()
        stop = threading.Event()

        def sample():
            interval = 1 / frequency
            while not stop.is_set():
                value = self.value(dut_pin)
                samples.append((time.monotonic() - start, value))
                if value == to_value:
                    reached.set()
                    return
                stop.wait(interval)

        sampler = threading.Thread(target=sample, name=f"probe-{dut_pin}", daemon=True)
        sampler.start()
        try:
            yield samples
            remaining = timeout - (time.monotonic() - start)
            if not reached.wait(max(remaining, 0)):
                trace = ", ".join(f"{t:.3f}s={int(v)}" for t, v in samples[-TRACE_SAMPLES:])
                raise ProbeTimeout(
                    f"{dut_pin} didn't go {int(from_value)} -> {int(to_value)} "
                    f"within {timeout}s: last read {int(samples[-1][1])} "
                    f"({len(samples)} samples; last: {trace})")
            log.info("%s went %d -> %d at %.3fs", dut_pin, from_value, to_value,
                     samples[-1][0])
        finally:
            stop.set()
            sampler.join()

    def close(self):
        """Release all 16 expander pins, so it drives nothing into the DUT
        after the test, then free the I2C bus."""
        try:
            self.release()
        finally:
            self._i2c.deinit()


# The host's MCP4728 quad DAC, always at this address
DAC_ADDRESS = 0x60
DAC_CHANNELS = 4
# PD 01 powers a channel down with its output pulled to GND through 1k,
# so it reads 0V.
DAC_PD_1K = 0b01
# Fast Write, one 2-byte word per channel A-D: 00 PD1 PD0 D11-D8, D7-D0.
# Changes only the input registers, which drive the outputs now.
DAC_RELEASE_CMD = bytes([DAC_PD_1K << 4, 0x00] * DAC_CHANNELS)
# Sequential Write from channel A (01010 DAC1 DAC0 UDAC), then one word
# per channel A-D: VREF PD1 PD0 Gx D11-D8, D7-D0. Writes the input
# registers and the EEPROM, which the DAC loads at power up.
DAC_SAVE_CMD = bytes([0b01010000] + [DAC_PD_1K << 5, 0x00] * DAC_CHANNELS)
DAC_EEPROM_TIMEOUT = 0.5  # the datasheet's EEPROM write is 50ms at most
DAC_MAX = 4095  # 12-bit
# Internal 2.048V reference at 2x gain: 4.096V full scale, 1mV per step
DAC_VREF_INTERNAL = 0b1
DAC_GAIN_2X = 0b1
DAC_FULL_SCALE = 2 * 2.048
# The most a DacProbe drives if its config has no max_voltage, the DUT's
# logic level
DAC_DEFAULT_MAX_VOLTAGE = 3.3


def _dac_read(i2c, address):
    """Each channel's (ready, (PD, value) of its input register, (PD,
    value) of its EEPROM). The DAC sends 6 bytes per channel: input
    register status, high, low, then EEPROM status, high, low."""
    buf = bytearray(6 * DAC_CHANNELS)
    i2c.readfrom_into(address, buf)
    channels = []
    for ch in range(DAC_CHANNELS):
        b = buf[6 * ch:6 * ch + 6]
        input_reg = ((b[1] >> 5) & 0b11, ((b[1] & 0x0F) << 8) | b[2])
        eeprom = ((b[4] >> 5) & 0b11, ((b[4] & 0x0F) << 8) | b[5])
        channels.append((bool(b[0] & 0x80), input_reg, eeprom))
    return channels


def release_dac(address=DAC_ADDRESS):
    """Power down all four channels of the MCP4728 at address, value 0
    and output pulled to GND through 1k, so it drives nothing into the
    DUT, then read them back to check. Saves that to the EEPROM too, so
    the DAC powers up released, but only when the EEPROM differs, to
    spare its write endurance."""
    import board
    import busio

    released = (DAC_PD_1K, 0)
    i2c = busio.I2C(board.SCL, board.SDA)
    try:
        while not i2c.try_lock():
            time.sleep(0.001)
        try:
            i2c.writeto(address, DAC_RELEASE_CMD)
            channels = _dac_read(i2c, address)
            # NOTE: the EEPROM is only written when it differs, since it's
            # rated for ~1M writes (~11 months at 3,000 tests/day if written
            # every test). We may want to write it unconditionally in the
            # future, e.g. if something can change the EEPROM without this
            # readback catching it; that costs 25-50ms per test.
            saved = all(eeprom == released for _, _, eeprom in channels)
            if not saved:
                i2c.writeto(address, DAC_SAVE_CMD)
                # RDY/BSY reads 0 while the EEPROM write is in progress
                deadline = time.monotonic() + DAC_EEPROM_TIMEOUT
                while True:
                    time.sleep(0.01)
                    channels = _dac_read(i2c, address)
                    if all(ready for ready, _, _ in channels):
                        break
                    if time.monotonic() > deadline:
                        raise ProbeError(f"DAC 0x{address:02x} EEPROM write "
                                         f"didn't finish in "
                                         f"{DAC_EEPROM_TIMEOUT}s")
        finally:
            i2c.unlock()
    finally:
        i2c.deinit()
    for ch, (_, input_reg, eeprom) in enumerate(channels):
        for where, (power_down, value) in (("output", input_reg),
                                           ("EEPROM", eeprom)):
            if (power_down, value) != released:
                raise ProbeError(f"DAC 0x{address:02x} channel {'ABCD'[ch]} "
                                 f"{where} didn't power down: PD "
                                 f"{power_down:02b}, value {value}")
    log.info("released all DAC channels at 0x%02x%s", address,
             "" if saved else ", saved to EEPROM")


class DacProbe:
    """Drive the DUT's analog inputs from the channels of an MCP4728 quad
    DAC, referenced to its internal 2.048V at 2x gain, so a channel
    outputs 4.096V * value / 4096, value mV, up to max_voltage."""

    def __init__(self, config):
        probe_type = _check_type("analog_input", config)
        try:
            self.address = int(str(config.get("address")), 0)
        except ValueError:
            raise ProbeError(f"analog_input probe 'address' must be an I2C "
                             f"address like \"0x60\", got "
                             f"{config.get('address')!r}") from None
        pin_map = config.get("pin_map")
        if not isinstance(pin_map, dict):
            raise ProbeError("analog_input probe 'pin_map' must be an object, "
                             "e.g. {\"A34\": \"A\"}")
        self.pin_map = {}
        for dut, channel in pin_map.items():
            if channel not in ("A", "B", "C", "D"):
                raise ProbeError(f"analog_input probe channel for {dut!r} "
                                 f"must be A-D, got {channel!r}")
            self.pin_map[dut] = "ABCD".index(channel)
        self.max_voltage = float(config.get("max_voltage",
                                            DAC_DEFAULT_MAX_VOLTAGE))
        if not 0 < self.max_voltage <= DAC_FULL_SCALE:
            raise ProbeError(f"analog_input probe 'max_voltage' must be "
                             f"0-{DAC_FULL_SCALE}V, got {self.max_voltage}")
        # The highest value write() takes, compared as an integer so float
        # rounding can't make max_voltage itself (e.g. 3.3) read as over
        self._max_value = min(int(self.max_voltage / DAC_FULL_SCALE
                                  * (DAC_MAX + 1) + 1e-6), DAC_MAX)

        import board
        import busio

        self._i2c = busio.I2C(board.SCL, board.SDA)
        log.info("dac probe %s at 0x%02x: %s, max %.3fV", probe_type,
                 self.address, pin_map, self.max_voltage)
        self.release()  # an earlier run may have left channels driving

    def write(self, dut_pin, value):
        """Power up the DAC channel wired to dut_pin and set it to value,
        0-4095 (mV), with Multi-Write (01000 DAC1 DAC0 UDAC, then VREF PD1
        PD0 Gx D11-D8, D7-D0): internal reference, 2x gain, output updated
        now. Refuses a value over max_voltage."""
        if dut_pin not in self.pin_map:
            raise ProbeError(f"DUT pin {dut_pin!r} isn't in the DAC probe's "
                             f"pin_map {sorted(self.pin_map)}")
        if not 0 <= value <= DAC_MAX:
            raise ProbeError(f"DAC value must be 0-{DAC_MAX}, got {value}")
        volts = DAC_FULL_SCALE * value / (DAC_MAX + 1)
        if value > self._max_value:
            raise ProbeError(f"DAC value {value} is {volts:.3f}V, over the "
                             f"probe's max_voltage {self.max_voltage}V")
        channel = self.pin_map[dut_pin]
        high = DAC_VREF_INTERNAL << 7 | DAC_GAIN_2X << 4 | value >> 8
        cmd = bytes([0b01000000 | channel << 1, high, value & 0xFF])
        while not self._i2c.try_lock():
            time.sleep(0.001)
        try:
            self._i2c.writeto(self.address, cmd)
        finally:
            self._i2c.unlock()
        log.info("dac driving %s (channel %s) %d, %.3fV", dut_pin,
                 "ABCD"[channel], value, volts)

    def set_voltage(self, dut_pin, volts):
        """Drive dut_pin to the nearest volts the DAC can make, 0 to
        max_voltage, and return the voltage it's set to."""
        if not 0 <= volts <= self.max_voltage:
            raise ProbeError(f"DAC voltage must be 0-{self.max_voltage}V, "
                             f"got {volts}")
        value = min(int(volts / DAC_FULL_SCALE * (DAC_MAX + 1) + 0.5), DAC_MAX)
        self.write(dut_pin, value)
        return DAC_FULL_SCALE * value / (DAC_MAX + 1)

    def release(self):
        """Power down all four channels, outputs pulled to 0V, saved to the
        EEPROM (see release_dac())."""
        release_dac(self.address)

    def close(self):
        """Release all four channels, then free the I2C bus."""
        try:
            self.release()
        finally:
            self._i2c.deinit()


def missed_edges(lengths, period_us):
    """How many edges a capture of pulse lengths (us) of a period_us signal
    missed. Every recorded pulse ends on an edge, so the time they cover
    holds 2 pulses per period; any fewer recorded were merged by a missed
    edge."""
    return max(round(2 * sum(lengths) / period_us) - len(lengths), 0)


class PulseInProbe:
    """Time the DUT's PWM output on host GPIOs with pulseio.PulseIn. The
    host pins are only ever inputs, so they never drive the DUT."""

    # Extra PulseIn capacity over what capture() needs, so pulses that
    # arrive between its last len() and pause() don't drop the oldest and
    # lose track of which pulses are high. 10ms of a 100 kHz signal.
    SLACK = 2048
    # Times measure() recaptures after one misses edges. The OS rarely
    # stalls the capture helper for a few ms (AO-00 saw it in 1 of ~1,400
    # captures), and no edge was missed at 5 kHz otherwise.
    STALL_RETRIES = 2
    # How long to wait for the PulseIn helper to answer one len() or [i].
    # It answers in well under 1ms when it's healthy.
    REPLY_TIMEOUT = 2

    def __init__(self, config):
        _check_type("pwm_input", config)
        pin_map = config.get("pin_map")
        if not isinstance(pin_map, dict):
            raise ProbeError("pwm_input probe 'pin_map' must be an object, "
                             "e.g. {\"D27\": \"D16\"}")

        import board
        import pulseio

        self._pulseio = pulseio
        self.pin_map = {}
        for dut, host in pin_map.items():
            if not hasattr(board, host):
                raise ProbeError(f"pwm_input probe pin {host!r} for {dut!r} "
                                 f"isn't a board pin")
            self.pin_map[dut] = getattr(board, host)
        self._pulses = {}
        log.info("pulse probe: %s", pin_map)

    def _pulse_in(self, dut_pin, maxlen):
        pulses = self._pulses.get(dut_pin)
        if pulses is None or pulses.maxlen < maxlen:
            if dut_pin not in self.pin_map:
                raise ProbeError(f"DUT pin {dut_pin!r} isn't in the pulse "
                                 f"probe's pin_map {sorted(self.pin_map)}")
            if pulses is not None:
                pulses.deinit()
            pulses = self._pulseio.PulseIn(self.pin_map[dut_pin],
                                           maxlen=maxlen, idle_state=False)
            # Blinka waits on the helper's replies (len(), [i]) with no
            # timeout, so a helper that dies mid-read hangs the test forever
            pulses._wait_receive_msg = functools.partial(
                self._wait_reply, dut_pin, pulses)
            self._pulses[dut_pin] = pulses
        return pulses

    def _wait_reply(self, dut_pin, pulses, timeout=0, type=2):  # pylint: disable=redefined-builtin
        """Stands in for pulses._wait_receive_msg: waits at most timeout
        (REPLY_TIMEOUT if 0) for the helper's reply, raising ProbeError if
        it doesn't come or the helper exits. A dead PulseIn is dropped, so
        the next call on dut_pin starts a new helper."""
        import sysv_ipc

        deadline = time.monotonic() + (timeout or self.REPLY_TIMEOUT)
        while True:
            try:
                return pulses._mq.receive(block=False, type=type)
            except sysv_ipc.BusyError:
                pass
            process = pulses._process
            if process is None or process.poll() is not None:
                problem = (f"PulseIn's helper process exited (code "
                           f"{None if process is None else process.returncode})")
            elif time.monotonic() > deadline:
                problem = (f"PulseIn's helper didn't reply within "
                           f"{timeout or self.REPLY_TIMEOUT}s")
            else:
                time.sleep(0.0001)
                continue
            if self._pulses.get(dut_pin) is pulses:
                del self._pulses[dut_pin]
            pulses.deinit()
            raise ProbeError(f"{dut_pin}: {problem}")

    def capture(self, dut_pin, count, timeout=2):
        """Capture count pulse lengths of dut_pin, an active high signal, in
        us. The first is a high pulse, so even indexes are high pulses and
        odd ones low, as long as no edge was missed."""
        needed = count + 2  # the first pair may be partial
        pulses = self._pulse_in(dut_pin, needed + self.SLACK)
        pulses.pause()
        pulses.clear()
        # After resume() the first pulse recorded is the active (high) one
        pulses.resume()
        start = time.monotonic()
        while len(pulses) < needed:
            if time.monotonic() - start > timeout:
                got = len(pulses)
                pulses.pause()
                raise ProbeTimeout(
                    f"{dut_pin}: only {got} of {needed} pulses within "
                    f"{timeout}s, is it toggling?")
            time.sleep(0.01)
        pulses.pause()
        got = len(pulses)
        if got >= pulses.maxlen:
            raise ProbeError(f"{dut_pin}: PulseIn filled ({got}), which "
                             f"pulses are high is unknown")
        return [pulses[i] for i in range(2, needed)]

    def measure(self, dut_pin, periods=100, timeout=2):
        """Capture periods full high+low periods of dut_pin, an active high
        PWM output, and return its duty (0-1) and mean period in us.
        Recaptures up to STALL_RETRIES times if a capture misses edges."""
        for attempt in range(self.STALL_RETRIES + 1):
            lengths = self.capture(dut_pin, 2 * periods, timeout)
            pairs = [h + l for h, l in zip(lengths[0::2], lengths[1::2])]
            # The median period ignores the few pairs a miss merges, so
            # this doesn't assume the DUT's frequency is right
            missed = missed_edges(lengths, statistics.median(pairs))
            if not missed:
                break
            log.warning("%s capture %d missed %d edges%s", dut_pin,
                        attempt + 1, missed,
                        ", retrying" if attempt < self.STALL_RETRIES else "")
        else:
            raise ProbeError(f"{dut_pin}: all {self.STALL_RETRIES + 1} "
                             f"captures missed edges, last {missed}")
        high, low = lengths[0::2], lengths[1::2]
        period_lengths = [h + l for h, l in zip(high, low)]
        duty = sum(high) / sum(period_lengths)
        period_us = statistics.mean(period_lengths)
        log.info("%s duty %.2f%%, period %.1fus (min %d, max %d) over %d "
                 "periods", dut_pin, duty * 100, period_us,
                 min(period_lengths), max(period_lengths), periods)
        return duty, period_us

    def count_pulses(self, dut_pin, window):
        """How many pulses dut_pin had over window seconds: 0 if it held
        one level throughout."""
        pulses = self._pulse_in(dut_pin, 2 + self.SLACK)
        pulses.pause()
        pulses.clear()
        pulses.resume()
        time.sleep(window)
        pulses.pause()
        return len(pulses)

    def start_record(self, dut_pin, max_pulses):
        """Start recording dut_pin's pulses, at most max_pulses of them,
        and return the time.time() the recording started, the clock
        SerialMonitor stamps lines with. The first pulse recorded is the
        first high one starting after that, within one period."""
        pulses = self._pulse_in(dut_pin, max_pulses + self.SLACK)
        pulses.pause()
        pulses.clear()
        pulses.resume()
        return time.time()

    def stop_record(self, dut_pin):
        """Stop the recording start_record() began and return its pulse
        lengths, in us, high first."""
        pulses = self._pulses[dut_pin]
        pulses.pause()
        count = len(pulses)
        if count >= pulses.maxlen:
            raise ProbeError(f"{dut_pin}: recording filled PulseIn ({count}), "
                             f"its start is lost")
        start = time.monotonic()
        lengths = [pulses[i] for i in range(count)]
        log.info("%s recorded %d pulses, read in %.2fs", dut_pin, count,
                 time.monotonic() - start)
        return lengths

    def level(self, dut_pin):
        """dut_pin's level now, read with the host pin's pull-down on, so
        an undriven DUT pin reads low. Stops any PulseIn on the pin."""
        if dut_pin not in self.pin_map:
            raise ProbeError(f"DUT pin {dut_pin!r} isn't in the pulse "
                             f"probe's pin_map {sorted(self.pin_map)}")
        pulses = self._pulses.pop(dut_pin, None)
        if pulses is not None:
            pulses.deinit()
        # lgpio, not digitalio: Blinka's deinit() keeps the line claimed,
        # so the next PulseIn couldn't take it
        import lgpio
        chip = lgpio.gpiochip_open(0)
        line = self.pin_map[dut_pin].id
        try:
            lgpio.gpio_claim_input(chip, line, lgpio.SET_PULL_DOWN)
            try:
                time.sleep(0.01)
                return bool(lgpio.gpio_read(chip, line))
            finally:
                lgpio.gpio_free(chip, line)
        finally:
            lgpio.gpiochip_close(chip)

    def close(self):
        """Stop every PulseIn and its helper process."""
        for pulses in self._pulses.values():
            pulses.deinit()
        self._pulses.clear()
