"""
Shared helpers for the digitalIO HIL tests (test_digital_out.py,
test_digital_in.py).
"""

# How long after checkin the DUT may take to handle a message, as the
# checkin fixture allows for the checkin itself.
CHECKIN_TIMEOUT = 30


def remove(serial, protomq, uid, pin):
    """Send a digital.Remove for pin and expect the DUT to remove it."""
    protomq.publish(uid, {"digital": {"remove": {"pin": pin}}})
    serial.expect(rf"^\[digital\] Pin removed: {pin.lstrip('D')}$", timeout=5)
