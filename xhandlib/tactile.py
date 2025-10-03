# xhandlib/tactile.py
from __future__ import annotations
import time
from typing import Iterable, Callable
from .device import XHandBus

_SENSOR_IDS = (0x11, 0x12, 0x13, 0x14, 0x15)  # thumb..pinky

def stream_tactile(bus: XHandBus, hand_id: int, rate_hz: float = 10.0, seconds: float = 3.0,
                   printer: Callable[[str], None] = print):
    """Stream tactile readings if supported by the SDK build; else raise NotImplementedError."""
    # reuse the device helper which already raises NotImplementedError if unsupported
    period = 1.0 / max(rate_hz, 1e-6)
    t0 = time.time()
    while time.time() - t0 < seconds:
        readings = bus.read_tactile_all(hand_id)  # may raise NotImplementedError
        line = " ".join(f"{hex(sid)}={readings.get(sid,'NA')}" for sid in _SENSOR_IDS)
        printer(line)
        time.sleep(period)
