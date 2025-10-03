# xhandlib/encoders.py
from __future__ import annotations
import time
from typing import Iterable, Callable
from .device import XHandBus

def stream_encoders(bus: XHandBus, ids: Iterable[int], rate_hz: float = 50.0, seconds: float = 2.0,
                    printer: Callable[[str], None] = print):
    """Stream joint positions for 'seconds' at 'rate_hz' for the given hand IDs."""
    ids = list(ids)
    period = 1.0 / max(rate_hz, 1e-6)
    t0 = time.time()
    while time.time() - t0 < seconds:
        parts = []
        for hid in ids:
            st = bus.read_state(hid, False)
            pos = [f"{fs.position:.3f}" for fs in st.finger_state]
            parts.append(f"ID{hid}:[" + ",".join(pos) + "]")
        printer("  ".join(parts))
        time.sleep(period)
