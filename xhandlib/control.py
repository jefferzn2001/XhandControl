# xhandlib/control.py
from __future__ import annotations
import time
from typing import Iterable, Sequence
from .device import XHandBus

def send_pose(bus: XHandBus, hand_id: int, targets: Iterable[float], hold_seconds: float = 2.0):
    """Send a normalized [0..1] 12-joint pose and hold it for hold_seconds."""
    targets = list(targets)
    if len(targets) == 1:
        targets *= 12
    if len(targets) != 12:
        raise ValueError("Expected 1 or 12 target values.")

    cmd = bus.make_position_command(targets)
    t0 = time.time()
    while time.time() - t0 < hold_seconds:
        bus.send_command(hand_id, cmd)
        time.sleep(0.02)  # ~50 Hz
