# xhandlib/device.py
from __future__ import annotations
from typing import Iterable, List, Tuple, Optional
import time

# v1.1.7 import style
from xhand_controller import xhand_control as xc

DEFAULT_BAUD = 3_000_000

class XHandBus:
    """Thin convenience wrapper around the vendor SDK for RS-485 use."""

    def __init__(self, port: str, baud: int = DEFAULT_BAUD):
        self.port = port
        self.baud = baud
        self.dev: Optional[xc.XHandControl] = None

    def open(self) -> None:
        self.dev = xc.XHandControl()
        e = self.dev.open_serial(self.port, self.baud)
        if e.error_code != 0:
            raise RuntimeError(f"open_serial failed: {e.error_message} (port={self.port}, baud={self.baud})")

    def close(self) -> None:
        if self.dev:
            self.dev.close_device()
            self.dev = None

    # ---------- discovery ----------
    def list_ids(self) -> List[int]:
        return self.dev.list_hands_id()

    # ---------- command helpers ----------
    def make_position_command(self, pos: Iterable[float], kp=500, kd=50, ki=0, tor_max=80, mode=3):
        """
        Build a HandCommand_t for 12 joints with normalized positions in [0..1].
        Adjust gains as needed for your model.
        """
        cmd = xc.HandCommand_t()
        pos_list = list(pos)
        if len(pos_list) != 12:
            raise ValueError("Expected 12 target positions.")
        for jid in range(12):
            fc = cmd.finger_command[jid]
            fc.id = jid
            fc.kp = kp
            fc.ki = ki
            fc.kd = kd
            fc.position = float(pos_list[jid])
            fc.tor_max = tor_max
            fc.mode = mode  # 3 = position mode
        return cmd

    def send_command(self, hand_id: int, cmd) -> None:
        err = self.dev.send_command(hand_id, cmd)
        if err.error_code != 0:
            raise RuntimeError(f"send_command failed (id={hand_id}): {err.error_message}")

    # ---------- state ----------
    def read_state(self, hand_id: int, force_update: bool = False):
        e, state = self.dev.read_state(hand_id, force_update)
        if e.error_code != 0:
            raise RuntimeError(f"read_state failed (id={hand_id}): {e.error_message}")
        return state

    # ---------- info ----------
    def get_info(self, hand_id: int):
        e, info = self.dev.read_device_info(hand_id)
        if e.error_code != 0:
            raise RuntimeError(f"read_device_info failed (id={hand_id}): {e.error_message}")
        return info

    def get_hand_type(self, hand_id: int) -> str:
        e, t = self.dev.get_hand_type(hand_id)
        if e.error_code != 0:
            raise RuntimeError(f"get_hand_type failed (id={hand_id}): {e.error_message}")
        return t

    # ---------- tactile (best-effort across SDK variants) ----------
    def read_tactile_all(self, hand_id: int):
        """
        Attempts to read tactile sensors on the hand.
        Some firmware expose:
          - read_sensor(hand_id, sensor_id) for sensor_id 0x11..0x15
        If not available, we raise a clear error.
        """
        # prefer a direct function if present
        # known sensor ids: 0x11..0x15 (thumb..pinky)
        if hasattr(self.dev, "read_sensor"):
            readings = {}
            for sid in (0x11, 0x12, 0x13, 0x14, 0x15):
                e, data = self.dev.read_sensor(hand_id, sid)
                if e.error_code == 0:
                    readings[sid] = data
                else:
                    readings[sid] = f"ERR:{e.error_code}:{e.error_message}"
            return readings
        raise NotImplementedError("Tactile read not supported by this SDK build (no read_sensor).")
