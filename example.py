#!/usr/bin/env python3
"""
example.py — press ENTER to step through 5 poses defined in DEGREES.
b = back | r = repeat | q = quit

Assumptions:
  • RS-485 @ 3,000,000 baud
  • Left hand ID = 1, Right hand ID = 2
  • 12 joints per hand, UI shows max angles approx:
      J0=105°, J1=100°, J2=100°, J3=10°, J4..J11=110°
Edit MAX_DEG below if your model differs.
"""

import sys, time
from xhandlib.device import XHandBus  # wrapper for vendor SDK

# ======= EDIT THESE IF NEEDED =======
PORT = "/dev/ttyUSB0"
BAUD = 3_000_000

LEFT_ID  = 1
RIGHT_ID = 2

HOLD_SEC    = 1.0       # how long to keep resending each pose
SEND_PERIOD = 0.05      # resend rate (~20 Hz is robust on RS-485)

# Control gains (gentle defaults)
GAINS = dict(kp=300, ki=0, kd=30, tor_max=60, mode=3)

# Per-joint max degrees (UI reference). Change if your device differs.
MAX_DEG = [105, 100, 100, 10] + [110]*8
# ====================================

# ──────────────────────────────────────────────────────────────
# Joint index → UI meaning (from your diagram)
#  J0 : Thumb base / CMC ab/ad (0..105°)
#  J1 : Thumb MCP flex
#  J2 : Thumb IP flex
#  J3 : Index yaw / ab-ad (±10° around 0)
#  J4 : Index MCP flex
#  J5 : Index DIP/PIP flex
#  J6 : Middle MCP flex
#  J7 : Middle DIP/PIP flex
#  J8 : Ring MCP flex
#  J9 : Ring DIP/PIP flex
#  J10: Little MCP flex
#  J11: Little DIP/PIP flex
#
# Max degrees used for normalization in code:
#   MAX_DEG = [105, 100, 100, 10] + [110]*8
# ──────────────────────────────────────────────────────────────

POSES_DEG = [
    # Pose 1 — Open hand / neutral
    #   Thumb relaxed, all fingers straight, index yaw centered
    [  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0 ],

    # Pose 2 — Power grasp / fist
    #   All fingers curled, thumb folded across, index yaw inward
    [ 20,  80,  70, -10,  90,  90,  90,  90,  90,  90,  90,  90 ],

    # Pose 3 — Pointing (index extended)
    #   Index mostly straight, other fingers curled, thumb partial flex
    [ 10,  40,  30,  -5,  10,   5,  100,  100,  100,  100,  100,  100 ],

    # Pose 4 — Pinch (thumb–index tip)
    #   Thumb folded in, index bent, others mostly open
    [ 15,  70,  70,  -8,  75,  90,  10,  10,  10,  10,  10,  10 ],

    # Pose 5 — “OK” sign (thumb–index circle, others half-flexed)
    #   Thumb opposes index, middle/ring/little partially flexed
    [ 20,  90,  90, -10,  50,  85,  40,  40,  40,  40,  40,  40 ],
]
# ──────────────────────────────────────────────────────────────


def deg_to_norm(deg_list):
    """Convert degrees→normalized [0..1] using MAX_DEG (clamped)."""
    if len(deg_list) != 12:
        raise ValueError("Each pose must have exactly 12 degree values (J0..J11).")
    out = []
    for d, m in zip(deg_list, MAX_DEG):
        m = float(m if m else 110.0)
        out.append(max(0.0, min(1.0, float(d)/m)))
    return out

def send_pose(bus: XHandBus, hand_id: int, targets_norm):
    """Send one pose with a light CRC-retry; re-send for HOLD_SEC."""
    cmd = bus.make_position_command(targets_norm, **GAINS)
    t_end = time.time() + HOLD_SEC
    while time.time() < t_end:
        try:
            bus.send_command(hand_id, cmd)
        except RuntimeError as e:
            if "CRC" in str(e) or "Communication" in str(e):
                print(f"[warn] CRC error on id={hand_id}; reopening…")
                bus.close(); time.sleep(0.1); bus.open()
                continue
            raise
        time.sleep(SEND_PERIOD)

def main():
    # Normalize poses once
    if not POSES_DEG or any(len(p)!=12 for p in POSES_DEG):
        raise SystemExit("Please define five 12-value degree poses in POSES_DEG.")
    poses_norm = [deg_to_norm(p) for p in POSES_DEG]

    print(f"[open] {PORT} @ {BAUD}")
    bus = XHandBus(PORT, BAUD)
    bus.open()
    try:
        ids = bus.list_ids()
        print("[ids]", ids)

        idx = 0
        print("\nPress ENTER = next | b = back | r = repeat | q = quit")
        while True:
            pose_norm = poses_norm[idx]
            pose_deg  = POSES_DEG[idx]
            print(f"\n[pose {idx+1}/{len(poses_norm)}] (deg) {pose_deg}")
            if LEFT_ID in ids:
                send_pose(bus, LEFT_ID, pose_norm)
            if RIGHT_ID in ids:
                send_pose(bus, RIGHT_ID, pose_norm)
            print("[done] pose sent. Await input…", end="", flush=True)

            cmd = sys.stdin.readline().strip().lower()
            if cmd == "q":
                print("\n[quit]"); break
            elif cmd == "b":
                idx = (idx - 1) % len(poses_norm)
            elif cmd == "r":
                continue
            else:
                idx = (idx + 1) % len(poses_norm)
    finally:
        bus.close()

if __name__ == "__main__":
    # Tip: after powering hands, wait ~10 s for auto-homing.
    main()
