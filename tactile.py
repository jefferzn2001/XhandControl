# tactile.py
# Live tactile visualization for XHAND (left/right), 5 rows of (Left heatmap | bar | Right heatmap).
# Assumes:
#   - Left hand:  /dev/ttyUSB0, hand_id=1
#   - Right hand: /dev/ttyUSB1, hand_id=0
# Requires: matplotlib, numpy

from xhandlib.xhand import XHandControl
import math
import time
import atexit

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


# --------------------------
# Config
# --------------------------
LEFT_SERIAL_PORT = "/dev/ttyUSB0"
RIGHT_SERIAL_PORT = "/dev/ttyUSB1"
LEFT_ID = 1
RIGHT_ID = 0
BAUD = 3_000_000
UPDATE_HZ = 25.0  # plotting rate
FORCE_VMAX_MIN = 10.0  # lower bound for colormap vmax


# --------------------------
# Helpers
# --------------------------
def forces_to_grid(raw_force_list):
    """
    Convert the SDK 'raw_force' (list of objects with .fx,.fy,.fz) into a 2D grid image
    of magnitudes. If N==120, map to (10,12). If N is a perfect square, map to sqrt x sqrt.
    Else, return 1 x N.
    """
    if not raw_force_list:
        return np.zeros((1, 1), dtype=float)

    mags = np.array(
        [math.sqrt(float(getattr(f, "fx", 0.0)) ** 2 +
                   float(getattr(f, "fy", 0.0)) ** 2 +
                   float(getattr(f, "fz", 0.0)) ** 2)
         for f in raw_force_list],
        dtype=float
    )

    n = mags.size
    if n == 120:
        return mags.reshape(10, 12)  # common layout for XHAND fingertip array
    side = int(round(n ** 0.5))
    if side * side == n:
        return mags.reshape(side, side)
    return mags.reshape(1, n)


def combined_norm(calc_force):
    """Return sqrt(fx^2+fy^2+fz^2) from a calc_force object."""
    fx = float(getattr(calc_force, "fx", 0.0))
    fy = float(getattr(calc_force, "fy", 0.0))
    fz = float(getattr(calc_force, "fz", 0.0))
    return math.sqrt(fx * fx + fy * fy + fz * fz)


# --------------------------
# Devices
# --------------------------
xhand_l = XHandControl(hand_id=LEFT_ID, position=0.1, mode=0)
xhand_r = XHandControl(hand_id=RIGHT_ID, position=0.1, mode=0)

left_ident = {"protocol": "RS485", "serial_port": LEFT_SERIAL_PORT, "baud_rate": BAUD}
right_ident = {"protocol": "RS485", "serial_port": RIGHT_SERIAL_PORT, "baud_rate": BAUD}

if not xhand_l.open_device(left_ident):
    raise SystemExit("Failed to open LEFT hand")
if not xhand_r.open_device(right_ident):
    xhand_l.close()
    raise SystemExit("Failed to open RIGHT hand")


# Ensure clean shutdown without try/except
def _close_all():
    xhand_l.close()
    xhand_r.close()
atexit.register(_close_all)


# --------------------------
# Figure / layout
# --------------------------
plt.close("all")
fig = plt.figure(figsize=(14, 8), dpi=100, constrained_layout=True)
gs = gridspec.GridSpec(
    5, 3, figure=fig,
    width_ratios=[3, 1.2, 3],
    height_ratios=[1, 1, 1, 1, 1]
)

imL_list = []
imR_list = []
bar_artists = []  # per row: (barL, barR)

for row in range(5):
    axL = fig.add_subplot(gs[row, 0])
    axBar = fig.add_subplot(gs[row, 1])
    axR = fig.add_subplot(gs[row, 2])

    # Initialize heatmaps with a 10x12 zero grid (matches 120 pts layout)
    base = np.zeros((10, 12))
    imL = axL.imshow(base, cmap="inferno", vmin=0, vmax=FORCE_VMAX_MIN, interpolation="nearest")
    imR = axR.imshow(base, cmap="inferno", vmin=0, vmax=FORCE_VMAX_MIN, interpolation="nearest")
    axL.set_xticks([]); axL.set_yticks([])
    axR.set_xticks([]); axR.set_yticks([])
    axL.set_title(f"Left Finger {row}")
    axR.set_title(f"Right Finger {row}")
    imL_list.append(imL)
    imR_list.append(imR)

    bars = axBar.bar(["L", "R"], [0.0, 0.0])
    axBar.set_ylim(0, 50)
    axBar.set_ylabel("|F| (N)")
    bar_artists.append(bars)

plt.ion()
plt.show(block=False)


# --------------------------
# Main loop
# --------------------------
period = 1.0 / UPDATE_HZ
while True:
    # Pull fresh frames (fingure_id=2 returns full-hand packet with sensor_data)
    s_l = xhand_l.read_state(xhand_l._hand_id, force_update=True)
    s_r = xhand_r.read_state(xhand_r._hand_id, force_update=True)

    sd_l = list(getattr(s_l, "sensor_data", []) or [])
    sd_r = list(getattr(s_r, "sensor_data", []) or [])

    for row in range(5):
        # ----- Left finger row -----
        if row < len(sd_l):
            calc_l = getattr(sd_l[row], "calc_force", None)
            raw_l = getattr(sd_l[row], "raw_force", [])
            grid_l = forces_to_grid(raw_l)
            imL_list[row].set_data(grid_l)
            vmax_l = max(FORCE_VMAX_MIN, float(np.percentile(grid_l, 99)))
            imL_list[row].set_clim(vmin=0.0, vmax=vmax_l)
            bar_artists[row][0].set_height(combined_norm(calc_l) if calc_l is not None else 0.0)
        else:
            z = np.zeros((10, 12))
            imL_list[row].set_data(z)
            imL_list[row].set_clim(vmin=0.0, vmax=FORCE_VMAX_MIN)
            bar_artists[row][0].set_height(0.0)

        # ----- Right finger row -----
        if row < len(sd_r):
            calc_r = getattr(sd_r[row], "calc_force", None)
            raw_r = getattr(sd_r[row], "raw_force", [])
            grid_r = forces_to_grid(raw_r)
            imR_list[row].set_data(grid_r)
            vmax_r = max(FORCE_VMAX_MIN, float(np.percentile(grid_r, 99)))
            imR_list[row].set_clim(vmin=0.0, vmax=vmax_r)
            bar_artists[row][1].set_height(combined_norm(calc_r) if calc_r is not None else 0.0)
        else:
            z = np.zeros((10, 12))
            imR_list[row].set_data(z)
            imR_list[row].set_clim(vmin=0.0, vmax=FORCE_VMAX_MIN)
            bar_artists[row][1].set_height(0.0)

    plt.pause(period)
