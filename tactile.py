# tactile.py
# Live tactile visualization for XHAND (left/right), 5 rows of (Left heatmap | bar | Right heatmap).
# Assumes:
#   - Left hand:  /dev/ttyUSB0, hand_id=1
#   - Right hand: /dev/ttyUSB1, hand_id=0
# Requires: matplotlib, numpy

from xhandlib.xhand import XHandControl, get_device_configs
import math
import time
import sys
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
    of magnitudes. If N==120, map to (10,12) then rotate 90° clockwise + flip 180° + mirror horizontally.
    If N is a perfect square, map to sqrt x sqrt then rotate + flip + mirror.
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
        # Reshape to (10,12) then rotate 90° clockwise + flip 180° + mirror horizontally
        grid = mags.reshape(10, 12)
        grid = np.rot90(grid, k=-1)  # k=-1 rotates 90° clockwise
        grid = np.flipud(grid)       # Flip 180° vertically (upside down)
        grid = np.fliplr(grid)       # Mirror horizontally (fix left/right mapping)
        return grid
    side = int(round(n ** 0.5))
    if side * side == n:
        grid = mags.reshape(side, side)
        grid = np.rot90(grid, k=-1)  # k=-1 rotates 90° clockwise
        grid = np.flipud(grid)       # Flip 180° vertically (upside down)
        grid = np.fliplr(grid)       # Mirror horizontally (fix left/right mapping)
        return grid
    return mags.reshape(1, n)


def combined_norm(calc_force):
    """Return sqrt(fx^2+fy^2+fz^2) from a calc_force object."""
    fx = float(getattr(calc_force, "fx", 0.0))
    fy = float(getattr(calc_force, "fy", 0.0))
    fz = float(getattr(calc_force, "fz", 0.0))
    return math.sqrt(fx * fx + fy * fy + fz * fz)


from xhandlib.xhand import XHandControl, get_device_configs
import math
import time
import atexit

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


# --------------------------
# Config
# --------------------------
BAUD = 3_000_000
UPDATE_HZ = 25.0  # plotting rate
FORCE_VMAX_MIN = 10.0  # lower bound for colormap vmax

print("Auto-detecting USB devices...")
try:
    left_config, right_config = get_device_configs()
    print(f"Left hand:  {left_config['serial_port']}")
    print(f"Right hand: {right_config['serial_port']}")
except Exception as e:
    print(f"Error detecting devices: {e}")
    raise SystemExit(f"Device detection failed: {e}")

# --------------------------
# Devices
# --------------------------
xhand_l = XHandControl(hand_id=0, position=0.1, mode=0)  # Start with any ID
xhand_r = XHandControl(hand_id=1, position=0.1, mode=0)  # Start with any ID

if not xhand_l.open_device(left_config):
    raise SystemExit("Failed to open LEFT hand")
if not xhand_r.open_device(right_config):
    xhand_l.close()
    raise SystemExit("Failed to open RIGHT hand")

print("✅ Both hands connected successfully!")

# Debug: Check which device is connected to which hand
print("Debug: Checking device connections...")
print(f"xhand_l connected to: {left_config['serial_port']}")
print(f"xhand_r connected to: {right_config['serial_port']}")

# Set proper IDs for the hands
print("Setting hand IDs...")
try:
    # Get the actual device IDs from each device
    print("Checking actual device IDs...")
    
    # Try to read state to see what ID each device actually has
    try:
        # Try ID 0 first for left device
        test_state = xhand_l.read_state(0, force_update=True)
        xhand_l._hand_id = 0
        print("Left device responds to ID 0")
    except:
        try:
            # Try ID 1 for left device
            test_state = xhand_l.read_state(1, force_update=True)
            xhand_l._hand_id = 1
            print("Left device responds to ID 1")
        except:
            print("Warning: Left device doesn't respond to ID 0 or 1")
    
    try:
        # Try ID 1 first for right device
        test_state = xhand_r.read_state(1, force_update=True)
        xhand_r._hand_id = 1
        print("Right device responds to ID 1")
    except:
        try:
            # Try ID 0 for right device
            test_state = xhand_r.read_state(0, force_update=True)
            xhand_r._hand_id = 0
            print("Right device responds to ID 0")
        except:
            print("Warning: Right device doesn't respond to ID 0 or 1")
    
    print("✅ Hand IDs set successfully!")
    print(f"Left hand (_hand_id): {xhand_l._hand_id}")
    print(f"Right hand (_hand_id): {xhand_r._hand_id}")
except Exception as e:
    print(f"Warning: Could not set hand IDs: {e}")
    print("Continuing with current IDs...")


# Ensure clean shutdown without try/except
def _close_all():
    xhand_l.close()
    xhand_r.close()
atexit.register(_close_all)

# Sensor reset functionality
def reset_all_sensors():
    """Reset all tactile sensors to reduce noise."""
    print("Resetting tactile sensors...")
    
    # Reset left hand sensors
    print("Resetting LEFT hand sensors...")
    left_results = xhand_l.reset_all_tactile_sensors(1)
    for sensor_name, success in left_results.items():
        status = "✅" if success else "❌"
        print(f"  {status} {sensor_name}")
    
    # Reset right hand sensors  
    print("Resetting RIGHT hand sensors...")
    right_results = xhand_r.reset_all_tactile_sensors(0)
    for sensor_name, success in right_results.items():
        status = "✅" if success else "❌"
        print(f"  {status} {sensor_name}")
    
    print("Sensor reset complete!")

# Add GUI buttons for sensor reset
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

# Create button axes
button_ax_reset = plt.axes([0.02, 0.02, 0.1, 0.04])  # [left, bottom, width, height]
button_ax_quit = plt.axes([0.13, 0.02, 0.1, 0.04])

# Create buttons
button_reset = Button(button_ax_reset, 'Reset Sensors')
button_quit = Button(button_ax_quit, 'Quit')

def reset_button_callback(event):
    reset_all_sensors()

def quit_button_callback(event):
    print("Exiting...")
    plt.close('all')
    sys.exit(0)

button_reset.on_clicked(reset_button_callback)
button_quit.on_clicked(quit_button_callback)


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

    # Initialize heatmaps with a 12x10 zero grid (rotated 90° clockwise from 10x12)
    base = np.zeros((12, 10))
    imL = axL.imshow(base, cmap="inferno", vmin=0, vmax=FORCE_VMAX_MIN, interpolation="nearest")
    imR = axR.imshow(base, cmap="inferno", vmin=0, vmax=FORCE_VMAX_MIN, interpolation="nearest")
    axL.set_xticks([]); axL.set_yticks([])
    axR.set_xticks([]); axR.set_yticks([])
    axL.set_title(f"Left Finger {row}")
    axR.set_title(f"Right Finger {row}")
    imL_list.append(imL)
    imR_list.append(imR)

    bars = axBar.bar(["L", "R"], [0.0, 0.0])
    axBar.set_ylim(0, 200)
    axBar.set_ylabel("F")
    bar_artists.append(bars)

# Add GUI buttons for sensor reset
from matplotlib.widgets import Button

# Create button axes (positioned at bottom of figure)
button_ax_reset = plt.axes([0.02, 0.02, 0.12, 0.05])  # [left, bottom, width, height]
button_ax_quit = plt.axes([0.15, 0.02, 0.12, 0.05])

# Create buttons
button_reset = Button(button_ax_reset, 'Reset Sensors')
button_quit = Button(button_ax_quit, 'Quit')

def reset_button_callback(event):
    reset_all_sensors()

def quit_button_callback(event):
    print("Exiting...")
    plt.close('all')
    sys.exit(0)

button_reset.on_clicked(reset_button_callback)
button_quit.on_clicked(quit_button_callback)

plt.ion()
plt.show(block=False)

print("=" * 60)
print("XHAND Tactile Visualization")
print("=" * 60)
print("Controls:")
print("  Reset Sensors button - Reset all tactile sensors (reduce noise)")
print("  Quit button - Exit application")
print("=" * 60)


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
            z = np.zeros((12, 10))  # Updated to match rotated grid
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
            z = np.zeros((12, 10))  # Updated to match rotated grid
            imR_list[row].set_data(z)
            imR_list[row].set_clim(vmin=0.0, vmax=FORCE_VMAX_MIN)
            bar_artists[row][1].set_height(0.0)

    plt.pause(period)
