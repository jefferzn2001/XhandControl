from xhandlib.xhand import XHandControl
import sys
import math
import tkinter as tk

# -------------------------------
# Official joint limits (degrees)
# J0..J11
# 0:[0,90], 1:[-60,90], 2:[0,90], 3:[-5,17], 4..11:[0,110]
# -------------------------------
DEG_LIMITS = [
    (0.0, 90.0),    # J0
    (-60.0, 90.0),  # J1
    (0.0, 90.0),    # J2
    (-5.0, 17.0),   # J3
] + [(0.0, 110.0)] * 8  # J4..J11

def clamp_deg(j_index: int, value_deg: float) -> float:
    """Clamp a degree value to the official joint limits for the given joint index."""
    if 0 <= j_index < len(DEG_LIMITS):
        lo, hi = DEG_LIMITS[j_index]
        if value_deg < lo:
            return lo
        if value_deg > hi:
            return hi
    return value_deg


if __name__ == "__main__":
    # Unified naming and mapping (same as tactile.py and demo.py)
    LEFT_SERIAL_PORT = "/dev/ttyUSB1"
    RIGHT_SERIAL_PORT = "/dev/ttyUSB0"
    LEFT_ID = 2
    RIGHT_ID = 1

    # Keep mode fixed to 0 for encoder
    xhand_l = XHandControl(hand_id=LEFT_ID, position=0.1, mode=0)
    xhand_r = XHandControl(hand_id=RIGHT_ID, position=0.1, mode=0)

    # Open devices over RS485
    left_device_identifier = {
        "protocol": "RS485",
        "serial_port": LEFT_SERIAL_PORT,
        "baud_rate": 3000000,
    }
    right_device_identifier = {
        "protocol": "RS485",
        "serial_port": RIGHT_SERIAL_PORT,
        "baud_rate": 3000000,
    }

    if not xhand_l.open_device(left_device_identifier):
        sys.exit(1)
    if not xhand_r.open_device(right_device_identifier):
        xhand_l.close()
        sys.exit(1)


    # Build a simple live GUI for encoder angles
    root = tk.Tk()
    root.title("XHAND Encoder Monitor (deg)")

    header_l = tk.Label(root, text="Left Hand (deg)", font=("Arial", 18, "bold"))
    header_r = tk.Label(root, text="Right Hand (deg)", font=("Arial", 18, "bold"))
    header_l.grid(row=0, column=0, columnspan=12, pady=(12, 8))
    header_r.grid(row=2, column=0, columnspan=12, pady=(12, 8))

    labels_l = []
    labels_r = []
    for i in range(12):
        lbl = tk.Label(root, text=f"J{i}: --.-°", width=10, anchor="center", font=("Consolas", 16))
        lbl.grid(row=1, column=i, padx=4, pady=2)
        labels_l.append(lbl)
    for i in range(12):
        lbl = tk.Label(root, text=f"J{i}: --.-°", width=10, anchor="center", font=("Consolas", 16))
        lbl.grid(row=3, column=i, padx=4, pady=2)
        labels_r.append(lbl)

    status_var = tk.StringVar(value="Streaming... Press Ctrl+C or close window to stop.")
    status = tk.Label(root, textvariable=status_var, fg="gray", font=("Arial", 12))
    status.grid(row=4, column=0, columnspan=12, pady=(6, 8))

    def update_loop():
        try:
            # force_update=True to ensure fresh data when not sending commands
            state_l = xhand_l.read_state(xhand_l._hand_id, force_update=True)
            state_r = xhand_r.read_state(xhand_r._hand_id, force_update=True)

            finger_states_l = getattr(state_l, "finger_state", [])
            finger_states_r = getattr(state_r, "finger_state", [])

            # Convert to degrees; mapping note:
            # Here we intentionally swap: Left row shows xhand_r, Right row shows xhand_l
            for i in range(12):
                deg_row_left = 0.0
                deg_row_right = 0.0

                if i < len(finger_states_r):
                    fs = finger_states_r[i]
                    val = getattr(fs, "position", 0.0)
                    try:
                        deg_row_left = math.degrees(float(val))
                    except Exception:
                        deg_row_left = 0.0

                if i < len(finger_states_l):
                    fs = finger_states_l[i]
                    val = getattr(fs, "position", 0.0)
                    try:
                        deg_row_right = math.degrees(float(val))
                    except Exception:
                        deg_row_right = 0.0

                # Clamp to official joint limits (per-joint)
                deg_row_left = clamp_deg(i, deg_row_left)
                deg_row_right = clamp_deg(i, deg_row_right)

                labels_l[i].configure(text=f"J{i}: {deg_row_left:6.1f}°")
                labels_r[i].configure(text=f"J{i}: {deg_row_right:6.1f}°")

            root.after(50, update_loop)
        except Exception as e:
            status_var.set(f"Error: {e}")

    def on_close():
        try:
            xhand_l.close()
        finally:
            try:
                xhand_r.close()
            finally:
                root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.after(50, update_loop)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        on_close()
    finally:
        on_close()
