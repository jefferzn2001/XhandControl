from xhandlib.xhand import XHandControl, get_device_configs
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
    print("Auto-detecting USB devices...")
    try:
        left_config, right_config = get_device_configs()
        print(f"Left hand:  {left_config['serial_port']}")
        print(f"Right hand: {right_config['serial_port']}")
    except Exception as e:
        print(f"Error detecting devices: {e}")
        sys.exit(1)

    # Keep mode fixed to 0 for encoder
    xhand_l = XHandControl(hand_id=0, position=0.1, mode=0)  # Start with any ID
    xhand_r = XHandControl(hand_id=1, position=0.1, mode=0)  # Start with any ID

    if not xhand_l.open_device(left_config):
        print("Failed to connect to left hand")
        sys.exit(1)
    if not xhand_r.open_device(right_config):
        print("Failed to connect to right hand")
        xhand_l.close()
        sys.exit(1)

    print("✅ Both hands connected successfully!")
    
    # Set proper IDs for the hands
    print("Setting hand IDs...")
    try:
        left_id_found = False
        right_id_found = False
        
        try:
            test_state = xhand_l.read_state(0, force_update=True)
            xhand_l._hand_id = 0
            left_id_found = True
        except:
            try:
                test_state = xhand_l.read_state(1, force_update=True)
                xhand_l._hand_id = 1
                left_id_found = True
            except:
                pass
        
        try:
            test_state = xhand_r.read_state(1, force_update=True)
            xhand_r._hand_id = 1
            right_id_found = True
        except:
            try:
                test_state = xhand_r.read_state(0, force_update=True)
                xhand_r._hand_id = 0
                right_id_found = True
            except:
                pass
        
        if not left_id_found or not right_id_found:
            xhand_l._hand_id = 1   # Left device (FTA7NRXW) has ID 1
            xhand_r._hand_id = 0  # Right device (FTA6HQ26) has ID 0
        
        print(f"✓ Hand IDs set - Left: {xhand_l._hand_id}, Right: {xhand_r._hand_id}")
    except Exception as e:
        print(f"Warning: Could not set hand IDs: {e}")


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

            # Convert to degrees; correct mapping:
            # Left row shows xhand_l (left device), Right row shows xhand_r (right device)
            for i in range(12):
                deg_row_left = 0.0
                deg_row_right = 0.0

                if i < len(finger_states_l):
                    fs = finger_states_l[i]
                    val = getattr(fs, "position", 0.0)
                    try:
                        deg_row_left = math.degrees(float(val))
                    except Exception:
                        deg_row_left = 0.0

                if i < len(finger_states_r):
                    fs = finger_states_r[i]
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