import sys
import argparse
from xhand_controller import xhand_control as xc  # v1.1.7 import style

DEFAULT_BAUD = 3_000_000
DEFAULT_PORT = "/dev/ttyUSB0"

def open_dev(port: str, baud: int):
    dev = xc.XHandControl()
    err = dev.open_serial(port, baud)
    if err.error_code != 0:
        raise SystemExit(f"[open_serial] {err.error_code}: {err.error_message} (port={port}, baud={baud})")
    return dev

def do_list(port, baud):
    dev = open_dev(port, baud)
    ids = dev.list_hands_id()
    print("Detected hand IDs:", ids)
    dev.close_device()

def do_set(port, baud, old_id, new_id):
    if new_id < 0 or new_id > 125:
        raise SystemExit("New ID must be in [0, 125].")
    dev = open_dev(port, baud)
    before = dev.list_hands_id()
    print("Before:", before)
    if old_id not in before:
        dev.close_device()
        raise SystemExit(f"Old ID {old_id} not found on bus {port}. Connect ONE hand and retry.")
    err = dev.set_hand_id(old_id, new_id)
    print(f"set_hand_id({old_id}->{new_id}): {err.error_code} {err.error_message or ''}".strip())
    after = dev.list_hands_id()
    print("After:", after)
    dev.close_device()
    if err.error_code == 0:
        print("✔ Done. POWER-CYCLE the hand now to persist the new ID.")
    else:
        print("✖ ID change failed; do not power-cycle yet. Fix the issue and retry.")

def do_set_auto(port, baud, target_id):
    dev = open_dev(port, baud)
    ids = dev.list_hands_id()
    print("Detected hand IDs:", ids)
    if len(ids) != 1:
        dev.close_device()
        raise SystemExit("Please connect exactly ONE hand to set its ID automatically.")
    current = ids[0]
    if current == target_id:
        print(f"ID already {target_id}. Nothing to do.")
        dev.close_device()
        return
    err = dev.set_hand_id(current, target_id)
    print(f"set_hand_id({current}->{target_id}): {err.error_code} {err.error_message or ''}".strip())
    after = dev.list_hands_id()
    print("After:", after)
    dev.close_device()
    if err.error_code == 0:
        print("✔ Done. POWER-CYCLE the hand now to persist the new ID.")
    else:
        print("✖ ID change failed; do not power-cycle yet. Fix the issue and retry.")

def main():
    p = argparse.ArgumentParser(description="XHAND RS-485 ID tool (设备ID).")
    p.add_argument("--port", default=DEFAULT_PORT, help="Serial port (e.g., /dev/ttyUSB0 or /dev/serial/by-id/...)")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Baud rate (default 3000000)")
    p.add_argument("--list", action="store_true", help="List detected hands")
    p.add_argument("--old", type=int, help="Current ID (manual mode)")
    p.add_argument("--new", type=int, help="New ID [0..125] (manual mode)")
    p.add_argument("--target", type=int, help="Auto mode: set the sole connected hand to this ID")

    args = p.parse_args()

    if args.list:
        do_list(args.port, args.baud)
    elif args.target is not None:
        do_set_auto(args.port, args.baud, args.target)
    elif args.old is not None and args.new is not None:
        do_set(args.port, args.baud, args.old, args.new)
    else:
        p.print_help()
        sys.exit(1)

if __name__ == "__main__":
    main()