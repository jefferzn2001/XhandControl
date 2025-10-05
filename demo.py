from xhandlib.xhand import XHandControl
import time
import math
import sys


if __name__ == "__main__":
    # Unified naming and mapping (same as encoder.py and tactile.py)
    LEFT_SERIAL_PORT = "/dev/ttyUSB1"   # Left hand on USB1
    RIGHT_SERIAL_PORT = "/dev/ttyUSB0"  # Right hand on USB0
    LEFT_ID = 2
    RIGHT_ID = 1

    # Initialize left and right hands; keep mode fixed to 3 for demo
    xhand_l = XHandControl(hand_id=LEFT_ID, position=0.1, mode=3)
    xhand_r = XHandControl(hand_id=RIGHT_ID, position=0.1, mode=3)

    # Open devices over RS485
    left_device_identifier = {
        'protocol': 'RS485',
        'serial_port': LEFT_SERIAL_PORT,
        'baud_rate': 3000000,
    }
    right_device_identifier = {
        'protocol': 'RS485',
        'serial_port': RIGHT_SERIAL_PORT,
        'baud_rate': 3000000,
    }

    # Open both devices
    if not xhand_l.open_device(left_device_identifier):
        sys.exit(1)
    if not xhand_r.open_device(right_device_identifier):
        xhand_l.close()
        sys.exit(1)


    # List hand IDs (optional)
    # xhand_l.list_hands_id()
    # xhand_r.list_hands_id()
    # Read software SDK version
    # xhand_l.get_sdk_version()
    # Read hardware SDK version
    # xhand_l.read_version(xhand_l._hand_id)
    # Read hand device information
    # xhand_l.get_info(xhand_l._hand_id)
    # Get hand left/right type
    xhand_l.get_hand_type(xhand_l._hand_id)
    # Read hand serial number
    xhand_l.get_serial_number(xhand_l._hand_id)
    # Read various hand states
    # {2, 5, 7, 9, 11} are fingertip sensors
    # if not use send_command func, let force_update value to True to force update state
    xhand_l.read_state(xhand_l._hand_id, force_update=True)

    # Reset fingertip sensor
    # xhand.reset_sensor()
    # Set hand ID
    # xhand.set_hand_id(new_id=0)

    # Set hand name
    # xhand.set_hand_name(new_name="xhand")
    # Get hand name
    # xhand.get_hand_name()

    # ================================================================================
    # !! Warning: This function will send motion control command to device ！！
    # ================================================================================
    # Set hand mode（0: powerless, 3: position (default), 5: powerful）
    # xhand.set_hand_mode(mode=3)

    # ================================================================================
    # !! Warning: This function will send motion control command to device ！！
    # ================================================================================
    # Send hand control command
    # xhand.send_command()

    # ================================================================================
    # !! Warning: This function will send motion control command to device ！！
    # ================================================================================
    # Send xhand preset action list  
    actions_list = {
        'fist': [11.85, 74.58, 40, -3.08, 106.02, 110, 109.75, 107.56, 107.66, 110, 109.1, 109.15],
        'palm': [0, 80.66, 33.2, 0.00, 5.11, 0, 6.53, 0, 6.76, 4.41, 10.13, 0],
        'v': [38.32, 90, 52.08, 6.21, 2.6, 0, 2.1, 0, 110, 110, 110, 109.23],
        'ok': [45.88, 41.54, 67.35, 2.22, 80.45, 70.82, 31.37, 10.39, 13.69, 16.88, 1.39, 10.55],
        'point': [20,63,97,2,-0.7,0.2,109,107,103,110,97.5,110],
        'rock': [19, 54, 102, 2, 5.2, 0.2, 110, 95, 109, 101.7, 10.3, 10],
        'palm2': [0, 80.66, 33.2, 0.00, 5.11, 0, 6.53, 0, 6.76, 4.41, 10.13, 0],
    }

    for action in actions_list:
        for i in range(12):
            xhand_l._hand_command.finger_command[i].position = actions_list[action][i] * math.pi / 180
            xhand_r._hand_command.finger_command[i].position = actions_list[action][i] * math.pi / 180
        # Warning: The following will move both hands
        xhand_l.send_command()
        xhand_r.send_command()
        time.sleep(1)

    # Close device
    xhand_l.close()
    xhand_r.close()