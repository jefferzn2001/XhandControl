from xhandlib.xhand import XHandControl, get_device_configs
import time
import math
import sys


from xhandlib.xhand import XHandControl, get_device_configs
import time
import math
import sys


if __name__ == "__main__":
    print("Auto-detecting USB devices...")
    try:
        left_config, right_config = get_device_configs()
        print(f"Left hand:  {left_config['serial_port']}")
        print(f"Right hand: {right_config['serial_port']}")
    except Exception as e:
        print(f"Error detecting devices: {e}")
        sys.exit(1)

    # Initialize hands - we'll set proper IDs after connection
    xhand_l = XHandControl(hand_id=0, position=0.1, mode=3)  # Start with any ID
    xhand_r = XHandControl(hand_id=1, position=0.1, mode=3)  # Start with any ID

    # Open both devices using auto-detected configs
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

    # Left hand: normal order (front to back)
    # Right hand: reverse order (back to front)
    actions_order_left = list(actions_list.keys())
    actions_order_right = list(reversed(actions_list.keys()))
    
    print(f"Left hand actions: {actions_order_left}")
    print(f"Right hand actions: {actions_order_right}")
    
    for i, action_left in enumerate(actions_order_left):
        action_right = actions_order_right[i]
        
        # Set positions for left hand
        for j in range(12):
            xhand_l._hand_command.finger_command[j].position = actions_list[action_left][j] * math.pi / 180
        
        # Set positions for right hand  
        for j in range(12):
            xhand_r._hand_command.finger_command[j].position = actions_list[action_right][j] * math.pi / 180
        
        print(f"Executing: Left={action_left}, Right={action_right}")
        xhand_l.send_command()
        xhand_r.send_command()
        time.sleep(1)

    # Close device
    xhand_l.close()
    xhand_r.close()
