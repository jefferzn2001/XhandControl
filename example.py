from xhandlib.xhand import XHandControl
import time
import math
import sys


if __name__ == "__main__":
    # Default hand ID is 0, finger position is 0.1, mode is 3
    xhand_exam = XHandControl(hand_id=0, position=0.1, mode=3)

    # Choose one communication method, currently supports EtherCAT and RS485
    # First of all, open device 
    device_identifier = {}

    while True:
        communication_choice = input("Choose communication method (enter '1' for EtherCAT, enter '2' for RS485): ").strip()
        if communication_choice == '1':
            device_identifier['protocol'] = 'EtherCAT'
            if xhand_exam.exam_open_device(device_identifier):
                break
            else:
                sys.exit(1)
        elif communication_choice == '2':
            device_identifier['protocol'] = 'RS485'
            # You can use exam_enumerate_devices('RS485') to read serial port list information, choose ttyUSB prefixed port
            # Get serial port list, choose ttyUSB*
            xhand_exam.exam_enumerate_devices('RS485')
            device_identifier["serial_port"] = '/dev/ttyUSB0'
            device_identifier['baud_rate'] = 3000000
            if xhand_exam.exam_open_device(device_identifier):
                break
            else:
                sys.exit(1)
        else:
            print("Invalid choice, please enter '1' or '2'\n")

    # List hand IDs
    # xhand_exam.exam_list_hands_id()
    # Read software SDK version
    xhand_exam.exam_get_sdk_version()
    # Read hardware SDK version
    xhand_exam.exam_read_version()
    # Read hand device information
    xhand_exam.exam_read_device_info()
    # Get hand left/right type
    xhand_exam.exam_get_hand_type()
    # Read hand serial number
    xhand_exam.exam_serial_number()
    # Read various hand states
    # {2, 5, 7, 9, 11} are fingertip sensors
    # if not use send_command func, let force_update value to True to force update state
    xhand_exam.exam_read_state(fingure_id=5, force_update=True)

    # Reset fingertip sensor
    # xhand_exam.exam_reset_sensor()
    # Set hand ID
    # xhand_exam.exam_set_hand_id(new_id=0)

    # Set hand name
    # xhand_exam.exam_set_hand_name(new_name="xhand")
    # Get hand name
    # xhand_exam.exam_get_hand_name()

    # ================================================================================
    # !! Warning: This function will send motion control command to device ！！
    # ================================================================================
    # Set hand mode（0: powerless, 3: position (default), 5: powerful）
    # xhand_exam.set_hand_mode(mode=3)

    # ================================================================================
    # !! Warning: This function will send motion control command to device ！！
    # ================================================================================
    # Send hand control command
    # xhand_exam.exam_send_command()

    # ================================================================================
    # !! Warning: This function will send motion control command to device ！！
    # ================================================================================
    # Send xhand preset action list  
    actions_list = {
        'fist': [11.85, 74.58, 40, -3.08, 106.02, 110, 109.75, 107.56, 107.66, 110, 109.1, 109.15],
        'palm': [0, 80.66, 33.2, 0.00, 5.11, 0, 6.53, 0, 6.76, 4.41, 10.13, 0],
        'v': [38.32, 90, 52.08, 6.21, 2.6, 0, 2.1, 0, 110, 110, 110, 109.23],
        'ok': [45.88, 41.54, 67.35, 2.22, 80.45, 70.82, 31.37, 10.39, 13.69, 16.88, 1.39, 10.55]
    }

    for action in actions_list:
        for i in range(12):
            xhand_exam._hand_command.finger_command[i].position = actions_list[action][i] * math.pi / 180
        xhand_exam.exam_send_command()
        time.sleep(1)

    # Close device
    xhand_exam.exam_close_device()