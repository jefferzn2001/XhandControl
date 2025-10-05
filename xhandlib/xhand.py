# xhandlib/device.py
from __future__ import annotations
from typing import Iterable, List, Tuple, Optional, Dict, Any
import time
import math
import sys

# v1.1.7 import style
from xhand_controller import xhand_control as xc

DEFAULT_BAUD = 3_000_000

class XHandBus:
    """
    Comprehensive wrapper around the vendor SDK for RS-485 and EtherCAT use.
    
    Integrates all functionality from the SDK example while maintaining
    backward compatibility with existing code.
    """

    def __init__(self, port: str = None, baud: int = DEFAULT_BAUD):
        """
        Initialize XHandBus.
        
        Args:
            port (str, optional): Serial port for RS485. If None, use for EtherCAT.
            baud (int): Baud rate for RS485 communication.
        """
        self.port = port
        self.baud = baud
        self.dev: Optional[xc.XHandControl] = None
        self.protocol = "RS485" if port else "EtherCAT"

    def open(self) -> None:
        """Open device connection (RS485 mode for backward compatibility)."""
        if not self.port:
            raise RuntimeError("Port must be specified for RS485 mode")
        self.dev = xc.XHandControl()
        e = self.dev.open_serial(self.port, self.baud)
        if e.error_code != 0:
            raise RuntimeError(f"open_serial failed: {e.error_message} (port={self.port}, baud={self.baud})")

    def close(self) -> None:
        """Close device connection."""
        if self.dev:
            self.dev.close_device()
            self.dev = None

    # ---------- device enumeration and connection ----------
    def enumerate_devices(self, protocol: str = "RS485") -> List[str]:
        """
        Enumerate device hardware input ports.
        
        Args:
            protocol (str): "RS485" or "EtherCAT"
            
        Returns:
            List[str]: Available device ports
        """
        return self.dev.enumerate_devices(protocol)

    def open_device(self, device_identifier: Dict[str, Any]) -> bool:
        """
        Open hand device with flexible protocol support.
        
        Args:
            device_identifier (dict): Device connection parameters
                For RS485: {"protocol": "RS485", "serial_port": str, "baud_rate": int}
                For EtherCAT: {"protocol": "EtherCAT"}
                
        Returns:
            bool: True if successful, False otherwise
        """
        self.dev = xc.XHandControl()
        
        if device_identifier["protocol"] == "RS485":
            device_identifier["baud_rate"] = int(device_identifier["baud_rate"])
            rsp = self.dev.open_serial(
                device_identifier["serial_port"],
                device_identifier["baud_rate"],
            )
        elif device_identifier["protocol"] == "EtherCAT":
            ether_cat = self.enumerate_devices("EtherCAT")
            if not ether_cat:
                print("enumerate_devices_ethercat get empty")
                return False
            rsp = self.dev.open_ethercat(ether_cat[0])
        else:
            raise ValueError(f"Unsupported protocol: {device_identifier['protocol']}")
            
        if rsp.error_code != 0:
            print(f"open device error: {rsp.error_message}. Please check connection")
            return False
        
        self.protocol = device_identifier["protocol"]
        if self.protocol == "RS485":
            self.port = device_identifier["serial_port"]
            self.baud = device_identifier["baud_rate"]
            
        return True

    # ---------- discovery ----------
    def list_ids(self) -> List[int]:
        """List all connected hand IDs."""
        return self.dev.list_hands_id()

    # ---------- device identification and info ----------
    def set_hand_id(self, old_id: int, new_id: int) -> bool:
        """
        Set hand ID.
        
        Args:
            old_id (int): Current hand ID
            new_id (int): New hand ID to set
            
        Returns:
            bool: True if successful
        """
        err_struct = self.dev.set_hand_id(old_id, new_id)
        return err_struct.error_code == 0

    def get_hand_name(self, hand_id: int) -> Tuple[bool, str]:
        """
        Get hand name.
        
        Args:
            hand_id (int): Hand ID
            
        Returns:
            Tuple[bool, str]: (success, hand_name)
        """
        error_struct, hand_name = self.dev.get_hand_name(hand_id)
        return error_struct.error_code == 0, hand_name

    def set_hand_name(self, hand_id: int, new_name: str) -> bool:
        """
        Set hand name.
        
        Args:
            hand_id (int): Hand ID
            new_name (str): New name for the hand
            
        Returns:
            bool: True if successful
        """
        err_struct = self.dev.set_hand_name(hand_id, new_name)
        return err_struct.error_code == 0

    def get_serial_number(self, hand_id: int) -> Tuple[bool, str]:
        """
        Get hand serial number.
        
        Args:
            hand_id (int): Hand ID
            
        Returns:
            Tuple[bool, str]: (success, serial_number)
        """
        error_struct, serial_number = self.dev.get_serial_number(hand_id)
        return error_struct.error_code == 0, serial_number

    def get_sdk_version(self) -> str:
        """
        Get software SDK version.
        
        Returns:
            str: SDK version string
        """
        return self.dev.get_sdk_version()

    def read_version(self, hand_id: int, joint_id: int = 0) -> Tuple[bool, str]:
        """
        Read hardware SDK version.
        
        Args:
            hand_id (int): Hand ID
            joint_id (int): Joint ID (default: 0)
            
        Returns:
            Tuple[bool, str]: (success, version)
        """
        error_struct, version = self.dev.read_version(hand_id, joint_id)
        return error_struct.error_code == 0, version

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
        """
        Send command to hand device.
        
        Args:
            hand_id (int): Hand ID
            cmd: HandCommand_t object
        """
        err = self.dev.send_command(hand_id, cmd)
        if err.error_code != 0:
            raise RuntimeError(f"send_command failed (id={hand_id}): {err.error_message}")

    def set_hand_mode(self, hand_id: int, mode: int, position: float = 0.5, 
                      kp: int = 120, ki: int = 0, kd: int = 0, tor_max: int = 380) -> bool:
        """
        Set hand activity mode.
        
        Args:
            hand_id (int): Hand ID
            mode (int): Hand mode (0: powerless, 3: position, 5: powerful)
            position (float): Default position (0.0-1.0)
            kp (int): Proportional gain
            ki (int): Integral gain  
            kd (int): Derivative gain
            tor_max (int): Maximum torque
            
        Returns:
            bool: True if successful
        """
        hand_mode = xc.HandCommand_t()
        for i in range(12):
            hand_mode.finger_command[i].id = i
            hand_mode.finger_command[i].kp = kp
            hand_mode.finger_command[i].ki = ki
            hand_mode.finger_command[i].kd = kd
            hand_mode.finger_command[i].position = position
            hand_mode.finger_command[i].tor_max = tor_max
            hand_mode.finger_command[i].mode = mode
        
        error_struct = self.dev.send_command(hand_id, hand_mode)
        return error_struct.error_code == 0

    # ---------- state ----------
    def read_state(self, hand_id: int, force_update: bool = False):
        """
        Read hand state.
        
        Args:
            hand_id (int): Hand ID
            force_update (bool): Force update state
            
        Returns:
            Hand state object
        """
        e, state = self.dev.read_state(hand_id, force_update)
        if e.error_code != 0:
            raise RuntimeError(f"read_state failed (id={hand_id}): {e.error_message}")
        return state

    def read_detailed_state(self, hand_id: int, finger_id: int = 2, force_update: bool = True) -> Dict[str, Any]:
        """
        Read detailed hand state with fingertip sensor information.
        
        Args:
            hand_id (int): Hand ID
            finger_id (int): Finger ID to read (default: 2, fingertips: {2, 5, 7, 9, 11})
            force_update (bool): Force update state
            
        Returns:
            Dict containing finger state and sensor data
        """
        error_struct, state = self.dev.read_state(hand_id, force_update)
        if error_struct.error_code != 0:
            raise RuntimeError(f"read_state failed (id={hand_id}): {error_struct.error_message}")
        
        finger_state = state.finger_state[finger_id]
        result = {
            "finger_id": finger_state.id,
            "temperature": finger_state.temperature,
            "temperature_masked": finger_state.temperature & 0xFF,
            "commboard_err": finger_state.commboard_err,
            "jonitboard_err": finger_state.jonitboard_err,
            "tipboard_err": finger_state.tipboard_err,
            "fingertip_state": {}
        }
        
        # Fingertip sensor state for fingertip sensors
        if finger_state.id in {2, 5, 7, 9, 11}:
            sensor_data = state.sensor_data[0]
            result["fingertip_state"] = {
                "calc_pressure": [
                    sensor_data.calc_force.fx,
                    sensor_data.calc_force.fy, 
                    sensor_data.calc_force.fz,
                ],
                "raw_pressure": [
                    [force.fx, force.fy, force.fz]
                    for force in state.sensor_data[0].raw_force
                ],
                "sensor_temperature": sensor_data.calc_temperature
            }
        
        return result

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

    # ---------- sensor operations ----------
    def reset_sensor(self, hand_id: int, sensor_id: int = 17) -> bool:
        """
        Reset fingertip sensor.
        
        Args:
            hand_id (int): Hand ID
            sensor_id (int): Sensor ID (default: 17)
            
        Returns:
            bool: True if successful
        """
        result = self.dev.reset_sensor(hand_id, sensor_id)
        return result.error_code == 0

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

    # ---------- preset actions ----------
    def get_preset_actions(self) -> Dict[str, List[float]]:
        """
        Get predefined hand poses in degrees.
        
        Returns:
            Dict[str, List[float]]: Dictionary of action names to joint angles in degrees
        """
        return {
            'fist': [11.85, 74.58, 40, -3.08, 106.02, 110, 109.75, 107.56, 107.66, 110, 109.1, 109.15],
            'palm': [0, 80.66, 33.2, 0.00, 5.11, 0, 6.53, 0, 6.76, 4.41, 10.13, 0],
            'v': [38.32, 90, 52.08, 6.21, 2.6, 0, 2.1, 0, 110, 110, 110, 109.23],
            'ok': [45.88, 41.54, 67.35, 2.22, 80.45, 70.82, 31.37, 10.39, 13.69, 16.88, 1.39, 10.55]
        }

    def send_preset_action(self, hand_id: int, action_name: str, 
                          kp: int = 100, ki: int = 0, kd: int = 0, tor_max: int = 300, mode: int = 3) -> bool:
        """
        Send a preset action to the hand.
        
        Args:
            hand_id (int): Hand ID
            action_name (str): Name of preset action ('fist', 'palm', 'v', 'ok')
            kp (int): Proportional gain
            ki (int): Integral gain
            kd (int): Derivative gain
            tor_max (int): Maximum torque
            mode (int): Control mode
            
        Returns:
            bool: True if successful
        """
        actions = self.get_preset_actions()
        if action_name not in actions:
            raise ValueError(f"Unknown action '{action_name}'. Available: {list(actions.keys())}")
        
        # Convert degrees to radians
        angles_deg = actions[action_name]
        angles_rad = [deg * math.pi / 180 for deg in angles_deg]
        
        # Create command
        hand_command = xc.HandCommand_t()
        for i in range(12):
            hand_command.finger_command[i].id = i
            hand_command.finger_command[i].kp = kp
            hand_command.finger_command[i].ki = ki
            hand_command.finger_command[i].kd = kd
            hand_command.finger_command[i].position = angles_rad[i]
            hand_command.finger_command[i].tor_max = tor_max
            hand_command.finger_command[i].mode = mode
        
        error_struct = self.dev.send_command(hand_id, hand_command)
        return error_struct.error_code == 0

    def send_preset_sequence(self, hand_id: int, actions: List[str] = None, 
                           hold_time: float = 1.0, **command_params) -> None:
        """
        Send a sequence of preset actions.
        
        Args:
            hand_id (int): Hand ID
            actions (List[str]): List of action names (default: all actions)
            hold_time (float): Time to hold each pose in seconds
            **command_params: Additional parameters for send_preset_action
        """
        if actions is None:
            actions = list(self.get_preset_actions().keys())
        
        for action in actions:
            print(f"Sending action: {action}")
            success = self.send_preset_action(hand_id, action, **command_params)
            if not success:
                print(f"Warning: Failed to send action '{action}'")
            time.sleep(hold_time)

    # ---------- convenience methods ----------
    def print_device_info(self, hand_id: int) -> None:
        """
        Print comprehensive device information.
        
        Args:
            hand_id (int): Hand ID
        """
        print("=" * 50)
        print(f"Device Information for Hand ID: {hand_id}")
        print("=" * 50)
        
        try:
            # SDK version
            print(f"Software SDK Version: {self.get_sdk_version()}")
            
            # Hardware version
            success, hw_version = self.read_version(hand_id)
            if success:
                print(f"Hardware SDK Version: {hw_version}")
            
            # Device info
            info = self.get_info(hand_id)
            print(f"Serial Number: {info.serial_number[0:16]}")
            print(f"Hand ID: {info.hand_id}")
            print(f"EV Hand: {info.ev_hand}")
            
            # Hand type
            hand_type = self.get_hand_type(hand_id)
            print(f"Hand Type: {hand_type}")
            
            # Hand name
            success, hand_name = self.get_hand_name(hand_id)
            if success:
                print(f"Hand Name: '{hand_name}'")
                
            # Serial number (alternative method)
            success, serial = self.get_serial_number(hand_id)
            if success:
                print(f"Serial Number (alt): {serial}")
                
        except Exception as e:
            print(f"Error reading device info: {e}")
        
        print("=" * 50)
