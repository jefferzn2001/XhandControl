"""
xhandlib/isaacsim_utils.py

Utilities for controlling XHAND in Isaac Sim with the same interface as real hardware.
Provides unified command format and conversion functions.
"""

import math
from typing import List, Dict, Tuple
import numpy as np


# Preset hand poses in degrees (hardware order: J0-J11)
# Hardware order (from demo.py):
# J0: Thumb CMC/yaw, J1: Thumb MCP, J2: Thumb IP
# J3: Index yaw, J4: Index MCP, J5: Index PIP
# J6: Middle MCP, J7: Middle PIP
# J8: Ring MCP, J9: Ring PIP
# J10: Pinky MCP, J11: Pinky PIP
PRESET_POSES = {
    'palm': [0, 80.66, 33.2, 0.00, 5.11, 0, 6.53, 0, 6.76, 4.41, 10.13, 0],
    'fist': [11.85, 74.58, 40, -3.08, 106.02, 110, 109.75, 107.56, 107.66, 105, 90.0, 109.15],
    'v': [38.32, 90, 52.08, 6.21, 2.6, 0, 2.1, 0, 110, 110, 110, 109.23],
    'ok': [45.88, 41.54, 67.35, 2.22, 80.45, 70.82, 31.37, 10.39, 13.69, 16.88, 1.39, 10.55]
}

# ═══════════════════════════════════════════════════════════════════════════
# CRITICAL: DEFINITIVE JOINT MAPPING (verified via GUI testing)
# ═══════════════════════════════════════════════════════════════════════════
#
# URDF joint order (alphabetical order in Isaac Sim scene):
# URDF[0]=thumb_bend, URDF[1]=thumb_rota1, URDF[2]=thumb_rota2,
# URDF[3]=index_bend, URDF[4]=index_j1, URDF[5]=index_j2,
# URDF[6]=mid_j1, URDF[7]=mid_j2, URDF[8]=ring_j1, URDF[9]=ring_j2,
# URDF[10]=pinky_j1, URDF[11]=pinky_j2
#
# Hardware joint order (from demo.py, encoder.py):
# HW J0: Thumb CMC/yaw
# HW J1: Thumb MCP
# HW J2: Thumb IP
# HW J3: Index yaw
# HW J4: Index MCP
# HW J5: Index PIP
# HW J6: Middle MCP
# HW J7: Middle PIP
# HW J8: Ring MCP
# HW J9: Ring PIP
# HW J10: Pinky MCP
# HW J11: Pinky PIP
#
# Verified mapping from GUI testing:
# URDF Slider 0 → HW J3  (Index yaw)
# URDF Slider 1 → HW J6  (Middle MCP)
# URDF Slider 2 → HW J10 (Pinky MCP)
# URDF Slider 3 → HW J8  (Ring MCP)
# URDF Slider 4 → HW J0  (Thumb yaw)
# URDF Slider 5 → HW J4  (Index MCP)
# URDF Slider 6 → HW J7  (Middle PIP)
# URDF Slider 7 → HW J11 (Pinky PIP)
# URDF Slider 8 → HW J9  (Ring PIP)
# URDF Slider 9 → HW J1  (Thumb MCP)
# URDF Slider 10 → HW J5 (Index PIP)
# URDF Slider 11 → HW J2 (Thumb IP)
#
# ═══════════════════════════════════════════════════════════════════════════

# Map: URDF joint index → Hardware joint index
URDF_TO_HARDWARE_MAPPING = [
    3,   # URDF J0 ← HW J3 (Index yaw)
    6,   # URDF J1 ← HW J6 (Middle MCP)
    10,  # URDF J2 ← HW J10 (Pinky MCP)
    8,   # URDF J3 ← HW J8 (Ring MCP)
    0,   # URDF J4 ← HW J0 (Thumb yaw)
    4,   # URDF J5 ← HW J4 (Index MCP)
    7,   # URDF J6 ← HW J7 (Middle PIP)
    11,  # URDF J7 ← HW J11 (Pinky PIP)
    9,   # URDF J8 ← HW J9 (Ring PIP)
    1,   # URDF J9 ← HW J1 (Thumb MCP)
    5,   # URDF J10 ← HW J5 (Index PIP)
    2,   # URDF J11 ← HW J2 (Thumb IP)
]

# Reverse mapping: Hardware joint index → URDF joint index
# When you have a hardware command [HW J0, HW J1, ..., HW J11],
# this tells you where to put each value in the URDF array
HARDWARE_TO_URDF_MAPPING = [0] * 12
for urdf_idx, hw_idx in enumerate(URDF_TO_HARDWARE_MAPPING):
    HARDWARE_TO_URDF_MAPPING[hw_idx] = urdf_idx

print(f"[DEBUG] Hardware → URDF mapping: {HARDWARE_TO_URDF_MAPPING}")
# This should output: [4, 9, 11, 0, 5, 10, 1, 6, 3, 8, 2, 7]

# Joint limits in degrees (from your hardware)
# J0=105°, J1=100°, J2=100°, J3=10°, J4..J11=110°
MAX_DEG = [105, 100, 100, 10] + [110] * 8


def hardware_to_urdf_order(hardware_values: List[float]) -> List[float]:
    """
    Remap hardware joint order to URDF joint order.
    
    Args:
        hardware_values (List[float]): 12 values in hardware order (J0-J11)
        
    Returns:
        List[float]: 12 values in URDF order
    """
    if len(hardware_values) != 12:
        raise ValueError(f"Expected 12 values, got {len(hardware_values)}")
    
    urdf_values = [0.0] * 12
    for hw_idx, value in enumerate(hardware_values):
        urdf_idx = HARDWARE_TO_URDF_MAPPING[hw_idx]
        urdf_values[urdf_idx] = value
    
    return urdf_values


def degrees_to_radians(degrees: List[float], clamp_to_urdf_limits: bool = True, for_isaac_sim: bool = True) -> List[float]:
    """
    Convert joint angles from degrees to radians with proper URDF remapping.
    
    Args:
        degrees (List[float]): 12 joint angles in HARDWARE order (J0-J11)
        clamp_to_urdf_limits (bool): If True, clamp to URDF joint limits
        for_isaac_sim (bool): If True, remap to URDF joint order
        
    Returns:
        List[float]: 12 joint angles in radians, in URDF order
    """
    if len(degrees) != 12:
        raise ValueError(f"Expected 12 joint angles, got {len(degrees)}")
    
    # Convert to radians first (still in hardware order)
    radians_hw = [deg * math.pi / 180.0 for deg in degrees]
    
    # Remap to URDF order if needed
    if for_isaac_sim:
        radians = hardware_to_urdf_order(radians_hw)
    else:
        radians = radians_hw
    
    # URDF joint limits in radians (in URDF order)
    urdf_limits = [
        (0.0, 1.832),      # URDF J0: thumb_bend
        (-0.698, 1.745),   # URDF J1: thumb_rota1
        (0.0, 1.745),      # URDF J2: thumb_rota2
        (-0.174, 0.174),   # URDF J3: index_bend
        (0.0, 1.919),      # URDF J4: index_j1
        (0.0, 1.919),      # URDF J5: index_j2
        (0.0, 1.919),      # URDF J6: mid_j1
        (0.0, 1.919),      # URDF J7: mid_j2
        (0.0, 1.919),      # URDF J8: ring_j1
        (0.0, 1.919),      # URDF J9: ring_j2
        (0.0, 1.919),      # URDF J10: pinky_j1
        (0.0, 1.919),      # URDF J11: pinky_j2
    ]
    
    # Clamp to URDF limits
    if clamp_to_urdf_limits:
        clamped = []
        for rad, (lower, upper) in zip(radians, urdf_limits):
            epsilon = 0.001
            clamped_val = max(lower + epsilon, min(upper - epsilon, rad))
            clamped.append(clamped_val)
        return clamped
    
    return radians


def radians_to_degrees(radians: List[float]) -> List[float]:
    """
    Convert joint angles from radians to degrees.
    
    Args:
        radians (List[float]): 12 joint angles in radians
        
    Returns:
        List[float]: 12 joint angles in degrees
    """
    if len(radians) != 12:
        raise ValueError(f"Expected 12 joint angles, got {len(radians)}")
    return [rad * 180.0 / math.pi for rad in radians]


def degrees_to_normalized(degrees: List[float]) -> List[float]:
    """
    Convert degrees to normalized [0..1] using MAX_DEG limits.
    
    Args:
        degrees (List[float]): 12 joint angles in degrees
        
    Returns:
        List[float]: 12 normalized joint positions [0..1]
    """
    if len(degrees) != 12:
        raise ValueError(f"Expected 12 joint angles, got {len(degrees)}")
    
    normalized = []
    for deg, max_deg in zip(degrees, MAX_DEG):
        # Clamp and normalize
        norm = max(0.0, min(1.0, float(deg) / float(max_deg)))
        normalized.append(norm)
    return normalized


def normalized_to_degrees(normalized: List[float]) -> List[float]:
    """
    Convert normalized [0..1] to degrees using MAX_DEG limits.
    
    Args:
        normalized (List[float]): 12 normalized joint positions [0..1]
        
    Returns:
        List[float]: 12 joint angles in degrees
    """
    if len(normalized) != 12:
        raise ValueError(f"Expected 12 joint positions, got {len(normalized)}")
    
    return [norm * max_deg for norm, max_deg in zip(normalized, MAX_DEG)]


def get_preset_pose_radians(pose_name: str) -> List[float]:
    """
    Get a preset pose in radians for Isaac Sim.
    
    Args:
        pose_name (str): Name of preset ('palm', 'fist', 'v', 'ok')
        
    Returns:
        List[float]: 12 joint angles in radians
    """
    if pose_name not in PRESET_POSES:
        raise ValueError(f"Unknown pose '{pose_name}'. Available: {list(PRESET_POSES.keys())}")
    
    degrees = PRESET_POSES[pose_name]
    return degrees_to_radians(degrees)


def get_preset_pose_degrees(pose_name: str) -> List[float]:
    """
    Get a preset pose in degrees for real hardware.
    
    Args:
        pose_name (str): Name of preset ('palm', 'fist', 'v', 'ok')
        
    Returns:
        List[float]: 12 joint angles in degrees
    """
    if pose_name not in PRESET_POSES:
        raise ValueError(f"Unknown pose '{pose_name}'. Available: {list(PRESET_POSES.keys())}")
    
    return PRESET_POSES[pose_name].copy()


def create_joint_position_dict(joint_angles_rad: List[float], hand_type: str = "left") -> Dict[str, float]:
    """
    Create a dictionary mapping joint names to positions for Isaac Lab Articulation.
    
    Args:
        joint_angles_rad (List[float]): 12 joint angles in radians
        hand_type (str): "left" or "right"
        
    Returns:
        Dict[str, float]: Joint name to position mapping
    """
    if len(joint_angles_rad) != 12:
        raise ValueError(f"Expected 12 joint angles, got {len(joint_angles_rad)}")
    
    prefix = f"{hand_type}_hand"
    
    # Map based on URDF joint structure
    # J0: thumb_bend, J1: thumb_rota_joint1, J2: thumb_rota_joint2
    # J3: index_bend, J4: index_rota_joint1, J5: index_rota_joint2
    # J6: mid_joint1, J7: mid_joint2
    # J8: ring_joint1, J9: ring_joint2
    # J10: pinky_joint1, J11: pinky_joint2
    
    joint_map = {
        f"{prefix}_thumb_bend_joint": joint_angles_rad[0],
        f"{prefix}_thumb_rota_joint1": joint_angles_rad[1],
        f"{prefix}_thumb_rota_joint2": joint_angles_rad[2],
        f"{prefix}_index_bend_joint": joint_angles_rad[3],
        f"{prefix}_index_rota_joint1": joint_angles_rad[4],
        f"{prefix}_index_rota_joint2": joint_angles_rad[5],
        f"{prefix}_mid_joint1": joint_angles_rad[6],
        f"{prefix}_mid_joint2": joint_angles_rad[7],
        f"{prefix}_ring_joint1": joint_angles_rad[8],
        f"{prefix}_ring_joint2": joint_angles_rad[9],
        f"{prefix}_pinky_joint1": joint_angles_rad[10],
        f"{prefix}_pinky_joint2": joint_angles_rad[11],
    }
    
    return joint_map


class XHandSimCommand:
    """
    Unified command interface for XHAND in Isaac Sim.
    Matches the real hardware interface but outputs for simulation.
    """
    
    def __init__(self, hand_type: str = "left"):
        """
        Initialize XHandSimCommand.
        
        Args:
            hand_type (str): "left" or "right"
        """
        self.hand_type = hand_type
        self.current_pose_deg = [0.0] * 12
        self.current_pose_rad = [0.0] * 12
        
    def set_pose_degrees(self, degrees: List[float]) -> Dict[str, float]:
        """
        Set hand pose from degrees (real hardware format).
        
        Args:
            degrees (List[float]): 12 joint angles in degrees
            
        Returns:
            Dict[str, float]: Joint position dict for Isaac Lab
        """
        self.current_pose_deg = degrees
        self.current_pose_rad = degrees_to_radians(degrees)
        return create_joint_position_dict(self.current_pose_rad, self.hand_type)
    
    def set_preset_pose(self, pose_name: str) -> Dict[str, float]:
        """
        Set a preset pose.
        
        Args:
            pose_name (str): Name of preset ('palm', 'fist', 'v', 'ok')
            
        Returns:
            Dict[str, float]: Joint position dict for Isaac Lab
        """
        degrees = get_preset_pose_degrees(pose_name)
        return self.set_pose_degrees(degrees)
    
    def get_current_pose_radians(self) -> List[float]:
        """Get current pose in radians."""
        return self.current_pose_rad.copy()
    
    def get_current_pose_degrees(self) -> List[float]:
        """Get current pose in degrees."""
        return self.current_pose_deg.copy()

