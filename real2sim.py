# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""XHAND Real2Sim - Read encoder from real robots and mirror in simulation.

Usage:
    python real2sim.py

Features:
    - Reads joint positions from real robots (mode 0, powerless)
    - Mirrors movements in Isaac Sim in real-time
    - Shows encoder values in GUI
    - Perfect for teleoperation and demonstration recording

Hardware Setup:
    - Left hand: /dev/ttyUSB0, ID=1 (MODE 0 - Powerless for manual movement)
    - Right hand: /dev/ttyUSB1, ID=0 (MODE 0 - Powerless for manual movement)

"""

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="XHAND Real2Sim teleoperation.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments.")
parser.add_argument("--update_rate", type=int, default=30, help="Encoder update rate (Hz)")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch
import numpy as np
import omni.ui as ui
import sys
import math
import time

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import AssetBaseCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from dataclasses import dataclass, field

# Import XHAND hardware controller
from xhandlib.xhand import XHandControl
from xhandlib.isaacsim_utils import hardware_to_urdf_order


# Joint drive configuration
# ═══════════════════════════════════════════════════════════════════════════
# 🎯 TUNING PARAMETERS - Adjust these to match real robot behavior:
# - stiffness (kp): Controls position tracking stiffness (default: 1000.0)
#   Higher = stiffer/faster response, Lower = softer/slower response
# - damping (kd): Controls velocity damping (default: 100.0)
#   Higher = more damped/stable, Lower = less damped/oscillatory
# ═══════════════════════════════════════════════════════════════════════════
@dataclass
class JointDriveGainsCfg:
    stiffness: float = 1000.0  # kp gain - tune for position tracking
    damping: float = 100.0      # kd gain - tune for velocity damping

@dataclass  
class JointDriveCfg:
    gains: JointDriveGainsCfg = field(default_factory=lambda: JointDriveGainsCfg())
    max_force: float = 100.0
    drive_type: str = "force"  
    target_type: str = "position"


# XHAND Configurations (same as sim2.py)
XHAND_LEFT_CONFIG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path="xhandurdf/xhand_left/urdf/xhand_left_isaac.urdf",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False, max_depenetration_velocity=1.0),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
        activate_contact_sensors=False,
        fix_base=True,
        joint_drive=JointDriveCfg(),
    ),
    init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.20, 1.05), rot=(0.0, 0.0, 0.0, 1.0)),
    actuators={
        "all_joints": ImplicitActuatorCfg(
            joint_names_expr=[".*"], 
            stiffness=3000.0,  # 🎯 TUNE: Actuator stiffness (kp) - increase for faster response
            damping=200.0,     # 🎯 TUNE: Actuator damping (kd) - increase for more stability
            effort_limit_sim=100.0, 
            velocity_limit_sim=300.0,
        ),
    },
)

XHAND_RIGHT_CONFIG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path="xhandurdf/xhand_right/urdf/xhand_right_isaac.urdf",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False, max_depenetration_velocity=1.0),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
        activate_contact_sensors=False,
        fix_base=True,
        joint_drive=JointDriveCfg(),
    ),
    init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, -0.20, 1.05), rot=(0.0, 0.0, 0.0, 1.0)),
    actuators={
        "all_joints": ImplicitActuatorCfg(
            joint_names_expr=[".*"], 
            stiffness=3000.0,  # 🎯 TUNE: Actuator stiffness (kp) - increase for faster response
            damping=200.0,     # 🎯 TUNE: Actuator damping (kd) - increase for more stability
            effort_limit_sim=100.0, 
            velocity_limit_sim=300.0,
        ),
    },
)


class XHandSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())
    dome_light = AssetBaseCfg(prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75)))
    table = AssetBaseCfg(
        prim_path="/World/Table",
        spawn=sim_utils.UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 1.05)),
    )
    left_hand = XHAND_LEFT_CONFIG.replace(prim_path="{ENV_REGEX_NS}/LeftHand")
    right_hand = XHAND_RIGHT_CONFIG.replace(prim_path="{ENV_REGEX_NS}/RightHand")


class Real2SimMonitor:
    """Simple GUI to show encoder values."""
    
    def __init__(self):
        self.encoder_values_deg = [[0.0]*12, [0.0]*12]  # [left, right]
        self._build_ui()
    
    def _build_ui(self):
        """Build status window."""
        self.ui_window = ui.Window("XHAND Real2Sim Monitor", width=500, height=400)
        
        with self.ui_window.frame:
            with ui.VStack(spacing=5):
                ui.Label("XHAND Real2Sim Monitor", height=35, alignment=ui.Alignment.CENTER)
                ui.Separator()
                
                ui.Label("Left Hand:", height=20)
                self.left_label = ui.Label("Not updated yet...", height=60, word_wrap=True)
                
                ui.Separator()
                
                ui.Label("Right Hand:", height=20)
                self.right_label = ui.Label("Not updated yet...", height=60, word_wrap=True)
                
                ui.Separator()
                
                ui.Label("Status:", height=20)
                self.status_label = ui.Label("Running", height=30)
    
    def update_display(self, left_deg, right_deg):
        """Update the display with current encoder values."""
        left_str = ", ".join([f"J{i}:{deg:.1f}" for i, deg in enumerate(left_deg)])
        right_str = ", ".join([f"J{i}:{deg:.1f}" for i, deg in enumerate(right_deg)])
        
        self.left_label.text = left_str
        self.right_label.text = right_str
        self.encoder_values_deg = [left_deg, right_deg]


def run_real2sim(sim: sim_utils.SimulationContext, scene: InteractiveScene, monitor: Real2SimMonitor,
                 xhand_left: XHandControl, xhand_right: XHandControl):
    """
    Run real2sim teleoperation loop.
    Reads encoder from real robots and mirrors in simulation.
    """
    sim_dt = sim.get_physics_dt()
    update_interval = int(1.0 / args_cli.update_rate / sim_dt)  # Update every N sim steps
    count = 0
    
    print("[INFO]: ═══════════════════════════════════════════════════")
    print("[INFO]: Real2Sim Teleoperation Active")
    print("[INFO]: Simulation will mirror your movements")
    print("[INFO]: Update rate: ~{}Hz".format(args_cli.update_rate))
    print("[INFO]: ═══════════════════════════════════════════════════")
    
    while simulation_app.is_running():
        try:
            # Read encoder from real robots (only every N steps to match update rate)
            if count % update_interval == 0:
                # Read state from both hands
                state_left = xhand_left.read_state(xhand_left._hand_id, force_update=True)
                state_right = xhand_right.read_state(xhand_right._hand_id, force_update=True)
                
                # Extract joint positions (hardware order)
                left_positions_rad = []
                right_positions_rad = []
                
                for i in range(12):
                    if i < len(state_left.finger_state):
                        pos = float(state_left.finger_state[i].position)
                        left_positions_rad.append(pos)
                    else:
                        left_positions_rad.append(0.0)
                    
                    if i < len(state_right.finger_state):
                        pos = float(state_right.finger_state[i].position)
                        right_positions_rad.append(pos)
                    else:
                        right_positions_rad.append(0.0)
                
                # Convert to degrees for display
                left_deg = [math.degrees(r) for r in left_positions_rad]
                right_deg = [math.degrees(r) for r in right_positions_rad]
                
                # Update GUI
                monitor.update_display(left_deg, right_deg)
                
                # Apply to simulation (with hardware→URDF mapping)
                left_urdf_rad = hardware_to_urdf_order(left_positions_rad)
                right_urdf_rad = hardware_to_urdf_order(right_positions_rad)
                
                left_target = torch.tensor(left_urdf_rad, dtype=torch.float32).unsqueeze(0)
                right_target = torch.tensor(right_urdf_rad, dtype=torch.float32).unsqueeze(0)
                
                scene["left_hand"].set_joint_position_target(left_target)
                scene["right_hand"].set_joint_position_target(right_target)
                scene.write_data_to_sim()
            
            # Step simulation
            sim.step()
            scene.update(sim_dt)
            count += 1
            
        except KeyboardInterrupt:
            print("[INFO]: Ctrl+C detected, shutting down...")
            break
        except Exception as e:
            print(f"[ERROR]: {e}")
            import traceback
            traceback.print_exc()
            break


def main():
    """Main function."""
    
    # ========== Connect Real Robots (Mode 0 - Powerless) ==========
    LEFT_SERIAL_PORT = "/dev/ttyUSB0"
    RIGHT_SERIAL_PORT = "/dev/ttyUSB1"
    LEFT_ID = 1
    RIGHT_ID = 0
    
    print("[INFO]: Connecting to real robots in ENCODER mode (powerless)...")
    xhand_left = XHandControl(hand_id=LEFT_ID, position=0.1, mode=0)  # Mode 0 = powerless
    xhand_right = XHandControl(hand_id=RIGHT_ID, position=0.1, mode=0)
    
    left_config = {"protocol": "RS485", "serial_port": LEFT_SERIAL_PORT, "baud_rate": 3000000}
    right_config = {"protocol": "RS485", "serial_port": RIGHT_SERIAL_PORT, "baud_rate": 3000000}
    
    if not xhand_left.open_device(left_config):
        print("[ERROR]: Failed to connect to left hand")
        sys.exit(1)
    
    if not xhand_right.open_device(right_config):
        print("[ERROR]: Failed to connect to right hand")
        xhand_left.close()
        sys.exit(1)

    
    try:
        # ========== Initialize Simulation ==========
        sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
        sim = sim_utils.SimulationContext(sim_cfg)
        
        sim.set_camera_view(eye=[-1.5, 0.0, 1.8], target=[0.0, 0.0, 1.1])
        
        scene_cfg = XHandSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0)
        scene = InteractiveScene(scene_cfg)
        
        sim.reset()
        
        print("[INFO]: ✓ Simulation ready")
        
        # ========== Create Monitor GUI ==========
        monitor = Real2SimMonitor()
        
        # ========== Run Teleoperation Loop ==========
        run_real2sim(sim, scene, monitor, xhand_left, xhand_right)
        
    finally:
        # Clean shutdown
        print("[INFO]: Closing real robot connections...")
        xhand_left.close()
        xhand_right.close()



if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()

