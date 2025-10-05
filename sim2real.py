# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""XHAND Sim2Real - GUI sliders control BOTH simulation AND real robots simultaneously.

Usage:
    python sim2real.py

Features:
    - Interactive GUI sliders for each joint
    - Real-time control of BOTH sim and real robots
    - Preset pose buttons (fist, palm, etc.)
    - Same joint mapping as sim_tune.py

Hardware Setup:
    - Left hand: /dev/ttyUSB1, ID=2
    - Right hand: /dev/ttyUSB0, ID=1
    - Mode 3 (position control)

"""

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="XHAND Sim2Real control.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments.")
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

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import AssetBaseCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from dataclasses import dataclass, field

# Import XHAND hardware controller
from xhandlib.xhand import XHandControl
from xhandlib.isaacsim_utils import PRESET_POSES, hardware_to_urdf_order


# Joint drive configuration (same as sim_tune.py)
@dataclass
class JointDriveGainsCfg:
    stiffness: float = 1000.0
    damping: float = 100.0

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
    init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, -0.20, 1.05), rot=(0.0, 0.0, 0.0, 1.0)),
    actuators={
        "all_joints": ImplicitActuatorCfg(
            joint_names_expr=[".*"], stiffness=2000.0, damping=200.0,
            effort_limit_sim=100.0, velocity_limit_sim=100.0,
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
    init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.20, 1.05), rot=(0.0, 0.0, 0.0, 1.0)),
    actuators={
        "all_joints": ImplicitActuatorCfg(
            joint_names_expr=[".*"], stiffness=2000.0, damping=200.0,
            effort_limit_sim=100.0, velocity_limit_sim=100.0,
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


class Sim2RealGUI:
    """Interactive GUI for controlling both sim and real robots."""
    
    def __init__(self, scene: InteractiveScene, xhand_left: XHandControl, xhand_right: XHandControl):
        self.scene = scene
        self.xhand_left = xhand_left
        self.xhand_right = xhand_right
        
        # Joint slider values (in radians, hardware order)
        self.joint_values = [0.0] * 12
        
        # Joint info (hardware order)
        self.joint_info = [
            ("HW J0: Thumb Yaw", 0.0, 1.57, "thumb_yaw"),
            ("HW J1: Thumb MCP", -1.05, 1.57, "thumb_mcp"),
            ("HW J2: Thumb IP", 0.0, 1.57, "thumb_ip"),
            ("HW J3: Index Yaw", -0.09, 0.30, "index_yaw"),
            ("HW J4: Index MCP", 0.0, 1.92, "index_mcp"),
            ("HW J5: Index PIP", 0.0, 1.92, "index_pip"),
            ("HW J6: Middle MCP", 0.0, 1.92, "mid_mcp"),
            ("HW J7: Middle PIP", 0.0, 1.92, "mid_pip"),
            ("HW J8: Ring MCP", 0.0, 1.92, "ring_mcp"),
            ("HW J9: Ring PIP", 0.0, 1.92, "ring_pip"),
            ("HW J10: Pinky MCP", 0.0, 1.92, "pinky_mcp"),
            ("HW J11: Pinky PIP", 0.0, 1.92, "pinky_pip"),
        ]
        
        self._build_ui()
    
    def _build_ui(self):
        """Build the UI window with sliders."""
        self.ui_window = ui.Window("XHAND Sim2Real Controller", width=450, height=850)
        
        with self.ui_window.frame:
            with ui.VStack(spacing=5):
                ui.Label("XHAND Sim2Real Control", height=35, alignment=ui.Alignment.CENTER)
                ui.Label("⚠️ CAUTION: Sliders control REAL robots!", height=25, alignment=ui.Alignment.CENTER)
                ui.Separator()
                
                # Create slider for each joint
                self.sliders = []
                self.value_labels = []
                
                for idx, (name, min_val, max_val, _) in enumerate(self.joint_info):
                    with ui.HStack(height=55):
                        ui.Label(name, width=140)
                        
                        with ui.VStack(spacing=2):
                            slider = ui.FloatSlider(min=min_val, max=max_val, height=22)
                            slider.model.add_value_changed_fn(
                                lambda m, idx=idx: self._on_slider_changed(idx, m.get_value_as_float())
                            )
                            self.sliders.append(slider)
                            
                            value_label = ui.Label(f"0.000 rad (0.0°)", height=18)
                            self.value_labels.append(value_label)
                
                ui.Separator()
                
                # Preset buttons
                ui.Label("Preset Poses (⚠️ Moves Real Robots!):", height=22)
                with ui.HStack(height=35, spacing=5):
                    ui.Button("Fist", clicked_fn=lambda: self._load_preset('fist'), width=70)
                    ui.Button("Palm", clicked_fn=lambda: self._load_preset('palm'), width=70)
                    ui.Button("V", clicked_fn=lambda: self._load_preset('v'), width=70)
                    ui.Button("OK", clicked_fn=lambda: self._load_preset('ok'), width=70)
                    ui.Button("Reset", clicked_fn=lambda: self._reset_joints(), width=70)
    
    def _on_slider_changed(self, joint_idx, value):
        """Called when a slider value changes."""
        self.joint_values[joint_idx] = value
        deg = value * 180.0 / math.pi
        self.value_labels[joint_idx].text = f"{value:.3f} rad ({deg:.1f}°)"
    
    def _load_preset(self, preset_name):
        """Load a preset pose."""
        if preset_name in PRESET_POSES:
            degrees_hw = PRESET_POSES[preset_name]
            print(f"[INFO]: Loading '{preset_name}': {degrees_hw}")
            
            for hw_idx, deg in enumerate(degrees_hw):
                rad = deg * math.pi / 180.0
                _, min_val, max_val, _ = self.joint_info[hw_idx]
                rad = max(min_val, min(max_val, rad))
                
                self.joint_values[hw_idx] = rad
                self.sliders[hw_idx].model.set_value(rad)
    
    def _reset_joints(self):
        """Reset all joints to zero."""
        print("[INFO]: Resetting all joints to zero")
        for idx in range(12):
            self.joint_values[idx] = 0.0
            self.sliders[idx].model.set_value(0.0)
    
    def get_joint_targets_hw(self):
        """Get current joint targets in hardware order."""
        return self.joint_values


def run_sim2real(sim: sim_utils.SimulationContext, scene: InteractiveScene, gui: Sim2RealGUI,
                 xhand_left: XHandControl, xhand_right: XHandControl):
    """
    Run sim2real control loop.
    Controls BOTH simulation and real robots from GUI sliders.
    """
    sim_dt = sim.get_physics_dt()
    
    print("[INFO]: ═══════════════════════════════════════════════════")
    print("[INFO]: Sim2Real Mode Active")
    print("[INFO]: ⚠️  SLIDERS CONTROL REAL ROBOTS - BE CAREFUL!")
    print("[INFO]: ═══════════════════════════════════════════════════")
    
    while simulation_app.is_running():
        try:
            # Get GUI values (hardware order)
            hw_values_rad = gui.get_joint_targets_hw()
            
            # Apply to simulation (with hardware→URDF mapping)
            urdf_values_rad = hardware_to_urdf_order(hw_values_rad)
            sim_target = torch.tensor(urdf_values_rad, dtype=torch.float32).unsqueeze(0)
            
            scene["left_hand"].set_joint_position_target(sim_target)
            scene["right_hand"].set_joint_position_target(sim_target)
            scene.write_data_to_sim()
            
            # Apply to real robots (hardware order, no remapping needed!)
            for i in range(12):
                xhand_left._hand_command.finger_command[i].position = hw_values_rad[i]
                xhand_right._hand_command.finger_command[i].position = hw_values_rad[i]
            
            # Send to real robots
            xhand_left.send_command()
            xhand_right.send_command()
            
            # Step simulation
            sim.step()
            scene.update(sim_dt)
            
        except Exception as e:
            print(f"[ERROR]: {e}")
            break


def main():
    """Main function."""
    
    # ========== Connect Real Robots ==========
    LEFT_SERIAL_PORT = "/dev/ttyUSB1"
    RIGHT_SERIAL_PORT = "/dev/ttyUSB0"
    LEFT_ID = 2
    RIGHT_ID = 1
    
    print("[INFO]: Connecting to real robots...")
    xhand_left = XHandControl(hand_id=LEFT_ID, position=0.1, mode=3)
    xhand_right = XHandControl(hand_id=RIGHT_ID, position=0.1, mode=3)
    
    left_config = {"protocol": "RS485", "serial_port": LEFT_SERIAL_PORT, "baud_rate": 3000000}
    right_config = {"protocol": "RS485", "serial_port": RIGHT_SERIAL_PORT, "baud_rate": 3000000}
    
    if not xhand_left.open_device(left_config):
        print("[ERROR]: Failed to connect to left hand")
        sys.exit(1)
    
    if not xhand_right.open_device(right_config):
        print("[ERROR]: Failed to connect to right hand")
        xhand_left.close()
        sys.exit(1)
    
    print("[INFO]: ✓ Real robots connected")
    
    try:
        # ========== Initialize Simulation ==========
        sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
        sim = sim_utils.SimulationContext(sim_cfg)
        
        sim.set_camera_view(eye=[-1.5, 0.0, 1.8], target=[0.0, 0.0, 1.1])
        
        scene_cfg = XHandSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0)
        scene = InteractiveScene(scene_cfg)
        
        sim.reset()
        
        print("[INFO]: ✓ Simulation ready")
        print("[INFO]: Use GUI sliders to control both sim and real robots")
        
        # ========== Create GUI ==========
        gui = Sim2RealGUI(scene, xhand_left, xhand_right)
        
        # ========== Run Control Loop ==========
        run_sim2real(sim, scene, gui, xhand_left, xhand_right)
        
    finally:
        # Clean shutdown
        print("[INFO]: Closing real robot connections...")
        xhand_left.close()
        xhand_right.close()
        print("[INFO]: ✓ Disconnected")


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()

