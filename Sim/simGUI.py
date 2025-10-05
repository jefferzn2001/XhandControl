# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""XHAND Joint Tuning GUI - Interactive slider-based joint control for mapping calibration.

Usage:
    python sim_tune.py

This script provides real-time sliders for each joint to help you:
1. Visually tune each joint position
2. See the URDF angle values
3. Cross-reference with real robot joint angles
4. Build the correct remapping table

"""

"""Launch Isaac Sim Simulator first."""

import argparse
from isaaclab.app import AppLauncher

# Create argparser
parser = argparse.ArgumentParser(description="XHAND joint tuning GUI.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch
import numpy as np
import omni.ui as ui

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import AssetBaseCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from dataclasses import dataclass, field
import math


# Define joint drive configuration
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


# XHAND Configuration (same as sim.py but with zero initial pose)
XHAND_LEFT_CONFIG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path="xhandurdf/xhand_left/urdf/xhand_left_isaac.urdf",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
        ),
        activate_contact_sensors=False,
        fix_base=True,
        joint_drive=JointDriveCfg(),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, -0.20, 1.05),
        rot=(0.0, 0.0, 0.0, 1.0),
        # Start with all joints at zero for tuning
        joint_pos={jname: 0.0 for jname in [
            "left_hand_thumb_bend_joint", "left_hand_thumb_rota_joint1", "left_hand_thumb_rota_joint2",
            "left_hand_index_bend_joint", "left_hand_index_joint1", "left_hand_index_joint2",
            "left_hand_mid_joint1", "left_hand_mid_joint2",
            "left_hand_ring_joint1", "left_hand_ring_joint2",
            "left_hand_pinky_joint1", "left_hand_pinky_joint2"
        ]},
    ),
    actuators={
        "all_joints": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            stiffness=2000.0,
            damping=200.0,
            effort_limit_sim=100.0,
            velocity_limit_sim=100.0,
        ),
    },
)

# Same for right hand
XHAND_RIGHT_CONFIG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path="xhandurdf/xhand_right/urdf/xhand_right_isaac.urdf",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
        ),
        activate_contact_sensors=False,
        fix_base=True,
        joint_drive=JointDriveCfg(),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.20, 1.05),
        rot=(0.0, 0.0, 0.0, 1.0),
        joint_pos={jname: 0.0 for jname in [
            "right_hand_thumb_bend_joint", "right_hand_thumb_rota_joint1", "right_hand_thumb_rota_joint2",
            "right_hand_index_bend_joint", "right_hand_index_joint1", "right_hand_index_joint2",
            "right_hand_mid_joint1", "right_hand_mid_joint2",
            "right_hand_ring_joint1", "right_hand_ring_joint2",
            "right_hand_pinky_joint1", "right_hand_pinky_joint2"
        ]},
    ),
    actuators={
        "all_joints": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            stiffness=2000.0,
            damping=200.0,
            effort_limit_sim=100.0,
            velocity_limit_sim=100.0,
        ),
    },
)


class XHandSceneCfg(InteractiveSceneCfg):
    """Scene configuration."""
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())
    dome_light = AssetBaseCfg(prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75)))
    table = AssetBaseCfg(
        prim_path="/World/Table",
        spawn=sim_utils.UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 1.05)),
    )
    left_hand = XHAND_LEFT_CONFIG.replace(prim_path="{ENV_REGEX_NS}/LeftHand")
    right_hand = XHAND_RIGHT_CONFIG.replace(prim_path="{ENV_REGEX_NS}/RightHand")


class JointTuningGUI:
    """Interactive GUI for tuning hand joint positions."""
    
    def __init__(self, scene: InteractiveScene):
        self.scene = scene
        
        # Joint slider values (in radians)
        self.joint_values = [0.0] * 12
        
        # Sliders in HARDWARE order with hardware joint limits
        # Now uses hardware_to_urdf_order() in control loop
        self.joint_info = [
            ("HW J0: Thumb Yaw", 0.0, 1.83, "thumb_yaw"),         # 0-105°
            ("HW J1: Thumb MCP", -0.69, 1.74, "thumb_mcp"),       # -40-100°
            ("HW J2: Thumb IP", 0.0, 1.74, "thumb_ip"),           # 0-100°
            ("HW J3: Index Yaw", -0.17, 0.17, "index_yaw"),       # -10-10°
            ("HW J4: Index MCP", 0.0, 1.91, "index_mcp"),         # 0-110°
            ("HW J5: Index PIP", 0.0, 1.91, "index_pip"),         # 0-110°
            ("HW J6: Middle MCP", 0.0, 1.91, "mid_mcp"),          # 0-110°
            ("HW J7: Middle PIP", 0.0, 1.91, "mid_pip"),          # 0-110°
            ("HW J8: Ring MCP", 0.0, 1.91, "ring_mcp"),           # 0-110°
            ("HW J9: Ring PIP", 0.0, 1.91, "ring_pip"),           # 0-110°
            ("HW J10: Pinky MCP", 0.0, 1.91, "pinky_mcp"),        # 0-110°
            ("HW J11: Pinky PIP", 0.0, 1.91, "pinky_pip"),        # 0-110°
        ]
        
        self._build_ui()
    
    def _build_ui(self):
        """Build the UI window with sliders."""
        self.ui_window = ui.Window("XHAND Joint Tuner", width=400, height=800)
        
        with self.ui_window.frame:
            with ui.VStack(spacing=5):
                ui.Label("XHAND Joint Position Tuner", height=30, alignment=ui.Alignment.CENTER)
                ui.Label("Adjust sliders to tune joint positions", height=20)
                ui.Separator()
                
                # Create slider for each joint
                self.sliders = []
                self.value_labels = []
                
                for idx, (name, min_val, max_val, urdf_name) in enumerate(self.joint_info):
                    with ui.HStack(height=50):
                        ui.Label(name, width=120)
                        
                        with ui.VStack(spacing=2):
                            # Slider
                            slider = ui.FloatSlider(
                                min=min_val,
                                max=max_val,
                                height=20
                            )
                            slider.model.add_value_changed_fn(
                                lambda m, idx=idx: self._on_slider_changed(idx, m.get_value_as_float())
                            )
                            self.sliders.append(slider)
                            
                            # Value display
                            value_label = ui.Label(f"URDF: 0.000 rad (0.0°)", height=15)
                            self.value_labels.append(value_label)
                
                ui.Separator()
                
                # Preset buttons
                ui.Label("Preset Poses:", height=20)
                with ui.HStack(height=30):
                    ui.Button("Fist", clicked_fn=lambda: self._load_preset('fist'), width=80)
                    ui.Button("Palm", clicked_fn=lambda: self._load_preset('palm'), width=80)
                    ui.Button("Reset", clicked_fn=lambda: self._reset_joints(), width=80)
                
                ui.Separator()
                
                # Output mapping
                ui.Label("Current Mapping (copy to code):", height=20)
                self.mapping_output = ui.Label("", height=100, word_wrap=True)
    
    def _on_slider_changed(self, joint_idx, value):
        """Called when a slider value changes."""
        self.joint_values[joint_idx] = value
        
        # Update label
        deg = value * 180.0 / math.pi
        self.value_labels[joint_idx].text = f"URDF: {value:.3f} rad ({deg:.1f}°)"
        
        # Update mapping output
        self._update_mapping_output()
    
    def _load_preset(self, preset_name):
        """Load a preset pose in HARDWARE order."""
        from xhandlib.isaacsim_utils import PRESET_POSES
        
        if preset_name in PRESET_POSES:
            degrees_hw = PRESET_POSES[preset_name]
            print(f"[INFO]: Loading '{preset_name}' (HW order): {degrees_hw}")
            
            # Set sliders in HARDWARE order (direct, no conversion yet)
            for hw_idx, deg in enumerate(degrees_hw):
                rad = deg * math.pi / 180.0
                # Clamp to slider range
                _, min_val, max_val, _ = self.joint_info[hw_idx]
                rad = max(min_val, min(max_val, rad))
                
                self.joint_values[hw_idx] = rad
                self.sliders[hw_idx].model.set_value(rad)
    
    def _reset_joints(self):
        """Reset all joints to zero."""
        for idx in range(12):
            self.joint_values[idx] = 0.0
            self.sliders[idx].model.set_value(0.0)
    
    def _update_mapping_output(self):
        """Update the mapping output text."""
        mapping_text = "joint_pos={\n"
        for idx, (name, _, _, urdf_name) in enumerate(self.joint_info):
            rad = self.joint_values[idx]
            deg = rad * 180.0 / math.pi
            mapping_text += f'    "{urdf_name}": {rad:.3f},  # J{idx}: {deg:.1f}°\n'
        mapping_text += "}"
        
        self.mapping_output.text = mapping_text
    
    def get_joint_targets(self):
        """
        Get current joint targets as tensor.
        
        Returns values in the order that scene.set_joint_position_target expects,
        which is the alphabetical order of joint names in the URDF.
        """
        # Get the actual joint order from the scene
        # For now, return in the order that matches our joint_info URDF names
        # The scene will match by joint name, not by index
        
        # Return in hardware order - the control loop will handle it
        return torch.tensor(self.joint_values, dtype=torch.float32)


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene, gui: JointTuningGUI):
    """Run simulator with GUI control."""
    sim_dt = sim.get_physics_dt()
    
    print("[INFO]: ═══════════════════════════════════════════════════")
    print("[INFO]: XHAND with CORRECTED Joint Mapping")
    print("[INFO]: ═══════════════════════════════════════════════════")
    print("[INFO]: Sliders now labeled with HARDWARE joint names")
    print("[INFO]: Click 'Fist' to test the mapping!")
    print("[INFO]: ═══════════════════════════════════════════════════")
    
    while simulation_app.is_running():
        # Get GUI values and apply hardware→URDF mapping
        from xhandlib.isaacsim_utils import hardware_to_urdf_order
        
        # GUI values are in hardware order (slider 0 = HW J0, slider 1 = HW J1, etc.)
        hw_values = gui.get_joint_targets().numpy().tolist()
        
        # Remap to URDF order
        urdf_values = hardware_to_urdf_order(hw_values)
        
        target_tensor = torch.tensor(urdf_values, dtype=torch.float32).unsqueeze(0)  # [1, 12]
        
        # Apply to both hands
        scene["left_hand"].set_joint_position_target(target_tensor)
        scene["right_hand"].set_joint_position_target(target_tensor)
        
        # Write to simulation
        scene.write_data_to_sim()
        
        # Step physics
        sim.step()
        
        # Update scene
        scene.update(sim_dt)


def main():
    """Main function."""
    
    # Initialize simulation
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    
    # Set camera
    sim.set_camera_view(
        eye=[-1.5, 0.0, 1.8],
        target=[0.0, 0.0, 1.1]
    )
    
    # Create scene
    scene_cfg = XHandSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    
    # Reset simulation
    sim.reset()
    
    print("[INFO]: XHAND Joint Tuning GUI ready")
    print("[INFO]: Use the sliders in the UI window to adjust joints")
    print("[INFO]: Watch the hands update in real-time")
    print("[INFO]: Copy the mapping output when you find good values")
    
    # Create GUI
    gui = JointTuningGUI(scene)
    
    # Run simulator with GUI
    run_simulator(sim, scene, gui)


if __name__ == "__main__":
    main()
    simulation_app.close()

