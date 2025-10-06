# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""XHAND visualization - identical to sim_tune.py but without GUI.

Usage:
    python sim2.py

"""

import argparse
import os
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="XHAND dual-hand visualization.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

# Get the project root directory (parent of Sim/)
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
print(f"[DEBUG] Project root: {PROJECT_ROOT}")
print(f"[DEBUG] Left URDF path: {os.path.join(PROJECT_ROOT, 'xhandurdf/xhand_left/urdf/xhand_left_isaac.urdf')}")
print(f"[DEBUG] Right URDF path: {os.path.join(PROJECT_ROOT, 'xhandurdf/xhand_right/urdf/xhand_right_isaac.urdf')}")
print(f"[DEBUG] Left URDF exists: {os.path.exists(os.path.join(PROJECT_ROOT, 'xhandurdf/xhand_left/urdf/xhand_left_isaac.urdf'))}")
print(f"[DEBUG] Right URDF exists: {os.path.exists(os.path.join(PROJECT_ROOT, 'xhandurdf/xhand_right/urdf/xhand_right_isaac.urdf'))}")

# Add project root to Python path for xhandlib imports
import sys
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)
print(f"[DEBUG] Added to Python path: {PROJECT_ROOT}")

import torch
import numpy as np

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


# XHAND Configuration (same as sim_tune.py)
XHAND_LEFT_CONFIG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path=os.path.join(PROJECT_ROOT, "xhandurdf/xhand_left/urdf/xhand_left_isaac.urdf"),
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

XHAND_RIGHT_CONFIG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path=os.path.join(PROJECT_ROOT, "xhandurdf/xhand_right/urdf/xhand_right_isaac.urdf"),
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


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    """Run simulator - EXACT SAME as sim_tune.py control loop."""
    sim_dt = sim.get_physics_dt()
    
    # Get fist pose in HARDWARE order
    from xhandlib.isaacsim_utils import PRESET_POSES, hardware_to_urdf_order
    
    fist_hw_deg = PRESET_POSES['fist']
    print(f"[INFO]: Fist pose (HW order, degrees): {fist_hw_deg}")
    
    # Convert to radians (still in hardware order)
    fist_hw_rad = [deg * math.pi / 180.0 for deg in fist_hw_deg]
    
    # Apply hardware→URDF mapping (SAME AS sim_tune.py!)
    fist_urdf_rad = hardware_to_urdf_order(fist_hw_rad)
    
    print(f"[INFO]: Fist pose (URDF order, radians): {['%.3f' % r for r in fist_urdf_rad]}")
    print(f"[INFO]: Holding fist pose - should match sim_tune.py perfectly!")
    
    fist_tensor = torch.tensor(fist_urdf_rad, dtype=torch.float32)
    
    while simulation_app.is_running():
        # Apply fist pose (exact same method as sim_tune.py)
        target_tensor = fist_tensor.unsqueeze(0)  # [1, 12]
        
        scene["left_hand"].set_joint_position_target(target_tensor)
        scene["right_hand"].set_joint_position_target(target_tensor)
        
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)


def main():
    """Main function."""
    
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    
    sim.set_camera_view(
        eye=[-1.5, 0.0, 1.8],
        target=[0.0, 0.0, 1.1]
    )
    
    scene_cfg = XHandSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    
    sim.reset()
    
    print("[INFO]: XHAND sim2.py - Same code path as sim_tune.py")
    print("[INFO]: This should produce IDENTICAL results to sim_tune.py Fist button")
    
    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()

