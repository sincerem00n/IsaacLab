# Copyright (c) 2025, RAI Hanumanoid Project Developers.
# All right reserved.

# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.utils import configclass

from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg


###########################
# Pre-defined configs
###########################
from isaaclab_assets.robots.hanu import HANU_A0_CFG

@configclass
class HanumanoidA0RoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # scene
        # self.scene.robot.prim_path = "{ENV_REGEX_NS}/LegV5_URDF_Export"
        self.scene.robot = HANU_A0_CFG.replace(prim_path="{ENV_REGEX_NS}/LegV5_URDF_Export") #! TODO: check dir
        self.scene.robot = HANU_A0_CFG
        # self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/LegV5_URDF_Export/base_link" # TODO: change Robot name
        

        self.scene.terrain.terrain_generator.sub_terrains["boxes"].grid_height_range = (0.025, 0.1)
        self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_range = (0.01, 0.06)
        self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_step = 0.01

        # action scale
        self.actions.joint_pos.scale = 1.0
        # self.actions.joint_vel.scale = 1.0
        # self.actions.joint_torque.scale = 1.0

        # events
        self.events.push_robot = None
        # self.events.add_base_mass.params["mass_distribution_params"] = (-1.0, 3.0)
        self.events.add_base_mass.params["asset_cfg"].body_names = "base_link" # TODO: check "base" link
        self.events.base_external_force_torque.params["asset_cfg"].body_names = "base_link"
        self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
        self.events.reset_base.params = {
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        }

        # rewards
        # self.rewards.feet_air_time.params["sensor_cfg"].body_names = ["L13_Foot_1_1", "R13_Foot_1_1"] # TODO: check ".*_foot" link
        # self.rewards.feet_air_time.weight = 0.01 # default: 0.125
        self.rewards.lin_vel_z_l2.weight = 0.0
        # self.rewards.undesired_contacts.params["sensor_cfg"].body_names = "base_link" # TODO: check ".*_thigh" link
        # self.rewards.undesired_contacts.weight = -0.1 # default: -0.1
        self.rewards.flat_orientation_l2.weight = -1.0
        self.rewards.action_rate_l2.weight = -0.005
        self.rewards.dof_acc_l2.weight = -1.25e-7
        self.rewards.dof_torques_l2.weight = -0.0002
        self.rewards.track_lin_vel_xy_exp.weight = 1.5
        self.rewards.track_ang_vel_z_exp.weight = 0.75
        

        # terminations
        self.terminations.base_contact.params["sensor_cfg"].body_names = "base_link"

@configclass
class HanumanoidA0RoughEnvCfg_PLAY(HanumanoidA0RoughEnvCfg):
    def __post_init__(self) -> None:
        super().__post_init__()

        self.scene.num_envs = 32
        self.scene.env_spacing = 2.5
        
        self.scene.terrain.max_init_terrain_level = None

        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 5
            self.scene.terrain.terrain_generator.num_cols = 5
            self.scene.terrain.terrain_generator.curriculum = False

        self.commands.base_velocity.ranges.lin_vel_x = (1.0, 1.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-1.0, 1.0)
        self.commands.base_velocity.ranges.heading = (0.0, 0.0)

        self.observations.policy.enable_corruption = False
        # remove random pushing event
        self.events.base_external_force_torque = None
        self.events.push_robot = None
