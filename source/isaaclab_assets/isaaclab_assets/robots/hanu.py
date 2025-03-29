# Copyright (c) 2025, RAI Hanumanoid Project Developers.
# All right reserved.

"""Configuration for the XX-DOFs Hanumanoid robot.

The following configuration are available:
* :obj:`HANU_A0_CFG`: Hanumanoid A0 robot

Reference: https://github.com/Zartern/Hanumanoid
"""

import isaaclab.sim as sim_util
from isaaclab.actuators import ActuatorNetMLPCfg, DCMotorCfg, ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

HANU_A0_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/LegV5_URDF_Export",
    # prim_path="/LegV5_URDF_Export",
    # prim_path=f"/home/RAI_65011278/IsaacLab/source/isaaclab_assets/data/Hanu/LegV5_URDF_Export_description/",
    spawn=sim_util.UsdFileCfg(
        usd_path=f"/home/RAI_65011278/IsaacLab/source/isaaclab_assets/data/Hanu/LegV5_URDF_Export_description/urdf/LegV5/LegV5_v1.usd", # TODO: set path for usd
        activate_contact_sensors=True,
        rigid_props=sim_util.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0
        ),
        articulation_props=sim_util.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.65), # (x, y, z)
        joint_pos={
            "base_to_L1": 0.0,
            "base_to_R1": 0.0,
            "L1_to_L2_Rev": 0.0,
            "R1_to_R2_Rev": 0.0,
            "L2_to_L3_Rev": 0.0,
            "R2_to_R3_Rev": 0.0,
            "L3_to_L4_Rev": 0.0,
            "R3_to_R4_Rev": 0.0,
            "L4_to_L11_Rev": 0.0,
            "L4_to_L5_Rev": 0.0,
            "L4_to_L6_Rev": 0.0,
            "R4_to_R11_Rev": 0.0,
            "R4_to_R5_Rev": 0.0,
            "R4_to_R6_Rev": 0.0,
        },
        joint_vel={".*": 0.0},
        ),
        soft_joint_pos_limit_factor=0.9,
        actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                "base_to_L1",
                "base_to_R1",
                "L1_to_L2_Rev",
                "R1_to_R2_Rev",
                "L2_to_L3_Rev",
                "R2_to_R3_Rev",
                "L3_to_L4_Rev",
                "R3_to_R4_Rev",
                "L4_to_L11_Rev",
                "L4_to_L5_Rev",
                "L4_to_L6_Rev",
                "R4_to_R11_Rev",
                "R4_to_R5_Rev",
                "R4_to_R6_Rev",
            ],
            effort_limit=300,
            velocity_limit=100.0,
            stiffness={
                "base_to_L1": 10.0,
                "base_to_R1": 10.0,
                "L1_to_L2_Rev": 10.0,
                "R1_to_R2_Rev": 10.0,
                "L2_to_L3_Rev": 10.0,
                "R2_to_R3_Rev": 10.0,
                "L3_to_L4_Rev": 10.0,
                "R3_to_R4_Rev": 10.0,
                "L4_to_L11_Rev": 10.0,
                "L4_to_L5_Rev": 10.0,
                "L4_to_L6_Rev": 10.0,
                "R4_to_R11_Rev": 10.0,
                "R4_to_R5_Rev": 10.0,
                "R4_to_R6_Rev": 10.0,
                # ".*_hip_yaw_joint": 150.0,
                # ".*_hip_roll_joint": 150.0,
                # ".*_hip_pitch_joint": 200.0,
                # ".*_knee_joint": 200.0,
                # "torso_joint": 200.0,
            },
            damping={
                "base_to_L1": 1.0,
                "L1_to_L2_Rev": 1.0,
                "L4_to_L11_Rev": 1.0,
                "L4_to_L5_Rev": 1.0,
                "base_to_R1": 1.0,
                "R1_to_R2_Rev": 1.0,
                "R4_to_R11_Rev": 1.0,
                "R4_to_R5_Rev": 1.0,
                # ".*_hip_yaw_joint": 5.0,
                # ".*_hip_roll_joint": 5.0,
                # ".*_hip_pitch_joint": 5.0,
                # ".*_knee_joint": 5.0,
                # "torso_joint": 5.0,
            },
            # armature={
            #     ".*_hip_.*": 0.01,
            #     ".*_knee_joint": 0.01,
            #     "torso_joint": 0.01,
            # },
        ),
        "feet": ImplicitActuatorCfg(
            effort_limit=20,
            joint_names_expr=["L6_to_L13_Rev", "R6_to_R13_Rev"],
            stiffness=20.0,
            damping=2.0,
            armature=0.01,
        ),
        # "arms": ImplicitActuatorCfg(
        #     joint_names_expr=[
        #         ".*_shoulder_pitch_joint",
        #         ".*_shoulder_roll_joint",
        #         ".*_shoulder_yaw_joint",
        #         ".*_elbow_pitch_joint",
        #         ".*_elbow_roll_joint",
        #         ".*_five_joint",
        #         ".*_three_joint",
        #         ".*_six_joint",
        #         ".*_four_joint",
        #         ".*_zero_joint",
        #         ".*_one_joint",
        #         ".*_two_joint",
        #     ],
        #     effort_limit=300,
        #     velocity_limit=100.0,
        #     stiffness=40.0,
        #     damping=10.0,
        #     armature={
        #         ".*_shoulder_.*": 0.01,
        #         ".*_elbow_.*": 0.01,
        #         ".*_five_joint": 0.001,
        #         ".*_three_joint": 0.001,
        #         ".*_six_joint": 0.001,
        #         ".*_four_joint": 0.001,
        #         ".*_zero_joint": 0.001,
        #         ".*_one_joint": 0.001,
        #         ".*_two_joint": 0.001,
        #     },
        # ),
    },
)
"""Configuration for the Hanumanoid A0 robot."""