# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# Copyright (c) 2025, RAI Hanumanoid Project Developers.
# All right reserved.

# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Hanumanoid robot.

The following configuration are available:
* :obj:`HANU_A0_CFG`: Hanumanoid A0 robot with only legs and feet actuators.
* :obj:`HANU_A1_CFG`: Hanumanoid A1 robot (full body) with arms, legs, and torso actuators.

Reference: https://github.com/whaly-w/Hanumanoid
"""

import isaaclab.sim as sim_util
from isaaclab.actuators import ActuatorNetMLPCfg, DCMotorCfg, ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR
from isaaclab_assets import ISAACLAB_ASSETS_DATA_DIR

HANU_A0_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/LegV5_URDF_Export",
    # prim_path="/LegV5_URDF_Export",
    # prim_path=f"/home/RAI_65011278/IsaacLab/source/isaaclab_assets/data/Hanu/LegV5_URDF_Export_description/",
    spawn=sim_util.UsdFileCfg(
        usd_path=f"/home/RAI_65011278/IsaacLab/source/isaaclab_assets/data/Hanu/LegV5_URDF_Export_description/urdf/LegV5/LegV5_v1.usd",  # TODO: set path for usd
        activate_contact_sensors=True,
        rigid_props=sim_util.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_util.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.65),  # (x, y, z)
        rot=(1.0, 1.0, 0.0, 0.0),  # (w, x, y, z)
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
                "base_to_L1": 200.0,
                "base_to_R1": 200.0,
                "L1_to_L2_Rev": 150.0,
                "R1_to_R2_Rev": 150.0,
                "L2_to_L3_Rev": 100.0,
                "R2_to_R3_Rev": 100.0,
                "L3_to_L4_Rev": 100.0,
                "R3_to_R4_Rev": 100.0,
                "L4_to_L11_Rev": 100.0,
                "L4_to_L5_Rev": 100.0,
                "L4_to_L6_Rev": 100.0,
                "R4_to_R11_Rev": 100.0,
                "R4_to_R5_Rev": 100.0,
                "R4_to_R6_Rev": 100.0,
                # ".*_hip_yaw_joint": 150.0,
                # ".*_hip_roll_joint": 150.0,
                # ".*_hip_pitch_joint": 200.0,
                # ".*_knee_joint": 200.0,
                # "torso_joint": 200.0,
            },
            damping={
                ".*": 1.0,  # Set damping for all joints
            },
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

HANU_A1_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/full_body",  # TODO: check prim_path
    spawn=sim_util.UrdfFileCfg(
        fix_base=False,
        merge_fixed_joints=True,
        replace_cylinders_with_capsules=False,
        asset_path=f"{ISAACLAB_ASSETS_DATA_DIR}/Robots/hanu/full_body_description/urdf/full_body.urdf",
        # asset_path=r"C:\Users\jingj\IsaacLab\source\isaaclab_assets\data\Hanu\full_body_description\urdf\full_body.urdf",
        # usd_path=r"C:\Users\jingj\IsaacLab\source\isaaclab_assets\data\Hanu\full_body_description\urdf\full_body\full_body_v2.usd",  # TODO: set path for usd
        activate_contact_sensors=True,
        rigid_props=sim_util.RigidBodyPropertiesCfg(
            # disable_gravity=False,
            # retain_accelerations=False,
            rigid_body_enabled=True,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_util.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.93),  # (x, y, z)
        rot=(0.707, 0.0, 0.0, 0.707),  # (w, x, y, z)
        # joint_pos={".*": 0.0},
        joint_pos={
            # bend legs
            "LL1_Groin_to_LL2_Buttock_Pitch": -0.32,
            "RL1_Groin_to_RL2_Buttock_Pitch": -0.32,
            "LL3_Thigh_to_LL4_Calf_Pitch": 0.42,
            "RL3_Thigh_to_RL4_Calf_Pitch": 0.42,
            "LL4_Calf_to_LL5_ankle_Pitch": -0.30,
            "RL4_Calf_to_RL5_ankle_Pitch": -0.30,
            # bend arms 
            "torso_to_LA1_Shoulder_Pitch": -0.40,
            "torso_to_RA1_Shoulder_Pitch": 0.40, #! **
            "LA3_Upper_Arm_to_LA4_Lower_Arm_Pitch": -0.9,
            "RA3_Upper_Arm_to_RA4_Lower_Arm_Pitch_": 0.9, #! **
            # wider legs
            "LL2_Buttock_to_LL3_Thigh_Roll": 0.02, # hip roll
            "LL2_Buttock_to_LL3_Thigh_Roll": -0.02,
            "LL5_ankle_to_LL6_Foot_Roll": -0.02,
            "RL5_ankle_to_RL6_Foot_Roll": 0.02,
            "hip_to_LL1_Groin_Yaw": 0.08,
            "hip_to_RL1_Groin_Yaw": -0.08,
            # lean forward
            "abdomen_to_hip_Pitch": -0.15, #! ****

        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                "hip_to_LL1_Groin_Yaw",
                "hip_to_RL1_Groin_Yaw",
                "LL1_Groin_to_LL2_Buttock_Pitch",
                "LL2_Buttock_to_LL3_Thigh_Roll",
                "LL3_Thigh_to_LL4_Calf_Pitch",
                "RL1_Groin_to_RL2_Buttock_Pitch",
                "RL2_Buttock_to_RL3_Thigh_Roll",
                "RL3_Thigh_to_RL4_Calf_Pitch",
            ],
            effort_limit=300.0, #!follow g1
            # velocity_limit=100.0,
            # stiffness=200.0,
            stiffness={
                "hip_to_LL1_Groin_Yaw": 150.0, # hip yaw
                "hip_to_RL1_Groin_Yaw": 150.0,
                "LL1_Groin_to_LL2_Buttock_Pitch": 200.0, # hip pitch
                "RL1_Groin_to_RL2_Buttock_Pitch": 200.0,
                "LL2_Buttock_to_LL3_Thigh_Roll": 150.0, # hip roll
                "RL2_Buttock_to_RL3_Thigh_Roll": 150.0,
                "LL3_Thigh_to_LL4_Calf_Pitch": 200.0, # knee pitch
                "RL3_Thigh_to_RL4_Calf_Pitch": 200.0,
            },
            damping=5.0,
            # armature=0.01,
        ),
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[
                "LL4_Calf_to_LL5_ankle_Pitch",
                "RL4_Calf_to_RL5_ankle_Pitch",
                "LL5_ankle_to_LL6_Foot_Roll",
                "RL5_ankle_to_RL6_Foot_Roll",
            ],
            effort_limit=20.0, #! follow g1
            stiffness=20.0,
            damping=2.0,
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                "LA1_Shoulder_to_LA2_Shoulder_Roll",
                "LA2_Shoulder_to_LA3_Upper_Arm_Yaw",
                "LA3_Upper_Arm_to_LA4_Lower_Arm_Pitch",
                "LA4_Lower_Arm_to_LA5_Wrist_Yaw",
                "LA5_Wrist_to_LA6_Wrist_Pitch",
                "LA6_Wrist_to_LA7_Hand_Roll",
                "RA1_Shoulder_to_RA2_Shoulder_Roll_",
                "RA2_Shoulder_to_RA3_Upper_Arm_Yaw",
                "RA3_Upper_Arm_to_RA4_Lower_Arm_Pitch_",
                "RA4_Lower_Arm_to_RA5_Wrist_Yaw",
                "RA5_Wrist_to_RA6_Wrist_Pitch",
                "RA6_Wrist_to_RA7_Hand_Roll",
            ],
            effort_limit=300.0, #! follow g1
            # velocity_limit=100.0,
            stiffness=40.0,
            damping=10.0,
            # armature=0.01,
        ),
        "torso": ImplicitActuatorCfg(
            joint_names_expr=[
                "abdomen_to_hip_Pitch",
                "base_to_neck_yaw",
                "neck_to_torso_Pitch",
                "torso_to_LA1_Shoulder_Pitch",
                "torso_to_RA1_Shoulder_Pitch",
                "torso_to_abdomen_Yaw",
            ],
            effort_limit=300.0, #! follow arm
            velocity_limit=100.0,
            stiffness={
                "abdomen_to_hip_Pitch": 200.0,
                "base_to_neck_yaw": 40.0,
                "neck_to_torso_Pitch": 40.0,
                "torso_to_LA1_Shoulder_Pitch": 40.0,
                "torso_to_RA1_Shoulder_Pitch": 40.0,
                "torso_to_abdomen_Yaw": 40.0,
            },            
            damping={
                "abdomen_to_hip_Pitch": 5.0,
                "base_to_neck_yaw": 10.0,
                "neck_to_torso_Pitch": 10.0,
                "torso_to_LA1_Shoulder_Pitch": 10.0,
                "torso_to_RA1_Shoulder_Pitch": 10.0,
                "torso_to_abdomen_Yaw": 10.0,
            }
            # armature=0.01,
        ),
    },
)
"""Configuration for the Hanumanoid A1 robot."""