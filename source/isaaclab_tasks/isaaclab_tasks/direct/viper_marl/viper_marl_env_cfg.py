# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab_assets.robots.cart_double_pendulum import CART_DOUBLE_PENDULUM_CFG

from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectMARLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass

from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG

@configclass
class ViperMarlEnvCfg(DirectMARLEnvCfg):
    # env
    decimation = 2
    episode_length_s = 30.0

    # multi-agent specification and spaces definition
    num_agents = 3 # number of pursuers
    possible_agents = [f"agent_{i}" for i in range(num_agents)]

    # action and observation spaces
    action_spaces = {agent: 2 for agent in possible_agents}  # 3D move in x,y
    observation_spaces = {agent: 80 for agent in possible_agents} # placeholder vector size # graph obs handled by custom encoder
    state_space = 80  # global state not used

    # map parameters
    map_size = (100, 100)
    rfov = 10.0

    # simulation
    sim: SimulationCfg = SimulationCfg(dt=1 / 60, render_interval=decimation)

    # robot(s)
    robot_cfg: ArticulationCfg = UNITREE_GO2_CFG.replace(prim_path="/World/envs/env_*/Robot")
    # scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=1024, 
        env_spacing=4.0, 
        replicate_physics=True)

    # - reward scales
    c1 = 1.0         # reward scale for cleared area
    c2 = 0.1         # penalty for distance
    final_reward = 100.0

    # - reset states/conditions
    max_episode_length = 500