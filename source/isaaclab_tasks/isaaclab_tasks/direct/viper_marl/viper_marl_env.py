# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import math
import torch
from collections.abc import Sequence

from isaaclab.envs.mdp import observations
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.envs import DirectMARLEnv
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane
from isaaclab.utils.math import sample_uniform

from .viper_marl_env_cfg import ViperMarlEnvCfg


class ViperMarlEnv(DirectMARLEnv):
    """
    Multi-agent environment for ViPER-style pursuit-evasion.
    - Multiple pursuer robots
    - Shared occupancy grid map (cleared/contaminated)
    - Individual graph observations per agent
    - Joint/global reward shared by all agents
    """

    cfg: ViperMarlEnvCfg

    def __init__(self, cfg: ViperMarlEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        # self.num_agents = cfg.num_agents
        self.robots: list[Articulation] = []


        # shared occupancy grid map: 0 = unknown, 1 = cleared, 2 = contaminated
        H, W = cfg.map_size
        self.map_state = torch.zeros((H, W), dtype=torch.int, device=self.device)


    # ----------------------------
    # Scene setup
    # ----------------------------
    def _setup_scene(self):
        # spawn ground plane
        spawn_ground_plane(prim_path="/World/ground", cfg=GroundPlaneCfg())

        # add articulation to scene
        self.robots = []
        for i in range(self.cfg.num_agents):
            robot = Articulation(self.cfg.robot_cfg.replace(prim_path=f"/World/Robot_{i}"))
            # prim_path=f"/World/envs/env_*/Robot_{i}"
            self.scene.articulations[f"robot_{i}"] = robot
            self.robots.append(robot)

        # clone and replicate
        self.scene.clone_environments(copy_from_source=False)

        # filter collisions
        if self.device == "cpu":
            self.scene.filter_collisions(global_prim_paths=[])

        # add lights
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)


    # ----------------------------
    # Step logic
    # ----------------------------
    def _pre_physics_step(self, actions: dict[str, torch.Tensor]) -> None:
        num_envs = self.scene.num_envs
        for i in range(self.num_agents):
            a = actions.get(f"agent_{i}")
            assert a is not None, f"Missing action for agent_{i}"
            assert a.shape == (num_envs, 2), f"Bad action shape for agent_{i}: {a.shape}"
        self.actions = actions

    def _apply_action(self) -> None:
        # apply agent actions (e.g., move robot base)
        for i, robot in enumerate(self.robots):
            action = self.actions.get(f"agent_{i}")
            if action is None:
                continue
            assert action.dim() == 2 and action.shape[0] == self.scene.num_envs and action.shape[1] == 2, \
            f"Bad action shape for agent_{i}: {action.shape}"
            # TODO: map action[:, 0], action[:, 1] to velocity commands per env id

    # ----------------------------
    # Observations
    # ----------------------------
    def _get_observations(self) -> dict[str, torch.Tensor]:
        observations = {}
        num_envs = self.scene.num_envs
        for i in range(self.num_agents):
            x = self._build_graph_observation(i)
            assert x.shape == (num_envs, 80), f"obs shape wrong for agent_{i}: {x.shape}"
            observations[f"agent_{i}"] = x
        return observations

    def _build_graph_observation(self, agent_id: int) -> dict[str, torch.Tensor]:
        """
        Build ViPER-style graph observation for one agent.
        Returns:
        dict with node_features [N,F], adjacency [N,N]
        """
        # TODO: implement map scanning + node generation
        N, F = 10, 8 # placeholder sizes
        node_features = torch.zeros((N, F), device=self.device)
        adjacency = torch.eye(N, device=self.device)

        # return {
        #     "node_features": node_features,
        #     "adjacency": adjacency,
        #     }
        return node_features.flatten(start_dim=1)  # placeholder vector output
    
    def _get_rewards(self) -> dict[str, torch.Tensor]:
        num_envs = self.scene.num_envs
        delta_cleared = self._compute_cleared_delta()      # -> [num_envs]
        max_step = self._compute_max_step_distance()       # -> [num_envs]
        done_mask = self._check_completion().float()       # -> [num_envs], 0/1

        r = self.cfg.c1 * delta_cleared - self.cfg.c2 * max_step + done_mask * self.cfg.final_reward
        # Ensure shape is [num_envs]
        assert r.shape == (num_envs,), f"reward should be [num_envs], got {r.shape}"

        return {f"agent_{i}": r for i in range(self.num_agents)}

    # ----------------------------
    # Dones
    # ----------------------------
    def _get_dones(self) -> tuple[dict[str, torch.Tensor], dict[str, torch.Tensor]]:
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        done = self._check_completion()
        terminated = {f"agent_{i}": done for i in range(self.cfg.num_agents)}
        time_outs = {f"agent_{i}": time_out for i in range(self.cfg.num_agents)}
        return terminated, time_outs
    # ----------------------------
    # Reset
    # ----------------------------
    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = torch.arange(self.scene.num_envs, device=self.device)
        super()._reset_idx(env_ids)

        # reset map
        self.map_state[:] = 0

        # reset robots
        for i, robot in enumerate(self.robots):
            default_root_state = robot.data.default_root_state[env_ids]
            robot.write_root_pose_to_sim(default_root_state[:, :7], env_ids)
            robot.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids)
            
    # ----------------------------
    # Helper functions
    # ----------------------------
    def _compute_cleared_delta(self):
    # TODO: implement cleared area counting
        return torch.zeros(self.scene.num_envs, device=self.device, dtype=torch.float32)

    def _compute_max_step_distance(self):
    # TODO: compute max distance traveled by any agent
        return torch.zeros(self.scene.num_envs, device=self.device, dtype=torch.float32)

    def _check_completion(self):
    # TODO: check if map fully cleared
        return torch.zeros(self.scene.num_envs, device=self.device, dtype=torch.float32)


# @torch.jit.script
# def normalize_angle(angle):
#     return (angle + math.pi) % (2 * math.pi) - math.pi


# @torch.jit.script
# def compute_rewards(
#     rew_scale_alive: float,
#     rew_scale_terminated: float,
#     rew_scale_cart_pos: float,
#     rew_scale_cart_vel: float,
#     rew_scale_pole_pos: float,
#     rew_scale_pole_vel: float,
#     rew_scale_pendulum_pos: float,
#     rew_scale_pendulum_vel: float,
#     cart_pos: torch.Tensor,
#     cart_vel: torch.Tensor,
#     pole_pos: torch.Tensor,
#     pole_vel: torch.Tensor,
#     pendulum_pos: torch.Tensor,
#     pendulum_vel: torch.Tensor,
#     reset_terminated: torch.Tensor,
# ):
#     rew_alive = rew_scale_alive * (1.0 - reset_terminated.float())
#     rew_termination = rew_scale_terminated * reset_terminated.float()
#     rew_pole_pos = rew_scale_pole_pos * torch.sum(torch.square(pole_pos).unsqueeze(dim=1), dim=-1)
#     rew_pendulum_pos = rew_scale_pendulum_pos * torch.sum(
#         torch.square(pole_pos + pendulum_pos).unsqueeze(dim=1), dim=-1
#     )
#     rew_cart_vel = rew_scale_cart_vel * torch.sum(torch.abs(cart_vel).unsqueeze(dim=1), dim=-1)
#     rew_pole_vel = rew_scale_pole_vel * torch.sum(torch.abs(pole_vel).unsqueeze(dim=1), dim=-1)
#     rew_pendulum_vel = rew_scale_pendulum_vel * torch.sum(torch.abs(pendulum_vel).unsqueeze(dim=1), dim=-1)

#     total_reward = {
#         "cart": rew_alive + rew_termination + rew_pole_pos + rew_cart_vel + rew_pole_vel,
#         "pendulum": rew_alive + rew_termination + rew_pendulum_pos + rew_pendulum_vel,
#     }
#     return total_reward