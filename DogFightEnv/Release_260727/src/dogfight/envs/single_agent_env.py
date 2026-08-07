from __future__ import annotations

import copy
import datetime
import json
import math
import os
from typing import Dict, List, Optional, Tuple

import numpy as np
import pymap3d as pm
try:
    import gymnasium as gym
except ImportError:  # pragma: no cover - compatibility fallback for local setups
    import gym as gym

import FighterSim
import JSBSimWrapper
from GeoMathUtil import GeometryInfo
from dogfight.ai.action_provider import ActionContext
from dogfight.ai.native_bt import AIPilot
from dogfight.config import FEET_TO_METER, METER_TO_FEET, merge_env_config
from dogfight.envs.initial_scenario import sample_scenario_pool
from dogfight.envs.observation import (
    build_observation,
    normalize,
    observation_size as builtin_observation_size,
)
from dogfight.envs.reward import compute_reward
from dogfight.envs.termination import evaluate_termination
from dogfight.sim.state_schema import StateIndex


REF_OLD_RANDOM_SCENARIOS = {
    0: (
        [1000.0, 0.0, -8100.0, 0.0, 0.0, 0.0, 250.0, 2],
        [3000.0, 0.0, -8000.0, 0.0, 0.0, 0.0, 250.0, 2],
    ),
    1: (
        [1500.0, 500.0, -8500.0, 0.0, 0.0, 0.0, 250.0, 2],
        [3000.0, 0.0, -8000.0, 0.0, 0.0, 0.0, 250.0, 2],
    ),
    2: (
        [3000.0, 1000.0, -8000.0, 0.0, 0.0, 270.0, 250.0, 2],
        [3000.0, 0.0, -8000.0, 0.0, 0.0, 0.0, 250.0, 2],
    ),
    3: (
        [3000.0, 0.0, -8000.0, 0.0, 0.0, 0.0, 250.0, 2],
        [2000.0, 1000.0, -8000.0, 0.0, 0.0, 315.0, 250.0, 2],
    ),
    4: (
        [3000.0, 0.0, -8000.0, 0.0, 0.0, 0.0, 250.0, 2],
        [2000.0, 0.0, -8000.0, 0.0, 0.0, 0.0, 250.0, 2],
    ),
    5: (
        [3000.0, 1000.0, -8000.0, 0.0, 0.0, 0.0, 250.0, 1],
        [4000.0, 0.0, -8000.0, 0.0, 0.0, 0.0, 250.0, 1],
    ),
    6: (
        [3000.0, 1000.0, -8000.0, 0.0, 0.0, 0.0, 250.0, 1],
        [4000.0, 0.0, -8000.0, 0.0, 0.0, 0.0, 250.0, 1],
    ),
    7: (
        [3000.0, 1000.0, -8000.0, 0.0, 0.0, 0.0, 250.0, 1],
        [4000.0, 0.0, -8000.0, 0.0, 0.0, 0.0, 250.0, 1],
    ),
}


class DogFightEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(
        self,
        env_config: Optional[dict] = None,
        AIP_ownship=None,
        AIP_target=None,
        ownship_action_provider=None,
        target_action_provider=None,
        reward_fn=None,  # 학생 정의 보상 함수; None이면 reward.py 기본값 사용
        observation_fn=None,
        observation_size=None,
        observation_low=None,
        observation_high=None,
    ):
        super().__init__()
        self._closed = False
        self.battle_space_id: int | None = None
        self._sim = None
        self._target_sim = None
        self._ownship_ai = None
        self._target_ai = None
        self._ownship_action_provider = ownship_action_provider
        self._target_action_provider = target_action_provider

        self.config = merge_env_config(env_config)
        self._base_target_mode = str(self.config["target_mode"])
        self._base_target_loiter = copy.deepcopy(self.config["target_loiter"])
        self._base_target_autopilot = copy.deepcopy(
            self.config["target_autopilot"]
        )
        self._runner_index = self.config.get("_runner_index", "local")
        self._env_index = self.config.get("_env_index", 0)
        self._geo_info = GeometryInfo()
        self._sim_hz = int(self.config["sim_hz"])
        self._step_ratio = self._resolve_step_ratio(self.config)
        self._delta_t = 1.0 / self._sim_hz
        self._max_engage_time = float(self.config["max_engage_time"])
        self._min_altitude = float(self.config["min_altitude"])
        self._observation_mode = self.config["observation_mode"]
        self._episode_step_limit = self.config.get("episode_step_limit")
        self._geometry_guard = self.config.get("geometry_guard", {})
        self._wez = self.config["wez"]
        self._reward_config = self.config["reward"]
        self._artifacts_dir = self.config["artifacts_dir"]
        self._reward_fn = reward_fn  # None → compute_reward from reward.py
        self._observation_fn = observation_fn

        self.battle_space_id = JSBSimWrapper.create_battleSpace()
        self._ownship_ai = (
            AIP_ownship if AIP_ownship is not None else self._build_ownship_ai()
        )
        self._target_ai = (
            AIP_target if AIP_target is not None else self._build_target_ai()
        )

        self.num_action = 4
        self.num_observation = (
            int(observation_size)
            if observation_fn is not None
            else builtin_observation_size(self._observation_mode)
        )
        if observation_fn is not None:
            low = -1.0 if observation_low is None else observation_low
            high = 1.0 if observation_high is None else observation_high
            self.observation_space = gym.spaces.Box(
                low=np.full(self.num_observation, low, dtype=np.float32)
                if np.isscalar(low) else np.asarray(low, dtype=np.float32),
                high=np.full(self.num_observation, high, dtype=np.float32)
                if np.isscalar(high) else np.asarray(high, dtype=np.float32),
                shape=(self.num_observation,),
                dtype=np.float32,
            )
        elif self._observation_mode == "tactical16":
            self.observation_space = gym.spaces.Box(
                low=-1.0, high=1.0, shape=(self.num_observation,), dtype=np.float32
            )
        else:
            self.observation_space = gym.spaces.Box(
                low=-np.inf, high=np.inf, shape=(self.num_observation,), dtype=np.float32
            )
        # All 4 action dims in [-1, 1] (throttle remapped to [0,1] internally).
        # Untrained networks output ≈0 → throttle = (0+1)/2 = 0.5 → no stall.
        self.action_space = gym.spaces.Box(
            low=-np.ones(self.num_action, dtype=np.float32),
            high=np.ones(self.num_action, dtype=np.float32),
            shape=(self.num_action,),
            dtype=np.float32,
        )

        self._sim = FighterSim.JSBSim(
            [
                1,
                1,
                self.config["ownship"][0],
                self.config["ownship"][1],
                self.config["ownship"][2],
                self.config["ownship"][3],
                self.config["ownship"][4],
                self.config["ownship"][5],
                self.config["ownship"][6],
            ],
            self._ownship_ai,
            self._sim_hz,
            self.battle_space_id,
        )
        self._target_sim = FighterSim.JSBSim(
            [
                1,
                2,
                self.config["target"][0],
                self.config["target"][1],
                self.config["target"][2],
                self.config["target"][3],
                self.config["target"][4],
                self.config["target"][5],
                self.config["target"][6],
            ],
            self._target_ai,
            self._sim_hz,
            self.battle_space_id,
        )

        self.ownship_log: List[List[float]] = []
        self.target_log: List[List[float]] = []
        self.info: Dict[str, object] = {"end_condition": ""}
        self.ownship_damage = 0.0
        self.target_damage = 0.0
        self.num_engage = 0
        self.current_timestep = 0
        self.pre_obs = np.zeros(self.num_observation, dtype=np.float32)
        # Episode-level accumulators (reset in reset())
        self._in_wez = False
        self._ep_wez_steps = 0
        self._ep_step_count = 0
        self._ep_distance_sum = 0.0
        self._ep_distance_min = float("inf")
        self._ep_altitude_penalty_steps = 0
        self._ep_total_reward = 0.0
        self._ep_reward_components: Dict[str, float] = {}
        self._ep_action_sum = np.zeros(self.num_action, dtype=np.float64)
        self._ep_action_sq_sum = np.zeros(self.num_action, dtype=np.float64)
        self._initial_scenario_metrics: Dict[str, object] = {}

    # Sim expects throttle in [0, 1]; RL policy outputs throttle in [-1, 1].
    _SIM_ACTION_LOW  = np.array([-1., -1., -1., 0.], dtype=np.float32)
    _SIM_ACTION_HIGH = np.ones(4, dtype=np.float32)

    def _to_sim_action(self, rl_action: np.ndarray) -> np.ndarray:
        """Convert RL action space [-1,1]^4 → simulator format (throttle [0,1])."""
        a = np.clip(rl_action, -1.0, 1.0).astype(np.float32)
        a[3] = (a[3] + 1.0) / 2.0                        # [-1,1] → [0,1]
        return np.clip(a, self._SIM_ACTION_LOW, self._SIM_ACTION_HIGH)

    def _build_ai(self, dll_name):
        if not dll_name:
            return None
        return AIPilot(dll_name)

    def _build_ownship_ai(self):
        if self._ownship_action_provider is not None:
            return None
        if self.config["ownship_control_mode"] != "behavior_tree":
            return None
        return self._build_ai(self.config["ownship_behavior_dll"])

    def _build_target_ai(self):
        if self._target_action_provider is not None:
            return None
        if self.config["target_mode"] != "behavior_tree":
            return None
        return self._build_ai(self.config["target_behavior_dll"])

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        super().reset(seed=seed)
        if options:
            self.config = merge_env_config({**self.config, **options})
            if "target_mode" in options:
                self._base_target_mode = str(self.config["target_mode"])
            if "target_loiter" in options:
                self._base_target_loiter = copy.deepcopy(self.config["target_loiter"])
            if "target_autopilot" in options:
                self._base_target_autopilot = copy.deepcopy(
                    self.config["target_autopilot"]
                )
            self._episode_step_limit = self.config.get("episode_step_limit")
            self._geometry_guard = self.config.get("geometry_guard", {})
            self._reward_config = self.config["reward"]

        self._restore_target_behavior_config()
        scenario = self.config.get("initial_scenario", {})
        scenario_mode = scenario.get("mode", "default")
        if scenario_mode == "scenario_pool":
            self._apply_scenario_pool_initial_scenario(scenario)
        elif scenario_mode == "two_circle_headon":
            self._apply_two_circle_headon_initial_scenario(scenario)
        elif scenario_mode == "ref_old_random":
            self._apply_ref_old_random_initial_scenario(scenario)
        else:
            self._restore_default_initial_positions()
            self._initial_scenario_metrics = {}

        # Apply per-episode position randomization if configured
        rand = self.config.get("ownship_randomization", {})
        generated_modes = {"scenario_pool", "two_circle_headon", "ref_old_random"}
        if scenario_mode not in generated_modes and rand.get("enabled", False):
            self.add_random_init_position(
                "ownship",
                radius=float(rand.get("radius", 0)),
                r_roll=float(rand.get("r_roll", 0)),
                r_pitch=float(rand.get("r_pitch", 0)),
                r_heading=float(rand.get("r_heading", 0)),
            )

        JSBSimWrapper.Reset(self.battle_space_id)
        self._ownship_state = self._sim.reset()
        self._target_state = self._target_sim.reset()
        self._update_initial_geometry_metrics(scenario_mode)
        self.pre_obs = self.get_observation()
        self.info = {"end_condition": "", **self._initial_scenario_metrics}
        self.ownship_damage = 0.0
        self.target_damage = 0.0
        self.num_engage += 1
        self.current_timestep = 0
        self._reset_action_providers()
        # Reset episode accumulators
        self._in_wez = False
        self._ep_wez_steps = 0
        self._ep_step_count = 0
        self._ep_distance_sum = 0.0
        self._ep_distance_min = float("inf")
        self._ep_altitude_penalty_steps = 0
        self._ep_total_reward = 0.0
        self._ep_reward_components = {}
        self._ep_action_sum = np.zeros(self.num_action, dtype=np.float64)
        self._ep_action_sq_sum = np.zeros(self.num_action, dtype=np.float64)
        return np.array(self.pre_obs, dtype=np.float32), dict(self.info)

    def step(self, action) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        action = np.asarray(action, dtype=np.float32)
        failure = self._advance_simulation_step_ratio(action)
        if failure is not None:
            return failure
        cur_obs = self.get_observation()

        terminated, truncated, end_condition = evaluate_termination(
            self._ownship_state,
            self._target_state,
            self._sim,
            self._target_sim,
            self._max_engage_time,
            self._min_altitude,
            self.current_timestep,
            self._episode_step_limit,
            self._geo_info,
            self._geometry_guard,
        )

        ownship_health = float(self._ownship_state[StateIndex.HEALTH])
        target_health = float(self._target_state[StateIndex.HEALTH])
        outcome = self._classify_outcome(
            terminated,
            truncated,
            end_condition,
            ownship_health,
            target_health,
        )
        reward, components = self._compute_step_reward(
            terminated,
            truncated,
            end_condition,
        )

        ep_mean_dist, ep_min_dist = self._update_episode_metrics(
            action,
            reward,
            components,
        )

        self.info = {
            "end_condition": end_condition,
            "outcome": outcome,
            "ownship_damage": self.ownship_damage,
            "target_damage": self.target_damage,
            "ownship_health": ownship_health,
            "target_health": target_health,
            "reward_components": components,
            "ep_reward_components": dict(self._ep_reward_components),
            "ep_wez_steps": self._ep_wez_steps,
            "ep_step_count": self._ep_step_count,
            "ep_mean_distance": ep_mean_dist,
            "ep_min_distance": ep_min_dist,
            "ep_altitude_penalty_steps": self._ep_altitude_penalty_steps,
            "final_ata_deg": abs(float(self._geo_info._get_antenna_train_angle(
                self._ownship_state, self._target_state, True
            ))),
            "final_aa_deg": abs(float(self._geo_info._get_aspect_angle(
                self._ownship_state, self._target_state, True
            ))),
            "headon_guard_fail": end_condition == "two circle headon guard fail",
            **self._initial_scenario_metrics,
        }

        self._append_logs()
        self.pre_obs = copy.deepcopy(cur_obs)
        self.current_timestep += 1
        if terminated or truncated:
            self._print_episode_termination(end_condition)
        return np.array(cur_obs, dtype=np.float32), reward, terminated, truncated, dict(self.info)

    def _advance_simulation_step_ratio(
        self,
        action: np.ndarray,
    ) -> tuple[np.ndarray, float, bool, bool, Dict] | None:
        ownship_damage_total = 0.0
        target_damage_total = 0.0
        in_wez_any = False
        for _ in range(int(self._step_ratio)):
            self._step_controlled_aircraft(action)
            if np.isnan(self._sim.get_state()).any():
                info = {"end_condition": "Ownship FDM output Fall", "outcome": "crash"}
                self._ep_step_count += 1
                self._print_episode_termination(info["end_condition"])
                return np.array(self.pre_obs, dtype=np.float32), 0.0, True, False, info

            self._step_target_aircraft()
            if np.isnan(self._target_sim.get_state()).any():
                info = {"end_condition": "Target FDM output Fall", "outcome": "other"}
                self._ep_step_count += 1
                self._print_episode_termination(info["end_condition"])
                return np.array(self.pre_obs, dtype=np.float32), 0.0, True, False, info

            self.update_damage()
            ownship_damage_total += float(self.ownship_damage)
            target_damage_total += float(self.target_damage)
            in_wez_any = in_wez_any or self._in_wez
            self._ownship_state = self._sim.get_state()
            self._target_state = self._target_sim.get_state()

        self.ownship_damage = ownship_damage_total
        self.target_damage = target_damage_total
        self._in_wez = in_wez_any
        return None

    @staticmethod
    def _classify_outcome(
        terminated: bool,
        truncated: bool,
        end_condition: str,
        ownship_health: float,
        target_health: float,
    ) -> str:
        if terminated:
            if target_health <= 0.0 < ownship_health:
                return "win"
            if ownship_health <= 0.0 < target_health:
                return "loss"
            if end_condition in ("ownship altitude below min", "FDM Update Fail"):
                return "crash"
            if end_condition == "two circle headon guard fail":
                return "loss"
            return "draw"
        if truncated:
            # 08-04: 규정집 v1.4 제6조2항 — 라운드 시간만료 시 잔여 HP비율 높은 쪽 승리,
            # 동률이면 무승부. 기존엔 이 비교 없이 그냥 "timeout"으로만 분류해서 실제로는
            # 승리였을 경기가 전부 무승부로 잘못 집계되고 있었음(실측으로 발견: HP 0.63~0.74
            # vs 1.0인 경기들이 "timeout"으로만 나옴).
            if ownship_health > target_health:
                return "win"
            if target_health > ownship_health:
                return "loss"
            return "draw"
        return "ongoing"

    def _compute_step_reward(
        self,
        terminated: bool,
        truncated: bool,
        end_condition: str,
    ) -> tuple[float, dict]:
        _reward_fn = self._reward_fn if self._reward_fn is not None else compute_reward
        return _reward_fn(
            self._ownship_state,
            self._target_state,
            self.ownship_damage,
            self.target_damage,
            self._geo_info,
            self._wez,
            self._reward_config,
            terminated,
            truncated,
            end_condition,
        )

    def _update_episode_metrics(
        self,
        action: np.ndarray,
        reward: float,
        components: dict,
    ) -> tuple[float, float]:
        distance = self._geo_info._get_distance(self._ownship_state, self._target_state)
        self._ep_step_count += 1
        self._ep_total_reward += float(reward)
        self._ep_distance_sum += distance
        self._ep_distance_min = min(self._ep_distance_min, distance)
        if self._in_wez:
            self._ep_wez_steps += 1
        if components.get("safety", 0.0) < 0.0:
            self._ep_altitude_penalty_steps += 1
        for k, v in components.items():
            self._ep_reward_components[k] = self._ep_reward_components.get(k, 0.0) + v
        self._ep_action_sum += action.astype(np.float64)
        self._ep_action_sq_sum += action.astype(np.float64) ** 2

        ep_mean_dist = self._ep_distance_sum / self._ep_step_count
        ep_min_dist = self._ep_distance_min if self._ep_step_count > 0 else 0.0
        return ep_mean_dist, ep_min_dist

    @staticmethod
    def _resolve_step_ratio(config: dict) -> float:
        """Resolve RL-action hold ratio from explicit or legacy timing config."""
        ratio = config.get("step_ratio")
        if ratio is None:
            delta = config.get("delta")
            time_step = config.get("time_step")
            if delta is not None and time_step is not None:
                ratio = float(delta) / float(time_step)
            else:
                ratio = 1.0
        ratio = float(ratio)
        if int(ratio) <= 0:
            raise ValueError(f"step_ratio must resolve to at least 1, got {ratio}")
        return ratio

    def _step_controlled_aircraft(self, action: np.ndarray) -> None:
        if self._ownship_action_provider is not None:
            context = self._build_action_context(
                self._sim,
                self._target_sim,
                self._ownship_state,
                self._target_state,
                self.pre_obs,
            )
            result = self._ownship_action_provider.compute_action(context)
            self._sim.step(result.action)
            return

        control_mode = self.config["ownship_control_mode"]
        if control_mode == "behavior_tree":
            self._sim.step_behavior(self._target_sim.get_model())
        elif control_mode == "fixed":
            self._sim.step_fix()
        elif control_mode == "loiter":
            loiter = self.config["target_loiter"]
            self._sim.step_loiter(loiter["enabled"], loiter["bank"], loiter["pitch"])
        else:
            self._sim.step(self._to_sim_action(action))

    def _step_target_aircraft(self) -> None:
        if self._target_action_provider is not None:
            context = self._build_action_context(
                self._target_sim,
                self._sim,
                self._target_state,
                self._ownship_state,
                self.pre_obs,
            )
            result = self._target_action_provider.compute_action(context)
            self._target_sim.step(result.action)
            return

        target_mode = self.config["target_mode"]
        if target_mode == "behavior_tree":
            self._target_sim.step_behavior(self._sim.get_model())
        elif target_mode == "fixed":
            self._target_sim.step_fix()
        elif target_mode == "loiter":
            loiter = self.config["target_loiter"]
            self._target_sim.step_loiter(loiter["enabled"], loiter["bank"], loiter["pitch"])
        elif target_mode == "autopilot":
            autopilot = self.config["target_autopilot"]
            self._target_sim.step_autopilot(
                autopilot["heading_cmd"],
                autopilot["altitude_cmd"],
                autopilot["speed_cmd"],
            )
        else:
            self._target_sim.step_fix()

    def get_observation(self):
        return self._build_observation_for(self._ownship_state, self._target_state)

    def _build_observation_for(self, ownship_state, target_state) -> np.ndarray:
        if self._observation_fn is not None:
            observation = self._observation_fn(
                np.array(ownship_state, copy=True),
                np.array(target_state, copy=True),
                self._geo_info,
                self._wez,
            )
            observation = np.asarray(observation, dtype=np.float32)
            if observation.shape != (self.num_observation,):
                raise ValueError(
                    "custom observation_fn returned shape "
                    f"{observation.shape}, expected {(self.num_observation,)}"
                )
            return observation

        wez_cfg = self._wez if self._observation_mode == "tactical16" else None
        return build_observation(
            self._observation_mode,
            ownship_state,
            target_state,
            self._geo_info,
            wez_cfg,
        )

    def get_reward(self):
        terminated, truncated, end_condition = evaluate_termination(
            self._ownship_state,
            self._target_state,
            self._sim,
            self._target_sim,
            self._max_engage_time,
            self._min_altitude,
            self.current_timestep,
            self._episode_step_limit,
            self._geo_info,
            self._geometry_guard,
        )
        _reward_fn = self._reward_fn if self._reward_fn is not None else compute_reward
        result = _reward_fn(
            self._ownship_state,
            self._target_state,
            self.ownship_damage,
            self.target_damage,
            self._geo_info,
            self._wez,
            self._reward_config,
            terminated,
            truncated,
            end_condition,
        )
        return result[0] if isinstance(result, tuple) else result

    def get_done(self):
        return evaluate_termination(
            self._ownship_state,
            self._target_state,
            self._sim,
            self._target_sim,
            self._max_engage_time,
            self._min_altitude,
            self.current_timestep,
            self._episode_step_limit,
            self._geo_info,
            self._geometry_guard,
        )[:2]

    def _print_episode_termination(self, end_condition: str) -> None:
        print(
            f"runner:{self._runner_index}/env:{self._env_index}\t"
            f"num_steps=[{self._ep_step_count}] | "
            f"total rewards=[{self._ep_total_reward:.4f}] | "
            f"termination= [{end_condition}]",
            flush=True,
        )

    # 07-30: 로컬 harness는 원래 Phase1(<1도)만 구현했음(project_topgun_rules 메모리 참고).
    # 실제 대회 서버는 Phase1/2/3을 전부 평가(우선순위 Phase1>2>3, 계수 1.0/0.3/0.1) -- 로컬
    # 테스트가 Phase2/3에서 나는 부분점수를 전혀 못 보니 그쪽을 겨냥한 트리 실험(예: HABFM 각도
    # 컷오프 조정)이 로컬에서 아무 신호도 안 남는 문제가 있었음. Phase1 판정/데미지는 원래 그대로
    # 두고, Phase1이 안 맞을 때만 Phase2/3을 폴백으로 추가.
    # Phase1과 동일 관례: 규정에 적힌 각도값을 ata_deg에 직접 비교(추가로 반띵 안 함) --
    # Phase1도 angle_deg(2.0)/2=1.0을 "LOS<1도" 그대로에 직접 비교하는 방식이라 이를 따름.
    # 08-06 중대 정정: Phase2/3은 "항상"이 아니라 **교전 경과시간**으로 열린다.
    # 출처는 대회 오리엔테이션 PPT 슬라이드 7(팀원 ryujan이 원본 대조해 확인, 커밋 eeecad5).
    #   Phase1 t>=0s   LOS<1도, 500~3000ft, 계수 1.0
    #   Phase2 t>=100s LOS<2도, 500~3500ft, 계수 0.3
    #   Phase3 t>=150s LOS<3도, 500~4000ft, 계수 0.1
    # 기존 구현은 시간 게이트가 없어 0초부터 Phase2/3 부분점수를 줬음 -> 초반 100초의
    # "빗맞은 조준"이 전부 득점으로 잡혀 양쪽 데미지가 과대계상됐다. 콘 각도/사거리/계수
    # 자체는 공식값과 일치했으므로 게이트만 추가한다.
    _WEZ_PHASES_FT = (
        (2.0, 500.0, 3500.0),  # Phase2: LOS<2deg, 500-3500ft, coef0.3
        (3.0, 500.0, 4000.0),  # Phase3: LOS<3deg, 500-4000ft, coef0.1
    )
    _WEZ_PHASE_COEFS = (0.3, 0.1)
    _WEZ_PHASE_START_S = (100.0, 150.0)

    def _phase23_damage(self, dis_m: float, ata_deg: float, elapsed_s: float) -> float:
        for (los_limit_deg, min_ft, max_ft), coef, start_s in zip(
            self._WEZ_PHASES_FT, self._WEZ_PHASE_COEFS, self._WEZ_PHASE_START_S
        ):
            if elapsed_s < start_s:
                continue
            min_m = min_ft * FEET_TO_METER
            max_m = max_ft * FEET_TO_METER
            if min_m <= dis_m <= max_m and abs(ata_deg) <= los_limit_deg:
                return coef * (max_m - dis_m) / (max_m - min_m)
        return 0.0

    def update_damage(self):
        sim_state = self._sim.get_state()
        target_sim_state = self._target_sim.get_state()
        dis_m = self._geo_info._get_distance(sim_state, target_sim_state)
        ownship_ata_deg = self._geo_info._get_antenna_train_angle(sim_state, target_sim_state, False)
        target_ata_deg = self._geo_info._get_antenna_train_angle(target_sim_state, sim_state, False)

        max_range_m = self._wez["max_range_m"]
        min_range_m = self._wez["min_range_m"]
        base_range_m = max_range_m - min_range_m
        half_wez_angle_deg = self._wez["angle_deg"] / 2.0

        target_damage = 0.0
        ownship_damage = 0.0
        if base_range_m != 0 and min_range_m <= dis_m <= max_range_m:
            if half_wez_angle_deg >= abs(ownship_ata_deg):
                target_damage = ((max_range_m - dis_m) / base_range_m) * self._delta_t
            if half_wez_angle_deg >= abs(target_ata_deg):
                ownship_damage = ((max_range_m - dis_m) / base_range_m) * self._delta_t

        # Phase1 안 맞았을 때만 Phase2/3 폴백 (규정 우선순위 Phase1>2>3 반영).
        # Phase2/3은 경과시간 게이트를 통과해야 열린다(위 _WEZ_PHASE_START_S 참고).
        elapsed_s = self._ep_step_count * self._delta_t
        if target_damage == 0.0:
            target_damage = self._phase23_damage(dis_m, ownship_ata_deg, elapsed_s) * self._delta_t
        if ownship_damage == 0.0:
            ownship_damage = self._phase23_damage(dis_m, target_ata_deg, elapsed_s) * self._delta_t

        self.ownship_damage = ownship_damage
        self.target_damage = target_damage
        self._in_wez = target_damage > 0.0  # True when ownship is inside WEZ toward target
        self._sim.deduct_health(ownship_damage)
        self._target_sim.deduct_health(target_damage)

    def change_init_position(
        self,
        flight="ownship",
        init_n=0,
        init_e=0,
        init_d=-8000,
        init_roll=0,
        init_pitch=0,
        init_heading=0,
        init_speed=250,
        target_type=2,
    ):
        fighter = self._sim if flight == "ownship" else self._target_sim
        self._target_type = target_type
        fighter._init_pos_n = init_n
        fighter._init_pos_e = init_e
        fighter._init_pos_d = init_d
        fighter._init_roll = init_roll
        fighter._init_pitch = init_pitch
        fighter._init_heading = init_heading
        fighter._init_speed = init_speed
        lla = pm.ned2geodetic(
            fighter._init_pos_n,
            fighter._init_pos_e,
            fighter._init_pos_d,
            fighter._origin_lat,
            fighter._origin_lon,
            fighter._origin_alt,
        )
        fighter._init_pos_lat = lla[0]
        fighter._init_pos_lon = lla[1]
        fighter._init_pos_alt = lla[2]

    def _restore_target_behavior_config(self) -> None:
        """Restore target controls before a scenario applies episode overrides."""
        self.config["target_mode"] = self._base_target_mode
        self.config["target_loiter"] = copy.deepcopy(self._base_target_loiter)
        self.config["target_autopilot"] = copy.deepcopy(
            self._base_target_autopilot
        )

    def _restore_default_initial_positions(self) -> None:
        """Restore configured baseline states before legacy randomization."""
        self._set_initial_aircraft_state("ownship", self.config["ownship"])
        self._set_initial_aircraft_state("target", self.config["target"])

    def _set_initial_aircraft_state(self, flight: str, state: List[float]) -> None:
        """Apply a validated [N, E, D, roll, pitch, heading, speed] vector."""
        if len(state) != 7:
            raise ValueError(
                f"{flight} initial state must contain "
                "[N, E, D, roll, pitch, heading, speed]"
            )
        self.change_init_position(
            flight,
            init_n=float(state[0]),
            init_e=float(state[1]),
            init_d=float(state[2]),
            init_roll=float(state[3]),
            init_pitch=float(state[4]),
            init_heading=float(state[5]),
            init_speed=float(state[6]),
        )

    def _apply_scenario_pool_initial_scenario(self, scenario: dict) -> None:
        """Sample and apply one user-defined weighted initial scenario."""
        sampled, scenario_index = sample_scenario_pool(scenario, self.np_random)
        if self.config.get("scenario_pool_apply_target_mode", True):
            target_mode = str(
                sampled.get("target_mode", self._base_target_mode)
            )
        else:
            target_mode = self._base_target_mode
        if (
            target_mode == "behavior_tree"
            and self._target_action_provider is None
            and self._target_ai is None
        ):
            raise RuntimeError(
                "scenario_pool selected target_mode='behavior_tree', but the "
                "environment was not created with a behavior-tree target. Set "
                "the stage/base target_mode to 'behavior_tree' when any pooled "
                "scenario uses that mode."
            )
        self.config["target_mode"] = target_mode
        if "target_loiter" in sampled:
            if not isinstance(sampled["target_loiter"], dict):
                raise ValueError("scenario target_loiter must be a mapping")
            self.config["target_loiter"].update(sampled["target_loiter"])
        if "target_autopilot" in sampled:
            if not isinstance(sampled["target_autopilot"], dict):
                raise ValueError("scenario target_autopilot must be a mapping")
            self.config["target_autopilot"].update(sampled["target_autopilot"])

        self._set_initial_aircraft_state("ownship", sampled["ownship"])
        self._set_initial_aircraft_state("target", sampled["target"])
        self._initial_scenario_metrics = {
            "initial_scenario_index": float(scenario_index),
            "initial_scenario_name": sampled["name"],
        }

    def _apply_two_circle_headon_initial_scenario(self, scenario: dict) -> None:
        """Place both aircraft on the paper's two-circle head-on curriculum."""
        alpha_deg = float(scenario.get("alpha_deg", 0.0))
        alpha_rad = math.radians(alpha_deg)
        turn_diameter_ft = float(scenario.get("turn_diameter_ft", 6000.0))
        jitter_min_ft, jitter_max_ft = self._range_values(
            scenario.get("separation_jitter_ft", [3000.0, 6000.0])
        )
        jitter_ft = float(self.np_random.uniform(jitter_min_ft, jitter_max_ft))
        separation_m = (
            2.0 * turn_diameter_ft * math.sin(alpha_rad) + jitter_ft
        ) * FEET_TO_METER

        center_n = float(scenario.get("center_n_m", 3500.0))
        center_e = float(scenario.get("center_e_m", 0.0))
        altitude_m = float(scenario.get("altitude_m", 7000.0))
        half_sep = separation_m / 2.0

        speed_min, speed_max = self._range_values(
            scenario.get("speed_mps_range", [250.0, 300.0])
        )
        roll_min, roll_max = self._range_values(
            scenario.get("roll_range_deg", [0.0, 180.0])
        )
        side = float(self.np_random.choice(scenario.get("side_choices", [-1.0, 1.0])))
        pitch = float(self.np_random.choice(
            scenario.get("vertical_pitch_choices_deg", [0.0, 10.0, -10.0])
        ))

        own_heading = self._wrap_heading(side * alpha_deg)
        target_heading = self._wrap_heading(180.0 + side * alpha_deg)
        own_roll = float(self.np_random.uniform(roll_min, roll_max))
        target_roll = float(self.np_random.uniform(roll_min, roll_max))
        own_speed = float(self.np_random.uniform(speed_min, speed_max))
        target_speed = float(self.np_random.uniform(speed_min, speed_max))

        self.change_init_position(
            "ownship",
            init_n=center_n - half_sep,
            init_e=center_e,
            init_d=-altitude_m,
            init_roll=own_roll,
            init_pitch=pitch,
            init_heading=own_heading,
            init_speed=own_speed,
        )
        self.change_init_position(
            "target",
            init_n=center_n + half_sep,
            init_e=center_e,
            init_d=-altitude_m,
            init_roll=target_roll,
            init_pitch=-pitch,
            init_heading=target_heading,
            init_speed=target_speed,
        )

        self._initial_scenario_metrics = {
            "initial_alpha_deg": alpha_deg,
            "initial_distance_m": separation_m,
        }

    def _apply_ref_old_random_initial_scenario(self, scenario: dict) -> None:
        """Apply ref_oldDogFightEnv 1vs1 mixed BT/loiter initial scenarios."""
        indices = list(scenario.get("legacy_scenario_indices", [0]))
        if scenario.get("legacy_use_first_scenario_only", False):
            scenario_index = int(indices[0])
        elif scenario.get("legacy_use_random_scenario", True):
            scenario_index = int(self.np_random.choice(indices))
        else:
            scenario_index = int(scenario.get("legacy_scenario_index", indices[0]))
        ownship, target = REF_OLD_RANDOM_SCENARIOS[scenario_index]

        self.change_init_position(
            "ownship",
            init_n=ownship[0],
            init_e=ownship[1],
            init_d=ownship[2],
            init_roll=ownship[3],
            init_pitch=ownship[4],
            init_heading=ownship[5],
            init_speed=ownship[6],
            target_type=ownship[7],
        )
        self.change_init_position(
            "target",
            init_n=target[0],
            init_e=target[1],
            init_d=target[2],
            init_roll=target[3],
            init_pitch=target[4],
            init_heading=target[5],
            init_speed=target[6],
            target_type=target[7],
        )

        randomization = scenario.get("legacy_randomization", {})
        self.add_random_init_position(
            "ownship",
            radius=float(randomization.get("aircraft_radius_m", 100.0)),
            r_roll=float(randomization.get("roll_deg", 5.0)),
            r_pitch=float(randomization.get("pitch_deg", 5.0)),
            r_heading=float(randomization.get("heading_deg", 5.0)),
        )
        self.add_random_init_position(
            "target",
            radius=float(randomization.get("aircraft_radius_m", 100.0)),
            r_roll=float(randomization.get("roll_deg", 5.0)),
            r_pitch=float(randomization.get("pitch_deg", 5.0)),
            r_heading=float(randomization.get("heading_deg", 5.0)),
        )
        self._add_ref_old_shared_random_starting_position(randomization)

        if int(target[7]) == 1:
            bank_min, bank_max = self._range_values(
                randomization.get("loiter_bank_deg_range", [40.0, 70.0])
            )
            bank = float(
                self.np_random.choice([-1.0, 1.0])
                * self.np_random.uniform(bank_min, bank_max)
            )
            self.config["target_mode"] = "loiter"
            self.config["target_loiter"] = {
                "enabled": True,
                "bank": bank,
                "pitch": 0.0,
            }
        else:
            self.config["target_mode"] = "behavior_tree"

        self._initial_scenario_metrics = {
            "legacy_scenario_index": float(scenario_index),
        }

    def _add_ref_old_shared_random_starting_position(self, randomization: dict) -> None:
        sign = np.array([-1.0, 1.0])
        rand_n = float(
            self.np_random.choice(sign)
            * self.np_random.uniform(
                0.0, float(randomization.get("shared_n_m", 4000.0))
            )
        )
        rand_e = float(
            self.np_random.choice(sign)
            * self.np_random.uniform(
                0.0, float(randomization.get("shared_e_m", 4000.0))
            )
        )
        rand_d = float(
            self.np_random.choice(sign)
            * self.np_random.uniform(
                0.0, float(randomization.get("shared_d_m", 4000.0))
            )
        )
        rand_distance_n = float(
            self.np_random.choice(sign)
            * self.np_random.uniform(
                0.0, float(randomization.get("target_distance_n_m", 300.0))
            )
        )
        rand_speed = float(
            self.np_random.choice(sign)
            * self.np_random.uniform(
                0.0, float(randomization.get("speed_mps", 50.0))
            )
        )

        self._sim._init_pos_n += rand_n
        self._sim._init_pos_e += rand_e
        self._sim._init_pos_d += rand_d
        self._sim._init_speed += rand_speed
        self._target_sim._init_pos_n += rand_n + rand_distance_n
        self._target_sim._init_pos_e += rand_e
        self._target_sim._init_pos_d += rand_d
        self._target_sim._init_speed += rand_speed

        for fighter in (self._sim, self._target_sim):
            lla = pm.ned2geodetic(
                fighter._init_pos_n,
                fighter._init_pos_e,
                fighter._init_pos_d,
                fighter._origin_lat,
                fighter._origin_lon,
                fighter._origin_alt,
            )
            fighter._init_pos_lat = lla[0]
            fighter._init_pos_lon = lla[1]
            fighter._init_pos_alt = lla[2]

    def _update_initial_geometry_metrics(self, scenario_mode: str) -> None:
        if scenario_mode not in ("scenario_pool", "two_circle_headon", "ref_old_random"):
            return
        ata = abs(float(self._geo_info._get_antenna_train_angle(
            self._ownship_state, self._target_state, True
        )))
        aa = abs(float(self._geo_info._get_aspect_angle(
            self._ownship_state, self._target_state, True
        )))
        distance = float(self._geo_info._get_distance(
            self._ownship_state, self._target_state
        ))
        self._initial_scenario_metrics.update({
            "initial_ata_deg": ata,
            "initial_aa_deg": aa,
            "initial_distance_m": distance,
        })

    @staticmethod
    def _range_values(values) -> tuple[float, float]:
        if isinstance(values, (int, float)):
            value = float(values)
            return value, value
        if len(values) != 2:
            raise ValueError(f"Expected two range values, got {values!r}")
        return float(values[0]), float(values[1])

    @staticmethod
    def _wrap_heading(value: float) -> float:
        return float(value % 360.0)

    def add_random_init_position(
        self,
        flight="ownship",
        radius=500.0,
        r_roll=5,
        r_pitch=5,
        r_heading=5,
    ):
        fighter = self._sim if flight == "ownship" else self._target_sim
        fighter._init_pos_n += self._sample_symmetric_offset(radius)
        fighter._init_pos_e += self._sample_symmetric_offset(radius)
        fighter._init_pos_d += self._sample_symmetric_offset(radius)
        fighter._init_roll += self._sample_symmetric_offset(r_roll)
        fighter._init_pitch += self._sample_symmetric_offset(r_pitch)
        fighter._init_heading += self._sample_symmetric_offset(r_heading)

        if fighter._init_roll > 180:
            fighter._init_roll -= 360
        if fighter._init_roll < -180:
            fighter._init_roll += 360
        if fighter._init_pitch > 180:
            fighter._init_pitch -= 360
        if fighter._init_pitch < -180:
            fighter._init_pitch += 360
        if fighter._init_heading > 360:
            fighter._init_heading -= 360
        if fighter._init_heading < 0:
            fighter._init_heading += 360

        lla = pm.ned2geodetic(
            fighter._init_pos_n,
            fighter._init_pos_e,
            fighter._init_pos_d,
            fighter._origin_lat,
            fighter._origin_lon,
            fighter._origin_alt,
        )
        fighter._init_pos_lat = lla[0]
        fighter._init_pos_lon = lla[1]
        fighter._init_pos_alt = lla[2]

    def _sample_symmetric_offset(self, magnitude: float) -> float:
        """Sample a signed uniform offset, treating non-positive ranges as zero."""
        magnitude = float(magnitude)
        if magnitude <= 0.0:
            return 0.0
        return float(self.np_random.uniform(-magnitude, magnitude))

    def _append_logs(self):
        self.ownship_log.append(
            [
                self._ownship_state[StateIndex.LAT],
                self._ownship_state[StateIndex.LON],
                self._ownship_state[StateIndex.ALT],
                self._ownship_state[StateIndex.ROLL],
                self._ownship_state[StateIndex.PITCH],
                self._ownship_state[StateIndex.YAW],
                self._ownship_state[StateIndex.HEALTH],
            ]
        )
        self.target_log.append(
            [
                self._target_state[StateIndex.LAT],
                self._target_state[StateIndex.LON],
                self._target_state[StateIndex.ALT],
                self._target_state[StateIndex.ROLL],
                self._target_state[StateIndex.PITCH],
                self._target_state[StateIndex.YAW],
                self._target_state[StateIndex.HEALTH],
            ]
        )

    def _build_action_context(self, sim, opponent_sim, ownship_state, target_state, observation) -> ActionContext:
        if ownship_state is not None and target_state is not None:
            observation = self._build_observation_for(
                np.array(ownship_state, copy=True),
                np.array(target_state, copy=True),
            )
        return ActionContext(
            sim=sim,
            opponent_sim=opponent_sim,
            ownship_state=np.array(ownship_state, copy=True) if ownship_state is not None else None,
            target_state=np.array(target_state, copy=True) if target_state is not None else None,
            observation=np.array(observation, copy=True) if observation is not None else None,
            info={"timestep": self.current_timestep},
        )

    def _reset_action_providers(self) -> None:
        for provider, sim, opponent, ownship_state, target_state in (
            (self._ownship_action_provider, self._sim, self._target_sim, self._ownship_state, self._target_state),
            (self._target_action_provider, self._target_sim, self._sim, self._target_state, self._ownship_state),
        ):
            if provider is None:
                continue
            provider.reset(self._build_action_context(sim, opponent, ownship_state, target_state, self.pre_obs))

    def get_ownship_sim(self):
        return self._sim

    def get_target_sim(self):
        return self._target_sim

    def get_ownship_action(self):
        return np.array(self._sim.action, dtype=np.float32)

    def get_target_action(self):
        return np.array(self._target_sim.action, dtype=np.float32)

    def get_ownship_VP(self):
        return np.array(self._sim.VP, dtype=np.float32)

    def get_target_VP(self):
        return np.array(self._target_sim.VP, dtype=np.float32)

    def get_ownship_state(self) -> np.ndarray:
        return self._ownship_state

    def get_target_state(self) -> np.ndarray:
        return self._target_state

    def get_damage(self):
        return self.ownship_damage, self.target_damage

    def get_ownship_state_for_udp(self) -> List:
        return [
            self._ownship_state[StateIndex.N],
            self._ownship_state[StateIndex.E],
            -self._ownship_state[StateIndex.D],
            self._ownship_state[StateIndex.ROLL],
            self._ownship_state[StateIndex.PITCH],
            self._ownship_state[StateIndex.YAW],
            self._ownship_state[StateIndex.HEALTH],
        ]

    def get_target_state_for_udp(self) -> List:
        return [
            self._target_state[StateIndex.N],
            self._target_state[StateIndex.E],
            -self._target_state[StateIndex.D],
            self._target_state[StateIndex.ROLL],
            self._target_state[StateIndex.PITCH],
            self._target_state[StateIndex.YAW],
            self._target_state[StateIndex.HEALTH],
        ]

    def make_tacviewLog(self):
        os.makedirs(self._artifacts_dir, exist_ok=True)
        timestamp = datetime.datetime.today()
        ownship_filename = (
            f"{timestamp.year}_{timestamp.month}_{timestamp.day}_{timestamp.hour}_{timestamp.minute}_{timestamp.second}_ownship_(F-16)[Blue].csv"
        )
        target_filename = (
            f"{timestamp.year}_{timestamp.month}_{timestamp.day}_{timestamp.hour}_{timestamp.minute}_{timestamp.second}_target_(F-16)[Red].csv"
        )
        summary_filename = (
            f"{timestamp.year}_{timestamp.month}_{timestamp.day}_{timestamp.hour}_{timestamp.minute}_{timestamp.second}_summary.json"
        )
        self._write_log(os.path.join(self._artifacts_dir, ownship_filename), self.ownship_log)
        self._write_log(os.path.join(self._artifacts_dir, target_filename), self.target_log)
        self._write_log_summary(os.path.join(self._artifacts_dir, summary_filename))

    def _write_log(self, path: str, entries: List[List[float]]) -> None:
        with open(path, "w", encoding="utf-8") as file:
            file.write(
                "Time,Longitude,Latitude,Altitude,Roll (deg),Pitch (deg),"
                "Yaw (deg),Health\n"
            )
            for step, item in enumerate(entries):
                time_value = np.floor(self._delta_t * step * 10000) / 10000
                health = item[6] if len(item) > 6 else ""
                file.write(
                    f"{time_value},{item[1]},{item[0]},{item[2]},"
                    f"{item[3]},{item[4]},{item[5]},{health}\n"
                )

    def _write_log_summary(self, path: str) -> None:
        summary = {
            "end_condition": self.info.get("end_condition", ""),
            "outcome": self.info.get("outcome", ""),
            "ownship_health": self.info.get("ownship_health", None),
            "target_health": self.info.get("target_health", None),
        }
        with open(path, "w", encoding="utf-8") as file:
            json.dump(summary, file, indent=2, ensure_ascii=False)

    def render(self):
        return None

    def close(self):
        if getattr(self, "_closed", True):
            return
        self._closed = True
        providers = (
            getattr(self, "_ownship_action_provider", None),
            getattr(self, "_target_action_provider", None),
        )
        for provider in providers:
            if provider is not None:
                try:
                    provider.close()
                except Exception:
                    pass
        sim_ai_pairs = (
            (getattr(self, "_sim", None), getattr(self, "_ownship_ai", None)),
            (getattr(self, "_target_sim", None), getattr(self, "_target_ai", None)),
        )
        for sim, ai in sim_ai_pairs:
            if (
                ai is not None
                and sim is not None
                and getattr(sim, "_model", None) is not None
            ):
                try:
                    ai.RemoveBT(sim.get_model().fighterID)
                except Exception:
                    pass
        battle_space_id = getattr(self, "battle_space_id", None)
        if battle_space_id is not None:
            try:
                JSBSimWrapper.RemoveSpace(battle_space_id)
            except Exception:
                pass

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass


__all__ = ["DogFightEnv", "normalize", "FEET_TO_METER", "METER_TO_FEET"]
