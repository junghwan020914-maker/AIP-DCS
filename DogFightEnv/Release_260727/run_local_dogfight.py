from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parent   # Release/ 루트
SRC = ROOT / "src"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from DogFightEnvWrapper import DogFightWrapper
from dogfight.ai.bt_action_provider import BTActionProvider
from dogfight.ai.bt_rule_manager import activate_rule_xml
from dogfight.ai.curriculum import build_stage_env_config
from dogfight.ai.hybrid_action_provider import HybridActionProvider
from dogfight.ai.rllib_utils import build_algorithm_from_bundle
from dogfight.ai.rl_action_provider import RLActionProvider
from dogfight.ai.student_hooks import (
    load_curriculum_stages,
    load_observation_hook,
)
from dogfight.ai.training.config_io import deep_update, load_experiment_env_config
from dogfight.config import merge_env_config


def parse_args():
    parser = argparse.ArgumentParser(description="Run local dogfight simulation between two inference backends.")
    parser.add_argument("--ownship-backend", choices=["rl", "bt", "hybrid", "fixed"], required=True)
    parser.add_argument("--target-backend", choices=["rl", "bt", "hybrid", "fixed"], required=True)
    parser.add_argument("--ownship-bundle-dir")
    parser.add_argument("--target-bundle-dir")
    parser.add_argument("--ownship-bt-dll", default="AIP_DCS_ownship.dll")
    parser.add_argument("--target-bt-dll", default="AIP_BASE_target.dll")
    parser.add_argument("--bt-rule-xml", help="Optional Rule.xml source to activate while the simulation runs.")
    parser.add_argument("--ownship-policy-id", default="default_policy")
    parser.add_argument("--target-policy-id", default="default_policy")
    parser.add_argument("--observation-mode", default="tactical16", choices=["classic12", "relative14", "tactical16", "custom"])
    parser.add_argument("--observation-module", default="", help="Optional custom observation module.")
    parser.add_argument("--hybrid-mode", choices=["residual", "blend", "switch"], default="residual")
    parser.add_argument("--alpha", type=float, default=0.5)
    parser.add_argument("--residual-scale", type=float, default=0.35)
    parser.add_argument("--max-engage-time", type=float, default=300.0)
    parser.add_argument("--episode-step-limit", type=int, default=18000)
    parser.add_argument("--min-altitude", type=float, default=300.0)
    parser.add_argument(
        "--experiment-yaml",
        default="",
        help="Optional experiment YAML whose env_config supplies the scenario.",
    )
    parser.add_argument(
        "--stages-module",
        default="",
        help="Optional curriculum module used to reuse a stage scenario.",
    )
    parser.add_argument(
        "--stage-index", type=int, default=None, help="Stage index to reuse."
    )
    parser.add_argument("--save-log", action="store_true", help="Save tacview CSV log after the episode.")
    return parser.parse_args()


def build_provider(side: str, backend: str, bundle_dir: str | None, bt_dll: str, policy_id: str, hybrid_mode: str, alpha: float, residual_scale: float):
    if backend == "fixed":
        return None
    if backend == "bt":
        return BTActionProvider(dll_name=bt_dll)
    if backend == "rl":
        if not bundle_dir:
            raise ValueError(f"--{side}-bundle-dir is required when {side}-backend=rl")
        return RLActionProvider(bundle_dir=bundle_dir, algorithm_factory=build_algorithm_from_bundle, policy_id=policy_id)
    if backend == "hybrid":
        if not bundle_dir:
            raise ValueError(f"--{side}-bundle-dir is required when {side}-backend=hybrid")
        rl_provider = RLActionProvider(bundle_dir=bundle_dir, algorithm_factory=build_algorithm_from_bundle, policy_id=policy_id)
        bt_provider = BTActionProvider(dll_name=bt_dll)
        return HybridActionProvider(
            primary_provider=rl_provider,
            secondary_provider=bt_provider,
            mode=hybrid_mode,
            alpha=alpha,
            residual_scale=residual_scale,
        )
    raise ValueError(f"Unsupported backend: {backend}")


def backend_to_env_mode(backend: str) -> str:
    if backend == "fixed":
        return "fixed"
    return "rl"


def build_local_env_config(args, observation_mode: str) -> dict:
    """Build local evaluation config while keeping CLI backends authoritative."""
    env_config = merge_env_config({
        "observation_mode": observation_mode,
        "observation_module": args.observation_module,
        "max_engage_time": args.max_engage_time,
        "episode_step_limit": args.episode_step_limit,
        "min_altitude": args.min_altitude,
    })
    if args.experiment_yaml:
        deep_update(env_config, load_experiment_env_config(args.experiment_yaml, ROOT))
    if args.stages_module:
        if args.stage_index is None:
            raise ValueError("--stage-index is required with --stages-module")
        stages = load_curriculum_stages(args.stages_module)
        matches = [stage for stage in stages if stage.index == args.stage_index]
        if not matches:
            raise ValueError(
                f"stage index {args.stage_index} not found in {args.stages_module!r}"
            )
        env_config = build_stage_env_config(env_config, matches[0])
    elif args.stage_index is not None:
        raise ValueError("--stages-module is required with --stage-index")

    env_config["observation_mode"] = observation_mode
    env_config["observation_module"] = args.observation_module
    env_config["ownship_control_mode"] = backend_to_env_mode(args.ownship_backend)
    env_config["target_mode"] = backend_to_env_mode(args.target_backend)
    env_config["scenario_pool_apply_target_mode"] = False
    return env_config


def main():
    args = parse_args()
    observation_hook = load_observation_hook(args.observation_module) if args.observation_module else None

    ownship_provider = build_provider(
        side="ownship",
        backend=args.ownship_backend,
        bundle_dir=args.ownship_bundle_dir,
        bt_dll=args.ownship_bt_dll,
        policy_id=args.ownship_policy_id,
        hybrid_mode=args.hybrid_mode,
        alpha=args.alpha,
        residual_scale=args.residual_scale,
    )
    target_provider = build_provider(
        side="target",
        backend=args.target_backend,
        bundle_dir=args.target_bundle_dir,
        bt_dll=args.target_bt_dll,
        policy_id=args.target_policy_id,
        hybrid_mode=args.hybrid_mode,
        alpha=args.alpha,
        residual_scale=args.residual_scale,
    )

    observation_mode = observation_hook["mode"] if observation_hook else args.observation_mode
    env_config = build_local_env_config(args, observation_mode)
    with activate_rule_xml(args.bt_rule_xml, ROOT):
        env = DogFightWrapper(
            env_config=env_config,
            observation_fn=observation_hook["build_observation"] if observation_hook else None,
            observation_size=observation_hook["size"] if observation_hook else None,
            observation_low=observation_hook["low"] if observation_hook else None,
            observation_high=observation_hook["high"] if observation_hook else None,
            ownship_action_provider=ownship_provider,
            target_action_provider=target_provider,
        )

        try:
            observation, info = env.reset()
            terminated = False
            truncated = False
            total_reward = 0.0
            while not (terminated or truncated):
                observation, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
                total_reward += reward

            print("simulation finished")
            print(f"end_condition: {info.get('end_condition', '')}")
            print(f"terminated: {terminated} truncated: {truncated}")
            print(f"total_reward: {total_reward:.4f}")
            print(f"ownship_health: {info.get('ownship_health', 'n/a')}")
            print(f"target_health: {info.get('target_health', 'n/a')}")

            if args.save_log:
                env.make_tacviewLog()
                print("tacview log saved")
        finally:
            env.close()


if __name__ == "__main__":
    main()
