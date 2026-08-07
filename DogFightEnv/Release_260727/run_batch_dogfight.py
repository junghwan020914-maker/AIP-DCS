"""Multi-seed batch dogfight evaluation.

Runs N engagements with randomized initial conditions (one per seed) and
aggregates the results, so tuning decisions rest on statistics instead of a
single deterministic scenario.

The BT DLLs read their rule XML from a hardcoded filename next to the DLL
(this project's DLL reads Rule_mine.xml, the base/Trinity DLLs read
Rule_forTraining.xml). Place those files before running; this tool does not
manage them.

Usage:
    python run_batch_dogfight.py --ownship-bt-dll AIP_DCS.dll \
        --target-bt-dll AIP_DCS_trinity.dll --num-seeds 20
"""
from __future__ import annotations

import argparse
import statistics
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
SRC = ROOT / "src"
for p in (ROOT, SRC):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from DogFightEnvWrapper import DogFightWrapper
from dogfight.ai.bt_action_provider import BTActionProvider


def parse_args():
    ap = argparse.ArgumentParser(description="Batch (multi-seed) dogfight evaluation.")
    ap.add_argument("--ownship-bt-dll", default="AIP_DCS.dll")
    ap.add_argument("--target-bt-dll", default="AIP_BASE_target.dll")
    ap.add_argument("--num-seeds", type=int, default=20)
    ap.add_argument("--seed-start", type=int, default=0)
    ap.add_argument("--scenario-mode", default="ref_old_random",
                    choices=["ref_old_random", "two_circle_headon", "default"],
                    help="default = fixed spawn (seeds have no effect)")
    ap.add_argument("--max-engage-time", type=float, default=300.0)
    ap.add_argument("--min-altitude", type=float, default=300.0)
    ap.add_argument("--observation-mode", default="tactical16")
    ap.add_argument("--quiet", action="store_true", help="summary only")
    return ap.parse_args()


def classify(info):
    oh = float(info.get("ownship_health", 1.0))
    th = float(info.get("target_health", 1.0))
    if th <= 0.0 and oh > 0.0:
        return "WIN"
    if oh <= 0.0 and th > 0.0:
        return "LOSS"
    if oh <= 0.0 and th <= 0.0:
        return "BOTH"
    # 08-04: 규정집 v1.4 제6조2항 — 격추 없이 시간만료면 잔여 HP비율 높은 쪽 승리.
    # 기존엔 이 비교 없이 전부 DRAW로 뭉뚱그려서, 실제로는 승리/패배인 경기가 전부
    # 무승부로 잘못 집계되고 있었음.
    if oh > th:
        return "WIN"
    if th > oh:
        return "LOSS"
    return "DRAW"


def main():
    args = parse_args()

    env = DogFightWrapper(
        env_config={
            "observation_mode": args.observation_mode,
            "ownship_control_mode": "rl",
            "target_mode": "rl",
            "max_engage_time": args.max_engage_time,
            "min_altitude": args.min_altitude,
            # deep-merged: only the mode is overridden, other scenario keys survive
            "initial_scenario": {"mode": args.scenario_mode},
        },
        ownship_action_provider=BTActionProvider(dll_name=args.ownship_bt_dll),
        target_action_provider=BTActionProvider(dll_name=args.target_bt_dll),
    )

    rows = []
    try:
        for seed in range(args.seed_start, args.seed_start + args.num_seeds):
            env.reset(seed=seed)
            total_reward = 0.0
            terminated = truncated = False
            info = {}
            while not (terminated or truncated):
                _, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
                total_reward += reward

            row = {
                "seed": seed,
                "reward": total_reward,
                "result": classify(info),
                "end": str(info.get("end_condition", ""))[:28],
                "wez": int(info.get("ep_wez_steps", 0)),
                "steps": int(info.get("ep_step_count", 0)),
                "min_d": float(info.get("ep_min_distance", 0.0)),
                "ata": float(info.get("final_ata_deg", 0.0)),
                "oh": float(info.get("ownship_health", 1.0)),
                "th": float(info.get("target_health", 1.0)),
            }
            rows.append(row)
            if not args.quiet:
                print(f"seed {row['seed']:>3} | {row['result']:<5} | reward {row['reward']:>9.2f} "
                      f"| WEZ {row['wez']:>4} | minD {row['min_d']:>7.0f}m | oh {row['oh']:.2f} th {row['th']:.2f} "
                      f"| {row['end']}")
    finally:
        env.close()

    if not rows:
        print("no runs")
        return

    rewards = [r["reward"] for r in rows]
    wez = [r["wez"] for r in rows]
    wins = sum(1 for r in rows if r["result"] == "WIN")
    losses = sum(1 for r in rows if r["result"] == "LOSS")
    draws = len(rows) - wins - losses

    print("=" * 78)
    print(f"seeds={len(rows)}  scenario={args.scenario_mode}  "
          f"ownship={args.ownship_bt_dll}  target={args.target_bt_dll}")
    print(f"  mean reward = {statistics.fmean(rewards):.2f}"
          + (f"  (stdev {statistics.stdev(rewards):.2f})" if len(rewards) > 1 else ""))
    print(f"  win/loss/draw = {wins}/{losses}/{draws}")
    print(f"  WEZ steps: total={sum(wez)}  mean={statistics.fmean(wez):.1f}  "
          f"seeds with any WEZ={sum(1 for w in wez if w > 0)}")
    print(f"  mean min distance = {statistics.fmean([r['min_d'] for r in rows]):.0f} m")
    print("=" * 78)


if __name__ == "__main__":
    main()
