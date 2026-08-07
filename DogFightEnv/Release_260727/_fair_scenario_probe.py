"""One-off: re-test current best build vs ryujan v22c using ONLY scenario indices 2,3,4
(excludes 0,1,5,6,7 which all give ownship a starting tail-chase advantage per 08-04
analysis) to see true skill level without the scenario-pool bias. 200s per rules v1.4.
Not part of the student template; safe to delete after use.
"""
from __future__ import annotations

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

SEEDS = list(range(50))
wins = losses = draws = 0
wez_total = 0
wez_seeds = 0

for seed in SEEDS:
    env = DogFightWrapper(
        env_config={
            "observation_mode": "tactical16",
            "ownship_control_mode": "rl",
            "target_mode": "rl",
            "max_engage_time": 200.0,
            "min_altitude": 300.0,
            "initial_scenario": {
                "mode": "ref_old_random",
                "legacy_scenario_indices": [2, 3, 4],
            },
        },
        ownship_action_provider=BTActionProvider(dll_name="AIP_DCS.dll"),
        target_action_provider=BTActionProvider(dll_name="AIP_DCS_ryujan.dll"),
    )
    env.reset(seed=seed)
    terminated = truncated = False
    seed_wez = 0
    while not (terminated or truncated):
        _, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
        if info.get("target_damage", 0) > 0:
            seed_wez += 1
    outcome = info.get("outcome")
    if outcome == "win":
        wins += 1
    elif outcome == "loss":
        losses += 1
    else:
        draws += 1
    if seed_wez > 0:
        wez_seeds += 1
    wez_total += seed_wez
    print(f"[FAIR] seed={seed} scenario_index={info.get('legacy_scenario_index')} "
          f"outcome={outcome} wez_steps={seed_wez}", flush=True)
    env.close()

print(f"\n[SUMMARY] seeds={len(SEEDS)} win={wins} loss={losses} draw={draws} "
      f"wez_total={wez_total} wez_seeds={wez_seeds}", flush=True)
