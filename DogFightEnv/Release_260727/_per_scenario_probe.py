"""One-off: break down win rate by individual scenario index (0-7) to understand why
the "favorable" pool (0,1,5,6,7) showed a LOWER overall win rate (34%) than the
"unfavorable/neutral" pool (2,3,4, 66%) -- counterintuitive given 0,1,5,6,7 were
identified as giving ownship a starting position advantage. Uses current production
build (with fixed classify() logic and DBFM/HABFM counter-attack fix) vs ryujan v22c.
Not part of the student template; safe to delete after use.
"""
from __future__ import annotations

import sys
from pathlib import Path
from collections import defaultdict

import numpy as np

ROOT = Path(__file__).resolve().parent
SRC = ROOT / "src"
for p in (ROOT, SRC):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from DogFightEnvWrapper import DogFightWrapper
from dogfight.ai.bt_action_provider import BTActionProvider

SEEDS_PER_SCENARIO = 8
results = defaultdict(lambda: {"win": 0, "loss": 0, "draw": 0, "wez_seeds": 0})

for scenario_idx in range(8):
    for seed in range(SEEDS_PER_SCENARIO):
        env = DogFightWrapper(
            env_config={
                "observation_mode": "tactical16",
                "ownship_control_mode": "rl",
                "target_mode": "rl",
                "max_engage_time": 200.0,
                "min_altitude": 300.0,
                "initial_scenario": {
                    "mode": "ref_old_random",
                    "legacy_scenario_indices": [scenario_idx],
                },
            },
            ownship_action_provider=BTActionProvider(dll_name="AIP_DCS.dll"),
            target_action_provider=BTActionProvider(dll_name="AIP_DCS_ryujan.dll"),
        )
        env.reset(seed=seed)
        terminated = truncated = False
        seed_wez = False
        while not (terminated or truncated):
            _, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
            if info.get("target_damage", 0) > 0:
                seed_wez = True
        outcome = info.get("outcome")
        results[scenario_idx][outcome if outcome in ("win", "loss", "draw") else "draw"] += 1
        if seed_wez:
            results[scenario_idx]["wez_seeds"] += 1
        print(f"[PERSCEN] scenario={scenario_idx} seed={seed} outcome={outcome} "
              f"final_own={info.get('ownship_health'):.3f} final_tgt={info.get('target_health'):.3f}",
              flush=True)
        env.close()

print("\n[BREAKDOWN]", flush=True)
for idx in range(8):
    r = results[idx]
    total = r["win"] + r["loss"] + r["draw"]
    print(f"  scenario={idx}: win={r['win']} loss={r['loss']} draw={r['draw']} "
          f"wez_seeds={r['wez_seeds']}/{total}", flush=True)
