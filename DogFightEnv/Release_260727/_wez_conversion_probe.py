"""One-off: for the highest-WEZ seed (27, 280 steps) from the fair-scenario final50 test,
trace target_health over time to see whether WEZ damage accumulates toward a kill or
gets interrupted/reset before finishing the job.
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

SEEDS = [27, 9, 49]

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
    hist = []
    while not (terminated or truncated):
        _, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
        hist.append((info.get("target_health"), info.get("ownship_health"), info.get("target_damage", 0) > 0))

    n = len(hist)
    print(f"[CONV] seed={seed} steps={n} final_tgt_hp={hist[-1][0]:.4f} final_own_hp={hist[-1][1]:.4f}", flush=True)
    sample_every = max(1, n // 40)
    for i in range(0, n, sample_every):
        th, oh, wez = hist[i]
        t = i / 60.0
        print(f"   t={t:6.1f}s tgt_hp={th:.4f} own_hp={oh:.4f} wez={'*' if wez else ' '}", flush=True)
    env.close()
