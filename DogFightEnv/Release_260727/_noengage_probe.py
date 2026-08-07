"""One-off: for seeds flagged as "zero engagement the whole episode" by _onesided_probe.py
(seeds 0,2,3,6,7 among others), sample distance over time to see whether the two
aircraft ever get close (merge attempted but chase fails) or never get close at all
(scenario/geometry issue, or both sides just orbiting far apart).
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

SEEDS = [0, 2, 3, 6, 7]

for seed in SEEDS:
    env = DogFightWrapper(
        env_config={
            "observation_mode": "tactical16",
            "ownship_control_mode": "rl",
            "target_mode": "rl",
            "max_engage_time": 300.0,
            "min_altitude": 300.0,
            "initial_scenario": {"mode": "ref_old_random"},
        },
        ownship_action_provider=BTActionProvider(dll_name="AIP_DCS.dll"),
        target_action_provider=BTActionProvider(dll_name="AIP_DCS_ryujan.dll"),
    )
    env.reset(seed=seed)
    terminated = truncated = False
    step = 0
    min_dist_seen = 1e9
    dists = []
    while not (terminated or truncated):
        _, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
        step += 1
        dist = env._geo_info._get_distance(env._ownship_state, env._target_state)
        min_dist_seen = min(min_dist_seen, dist)
        if step % 300 == 0:  # every ~5s at 60Hz
            dists.append((step / 60.0, round(dist, 1)))
    print(f"[NOENGAGE] seed={seed} steps={step} min_dist_ever={min_dist_seen:.1f} "
          f"trajectory={dists}", flush=True)
    env.close()
