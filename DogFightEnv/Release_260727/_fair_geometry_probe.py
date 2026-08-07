"""One-off: for scenario_index=4 (target starts with tail advantage over ownship -- the
most common draw in the "fair" 30-seed test that showed WEZ=0 across the board), trace
distance and our own LOS (aim error) over the episode to see whether we ever even close
the range, or get close but can't align, or never get close at all.
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

SEEDS = [0, 2, 3]  # all scenario_index=4 per the fair probe log

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
                "legacy_scenario_indices": [4],
            },
        },
        ownship_action_provider=BTActionProvider(dll_name="AIP_DCS.dll"),
        target_action_provider=BTActionProvider(dll_name="AIP_DCS_ryujan.dll"),
    )
    env.reset(seed=seed)
    terminated = truncated = False
    step = 0
    hist = []
    while not (terminated or truncated):
        _, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
        step += 1
        dist = env._geo_info._get_distance(env._ownship_state, env._target_state)
        my_ata = abs(float(env._geo_info._get_antenna_train_angle(env._ownship_state, env._target_state, True)))
        their_ata = abs(float(env._geo_info._get_antenna_train_angle(env._target_state, env._ownship_state, True)))
        hist.append((dist, my_ata, their_ata))

    n = len(hist)
    sample_every = max(1, n // 40)
    print(f"[GEOM] seed={seed} steps={n} outcome={info.get('outcome')} min_dist={min(h[0] for h in hist):.0f}", flush=True)
    for i in range(0, n, sample_every):
        d, ma, ta = hist[i]
        t = i / 60.0
        print(f"   t={t:6.1f}s dist={d:7.0f}m my_ata={ma:6.1f} their_ata={ta:6.1f}", flush=True)
    env.close()
