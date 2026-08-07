"""One-off: for the near-miss seeds found by _noengage_probe.py (2 and 7, which got to
255.8m and 169.8m without ever scoring), find the exact tick of minimum distance and
report the aim angle (az/el via env._geo_info._get_los_angle) at that moment -- tells us
whether it's a "almost aligned, just needs a bit more precision" case or a "fast
pass-by with no real tracking" case.
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

SEEDS = [2, 7]

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
    best = (1e9, None, None, None)  # dist, step, az, el
    window = []  # (step, dist, az, el) for the 20 ticks around min

    history = []
    while not (terminated or truncated):
        _, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
        step += 1
        dist = env._geo_info._get_distance(env._ownship_state, env._target_state)
        az, el = env._geo_info._get_los_angle(env._ownship_state, env._target_state)
        history.append((step, dist, az, el))
        if dist < best[0]:
            best = (dist, step, az, el)
    env.close()

    min_step = best[1]
    lo = max(0, min_step - 10)
    hi = min(len(history), min_step + 10)
    print(f"[CLOSEPASS] seed={seed} min_dist={best[0]:.1f} at step={min_step} "
          f"az={best[2]:.2f} el={best[3]:.2f} combined_ang={np.hypot(best[2],best[3]):.2f}", flush=True)
    print(f"  window (step,dist,az,el):", flush=True)
    for h in history[lo:hi]:
        print(f"   {h[0]} dist={h[1]:.1f} az={h[2]:.2f} el={h[3]:.2f}", flush=True)
