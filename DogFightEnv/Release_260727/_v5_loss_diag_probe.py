"""One-off: v5(SmoothPursuit, evasive) is now a losing matchup (14/16/0 over 30 seeds)
despite WEZ forming in 25/30 seeds and mean_min_distance=63m (very close engagements) --
the opposite pattern from v29/v22c (where the problem was never closing/aiming at all).
Hypothesis to test: are we forming WEZ but for too few steps to convert (their WEZ steps
against us outpace ours), or are we aiming fine but taking damage from a different
mechanism (e.g. their Phase2/3-range fire while we're still closing)? Scan for loss
seeds, then trace distance/my_ata/their_ata/health timelines for a few.
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

NUM_SEEDS = 30
loss_seeds = []

for seed in range(NUM_SEEDS):
    env = DogFightWrapper(
        env_config={
            "observation_mode": "tactical16",
            "ownship_control_mode": "rl",
            "target_mode": "rl",
            "max_engage_time": 200.0,
            "min_altitude": 300.0,
            "initial_scenario": {"mode": "ref_old_random"},
        },
        ownship_action_provider=BTActionProvider(dll_name="AIP_DCS.dll"),
        target_action_provider=BTActionProvider(dll_name="AIP_v5.dll"),
    )
    env.reset(seed=seed)
    terminated = truncated = False
    while not (terminated or truncated):
        _, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
    outcome = info.get("outcome")
    print(f"[SCAN] seed={seed} outcome={outcome} own={info.get('ownship_health'):.3f} "
          f"tgt={info.get('target_health'):.3f}", flush=True)
    if outcome == "loss":
        loss_seeds.append(seed)
    env.close()

print(f"\n[LOSS SEEDS] {loss_seeds}", flush=True)

TRACE_SEEDS = loss_seeds[:4]
for seed in TRACE_SEEDS:
    env = DogFightWrapper(
        env_config={
            "observation_mode": "tactical16",
            "ownship_control_mode": "rl",
            "target_mode": "rl",
            "max_engage_time": 200.0,
            "min_altitude": 300.0,
            "initial_scenario": {"mode": "ref_old_random"},
        },
        ownship_action_provider=BTActionProvider(dll_name="AIP_DCS.dll"),
        target_action_provider=BTActionProvider(dll_name="AIP_v5.dll"),
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
        hist.append((dist, my_ata, their_ata, info.get("ownship_health"), info.get("target_health")))

    n = len(hist)
    sample_every = max(1, n // 30)
    print(f"\n[TRACE] seed={seed} steps={n} outcome={info.get('outcome')} "
          f"min_dist={min(h[0] for h in hist):.0f} final_own={hist[-1][3]:.3f} final_tgt={hist[-1][4]:.3f}",
          flush=True)
    for i in range(0, n, sample_every):
        d, ma, ta, oh, th = hist[i]
        t = i / 60.0
        print(f"   t={t:6.1f}s dist={d:7.0f}m my_ata={ma:6.1f} their_ata={ta:6.1f} own_hp={oh:.3f} tgt_hp={th:.3f}",
              flush=True)
    env.close()

print("\n[DONE]", flush=True)
