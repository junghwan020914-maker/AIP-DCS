"""One-off root-cause dig: most remaining losses end with target_health exactly 1.000
(we never land a single hit), even against v22c where we win 84% of the time. This
probe checks whether the bottleneck is (a) we rarely even get roughly pointed at the
target (my_ata never gets small), or (b) we get roughly pointed (e.g. <5-10deg) but
can't tighten further into the actual WEZ cone (1deg per WEZUpdate ConeDeg=1.0 in
Rule_mine.xml). For a mix of win and no-damage-loss seeds against v22c, tracks the full
tick-by-tick my_ata (our aim error) and reports: minimum ever achieved, time spent
under 10/5/3/1 degrees, and how many separate times we cross under 1deg (to see if we
touch the cone briefly but can't hold it, vs never reach it at all).
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

NUM_SCAN_SEEDS = 40
buckets = {"win": [], "loss_nodmg": [], "other": []}

for seed in range(NUM_SCAN_SEEDS):
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
        target_action_provider=BTActionProvider(dll_name="AIP_DCS_ryujan.dll"),
    )
    env.reset(seed=seed)
    terminated = truncated = False
    while not (terminated or truncated):
        _, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
    outcome = info.get("outcome")
    tgt = float(info.get("target_health", 1.0))
    if outcome == "win":
        buckets["win"].append(seed)
    elif outcome == "loss" and tgt >= 0.999:
        buckets["loss_nodmg"].append(seed)
    else:
        buckets["other"].append(seed)
    env.close()

print(f"[BUCKETS] win={buckets['win']}\nloss_nodmg={buckets['loss_nodmg']}\nother={buckets['other']}", flush=True)

TRACE_SEEDS = [("win", s) for s in buckets["win"][:4]] + [("loss_nodmg", s) for s in buckets["loss_nodmg"][:4]]

for bucket_name, seed in TRACE_SEEDS:
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
        target_action_provider=BTActionProvider(dll_name="AIP_DCS_ryujan.dll"),
    )
    env.reset(seed=seed)
    terminated = truncated = False
    my_atas = []
    while not (terminated or truncated):
        _, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
        my_ata = abs(float(env._geo_info._get_antenna_train_angle(env._ownship_state, env._target_state, False)))
        my_atas.append(my_ata)

    arr = np.array(my_atas)
    dt = 1.0 / 60.0
    t_under_10 = np.sum(arr < 10.0) * dt
    t_under_5 = np.sum(arr < 5.0) * dt
    t_under_3 = np.sum(arr < 3.0) * dt
    t_under_1 = np.sum(arr < 1.0) * dt
    # count separate crossings below 1deg (rising-edge count)
    below1 = arr < 1.0
    crossings = np.sum((~below1[:-1]) & below1[1:]) if len(below1) > 1 else 0

    print(f"[AIM_STATS] seed={seed} bucket={bucket_name} steps={len(arr)} "
          f"min_my_ata={arr.min():.2f} t<10deg={t_under_10:.1f}s t<5deg={t_under_5:.1f}s "
          f"t<3deg={t_under_3:.1f}s t<1deg={t_under_1:.2f}s crossings<1deg={crossings}",
          flush=True)
    env.close()

print("\n[DONE]", flush=True)
