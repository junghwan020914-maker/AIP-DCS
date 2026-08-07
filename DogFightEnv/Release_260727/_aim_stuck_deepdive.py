"""Follow-up to _aim_precision_probe.py: seeds 9 and 16 (loss, target_health=1.0)
NEVER get my_ata under 10deg the entire 200s episode, vs win seeds that spend
130-184s under 10deg. Trace distance + my_ata + BFM phase together to see whether
these episodes are stuck in DBFM(=2) the whole time (AimBlend should be nudging
that down via BreakTurn, distance>=600m) or in Jinking territory (<600m, no blend
active since that experiment was reverted) or something else (HABFM/OBFM
misclassification).
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

BFM_NAMES = {0: "OBFM", 1: "HABFM", 2: "DBFM", 3: "DETECTING", 4: "SCISSORS", 5: "NONE"}

for seed in [9, 16, 12]:
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
    step = 0
    hist = []
    bfm_counts = {}
    dist_lt600_ticks = 0
    while not (terminated or truncated):
        _, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
        step += 1
        dist = env._geo_info._get_distance(env._ownship_state, env._target_state)
        my_ata = abs(float(env._geo_info._get_antenna_train_angle(env._ownship_state, env._target_state, True)))
        their_ata = abs(float(env._geo_info._get_antenna_train_angle(env._target_state, env._ownship_state, True)))
        hist.append((dist, my_ata, their_ata))
        if dist < 600.0:
            dist_lt600_ticks += 1

    n = len(hist)
    sample_every = max(1, n // 30)
    print(f"\n[TRACE] seed={seed} steps={n} outcome={info.get('outcome')} "
          f"final_tgt={info.get('target_health'):.3f} ticks_dist<600={dist_lt600_ticks}/{n} "
          f"({100.0*dist_lt600_ticks/n:.0f}%)", flush=True)
    for i in range(0, n, sample_every):
        d, ma, ta = hist[i]
        t = i / 60.0
        print(f"   t={t:6.1f}s dist={d:7.0f}m my_ata={ma:6.1f} their_ata={ta:6.1f} "
              f"{'[JINK-ZONE]' if d < 600 else ''}", flush=True)
    env.close()

print("\n[DONE]", flush=True)
