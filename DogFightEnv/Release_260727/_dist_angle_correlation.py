"""One-off: WEZUpdate.cpp requires BOTH distance in [152,914]m AND my_ata<=1deg
SIMULTANEOUSLY to register TargetInMyWEZ. Hypothesis: our aim convergence moments
(my_ata briefly near 0) often happen at the WRONG distance (too far, since BreakTurn's
VP offset is 4500m out -- we might be pointed well but still far outside 914m), so
even good aim doesn't score. For seeds that get my_ata very low but still show zero
damage, log distance at each tick where my_ata<10deg to see whether those low-angle
moments coincide with in-range distance or not.
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

NUM_SEEDS = 40
low_angle_far_count = 0
low_angle_inrange_count = 0
per_seed_rows = []

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
        target_action_provider=BTActionProvider(dll_name="AIP_DCS_ryujan.dll"),
    )
    env.reset(seed=seed)
    terminated = truncated = False
    ticks_low_angle = 0
    ticks_low_angle_inrange = 0
    while not (terminated or truncated):
        _, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
        dist = env._geo_info._get_distance(env._ownship_state, env._target_state)
        my_ata = abs(float(env._geo_info._get_antenna_train_angle(env._ownship_state, env._target_state, True)))
        if my_ata < 10.0:
            ticks_low_angle += 1
            if 152.0 <= dist <= 914.0:
                ticks_low_angle_inrange += 1
    outcome = info.get("outcome")
    tgt = float(info.get("target_health", 1.0))
    frac_inrange = (ticks_low_angle_inrange / ticks_low_angle) if ticks_low_angle > 0 else float("nan")
    print(f"[CORR] seed={seed} outcome={outcome} tgt_final={tgt:.3f} "
          f"ticks_my_ata<10={ticks_low_angle} of_those_inrange152_914={ticks_low_angle_inrange} "
          f"frac_inrange={frac_inrange:.2f}", flush=True)
    per_seed_rows.append((seed, outcome, tgt, ticks_low_angle, ticks_low_angle_inrange))
    env.close()

# Aggregate: for no-damage losses specifically, what fraction of low-angle ticks were in-range?
nodmg_loss_rows = [r for r in per_seed_rows if r[1] == "loss" and r[2] >= 0.999]
win_rows = [r for r in per_seed_rows if r[1] == "win"]

def agg(rows):
    total_low = sum(r[3] for r in rows)
    total_inrange = sum(r[4] for r in rows)
    return total_low, total_inrange, (total_inrange / total_low if total_low else float("nan"))

nl, ni, nf = agg(nodmg_loss_rows)
wl, wi, wf = agg(win_rows)
print(f"\n[AGG] no-damage-loss seeds: total_low_angle_ticks={nl} in_range_frac={nf:.3f}", flush=True)
print(f"[AGG] win seeds:            total_low_angle_ticks={wl} in_range_frac={wf:.3f}", flush=True)
