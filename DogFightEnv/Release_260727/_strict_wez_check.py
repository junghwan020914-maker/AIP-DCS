"""Precise check for seed19 and seed31 (no-damage losses that showed substantial
my_ata<10deg time while simultaneously in the 152-914m range): does the STRICT WEZ
condition (dist in [152,914] AND my_ata<=1.0deg, matching WEZUpdate.cpp exactly)
ever actually occur? If yes but target_health never drops, something beyond
TargetInMyWEZ gates damage (e.g. IsAimmingMode dwell time, or a mismatch between our
Python geometry approximation and the DLL's internal Los_Degree). If no, confirms the
10deg proxy was too loose and the real bottleneck is still "never reach the 1deg cone
while in range simultaneously."
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

for seed in [19, 31]:
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
    strict_wez_ticks = 0
    min_my_ata_inrange = 999.0
    events = []
    prev_th = 1.0
    while not (terminated or truncated):
        _, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
        step += 1
        dist = env._geo_info._get_distance(env._ownship_state, env._target_state)
        my_ata = abs(float(env._geo_info._get_antenna_train_angle(env._ownship_state, env._target_state, False)))
        inrange = 152.0 <= dist <= 914.0
        if inrange and my_ata < min_my_ata_inrange:
            min_my_ata_inrange = my_ata
        if inrange and my_ata <= 1.0:
            strict_wez_ticks += 1
            if len(events) < 10:
                events.append((step / 60.0, dist, my_ata, info.get("target_damage", None),
                                info.get("target_health")))
        th = float(info.get("target_health", 1.0))
        if th < prev_th - 1e-6:
            print(f"[DAMAGE_EVENT] seed={seed} t={step/60.0:.2f}s tgt_health {prev_th:.4f}->{th:.4f} "
                  f"dist={dist:.0f} my_ata={my_ata:.2f}", flush=True)
        prev_th = th

    print(f"[STRICT] seed={seed} outcome={info.get('outcome')} final_tgt={info.get('target_health'):.3f} "
          f"strict_wez_ticks(dist_inrange&my_ata<=1)={strict_wez_ticks} "
          f"min_my_ata_while_inrange={min_my_ata_inrange:.3f}", flush=True)
    for ev in events:
        print(f"    sample: t={ev[0]:.2f}s dist={ev[1]:.0f} my_ata={ev[2]:.3f} "
              f"target_damage_field={ev[3]} tgt_health={ev[4]:.4f}", flush=True)
    env.close()
