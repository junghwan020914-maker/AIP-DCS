"""One-off: for known loss seeds (1,9,12,15,16,19 from _loss_diag_probe.py), sample
final_ata_deg (our aim error to them) and final_aa_deg (aspect/position) every 1s during
the bleed-out to see whether we're geometrically in a defensive posture (aa near 0 = we're
in front getting shot, or aa near 180 = we're behind but still eating damage some other way)
or something else entirely. Goal: figure out why the bleed-out isn't broken by DBFM evasion.
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

SEEDS = [1, 9, 12, 16, 19]

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
    hist = []
    while not (terminated or truncated):
        _, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
        step += 1
        dist = env._geo_info._get_distance(env._ownship_state, env._target_state)
        their_ata = abs(float(env._geo_info._get_antenna_train_angle(env._target_state, env._ownship_state, True)))
        hist.append((info.get("ownship_health"), their_ata, info.get("final_aa_deg"), dist))

    tail = hist[-1200:][::60]
    print(f"[BLEEDOUT] seed={seed} outcome={info.get('outcome')} steps={step}", flush=True)
    for i, (hp, their_ata, aa, d) in enumerate(tail):
        print(f"   t-{20-i}s  own_hp={hp:.3f} their_ata_on_me={their_ata:.1f} my_aa={aa:.1f} dist={d:.0f}", flush=True)
    env.close()
