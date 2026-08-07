"""Refined az/el decomposition: only steps where we are already roughly nose-on
(final_ata_deg < 30) and within WEZ-ish range — isolates the "closing the shot"
phase rather than merges/evasion where facing away is expected/correct.
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

SEEDS = list(range(15))

az_samples = []
el_samples = []

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
    while not (terminated or truncated):
        _, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
        dist = env._geo_info._get_distance(env._ownship_state, env._target_state)
        ata = info["final_ata_deg"]
        if dist < 1500.0 and ata < 30.0:
            az, el = env._geo_info._get_los_angle(env._ownship_state, env._target_state)
            az_samples.append(abs(az))
            el_samples.append(abs(el))
    env.close()

az = np.array(az_samples)
el = np.array(el_samples)
print(f"\nn={len(az)} steps with dist<1500m AND ata<30deg (closing-the-shot phase)")
if len(az) > 0:
    print(f"mean |az| (horizontal error) = {az.mean():.2f} deg")
    print(f"mean |el| (vertical error)   = {el.mean():.2f} deg")
    print(f"median |az| = {np.median(az):.2f}  median |el| = {np.median(el):.2f}")
    print(f"fraction where |el| > |az|: {(el > az).mean():.2%}")
