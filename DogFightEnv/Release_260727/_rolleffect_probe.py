"""One-off: correlate Controller_CY's Roll_Effect (pitch-authority gate) with the
az/el aiming-error decomposition, to test the hypothesis that pitch authority is
suppressed during the exact windows where vertical error is large.

C++ fprintf(stdout,...) writes to the OS-level fd, not Python's sys.stdout object,
so this just prints a tagged [PYAZEL] line after each step (both flush immediately)
and relies on natural interleaving in the combined captured stdout — parse
afterward by pairing each [PYAZEL] with the [CTRL_DBG] line(s) immediately
preceding it (last one = state closest to that step's outcome).
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

SEEDS = list(range(8))

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
        ownship_action_provider=BTActionProvider(dll_name="AIP_DCS_dbgtrace.dll"),
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
            print(f"[PYAZEL] az={abs(az):.3f} el={abs(el):.3f}", flush=True)
    env.close()
