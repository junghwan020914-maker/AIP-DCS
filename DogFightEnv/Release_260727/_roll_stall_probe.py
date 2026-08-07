"""One-off: capture per-tick [CTRL_DBG] UT (roll-alignment angle) trace to see whether
UT approaches 180 (full invert) monotonically-but-slowly, or plateaus/reverses around
70-90 deg (knife-edge) -- distinguishes "asymptotically slow" from "genuinely stuck".
Run with stdout redirected to a file (C++ fprintf goes to the real OS fd).
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

SEEDS = list(range(6))

for seed in SEEDS:
    print(f"[SEEDMARK] seed={seed}", flush=True)
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
    step = 0
    while not (terminated or truncated):
        _, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
        step += 1
        if step > 3000:
            break
    env.close()
