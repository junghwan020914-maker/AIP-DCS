"""One-off: for a batch of seeds vs ryujan, classify each seed's final outcome/end_condition,
and for LOSS seeds specifically capture the health trajectory in the last ~20s to see whether
death is a fast/decisive kill or a slow bleed-out. Goal: find a diagnosable death pattern
(similar in spirit to how bugs 1-3 were found via direct observation, not blind tuning).
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

SEEDS = list(range(20))

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
    own_hp_hist = []
    while not (terminated or truncated):
        _, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
        step += 1
        own_hp_hist.append(info.get("ownship_health"))

    outcome = info.get("outcome")
    end_cond = info.get("end_condition")
    print(f"[LOSSDIAG] seed={seed} outcome={outcome} end_condition={end_cond} steps={step}", flush=True)

    if outcome == "loss":
        # print health every 1s (60 ticks) for the last 20s
        tail = own_hp_hist[-1200:]
        sampled = tail[::60]
        print(f"  own_hp last~20s (1s steps): {[round(h,3) if h is not None else None for h in sampled]}", flush=True)
    env.close()
