"""One-off: fight current best (AIP_DCS.dll) vs v29 for 5 seeds, calling make_tacviewLog()
after each episode so replay CSVs land in artifacts/logs (run_batch_dogfight.py doesn't
call this by default).
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

SEEDS = list(range(5))

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
        target_action_provider=BTActionProvider(dll_name="AIP_v29.dll"),
    )
    env.reset(seed=seed)
    terminated = truncated = False
    while not (terminated or truncated):
        _, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
    print(f"[SEED{seed}] outcome={info.get('outcome')} end_condition={info.get('end_condition')} "
          f"own_hp={info.get('ownship_health'):.3f} tgt_hp={info.get('target_health'):.3f}", flush=True)
    env.make_tacviewLog()
    env.close()

print("[DONE] replay logs written to artifacts/logs/", flush=True)
