"""One-off: after adding DECO_StuckCheck, v29 results got WORSE (draws 7->2, losses
26->30) while v22c/v0/v1 improved. Trace [STUCK_FIRE]/[WOBBLE_FIRE] debug events
(printed by the DLL itself, team=1=us) per seed against v29 to see how often/when the
new forced counter-attack fires, and correlate with outcome to find the mechanism.
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

NUM_SEEDS = 50

for seed in range(NUM_SEEDS):
    print(f"[SEED_START] seed={seed}", flush=True)
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
        target_action_provider=BTActionProvider(dll_name="AIP_v29.dll"),
    )
    env.reset(seed=seed)
    terminated = truncated = False
    while not (terminated or truncated):
        _, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
    print(f"[SEED_END] seed={seed} outcome={info.get('outcome')} own={info.get('ownship_health'):.3f} "
          f"tgt={info.get('target_health'):.3f}", flush=True)
    env.close()

print("\n[DONE]", flush=True)
