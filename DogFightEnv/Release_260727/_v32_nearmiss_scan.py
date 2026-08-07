"""One-off: earlier diagnosis found one v32 loss (seed11, pre-AimBlend build) where we
drove target_health down to 0.037 (nearly a kill) but then disengaged and got reversed
into a loss 70s later. Only one example was seen. This scans more seeds with the
CURRENT best build (BreakTurn AimBlend=0.2) to quantify how often we get a target very
low (min_tgt_hp < 0.3) but still end up losing -- i.e. how common the "didn't finish
the kill" pattern actually is now.
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
nearmiss_losses = []

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
        target_action_provider=BTActionProvider(dll_name="AIP_v32.dll"),
    )
    env.reset(seed=seed)
    terminated = truncated = False
    min_tgt_hp = 1.0
    while not (terminated or truncated):
        _, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
        th = float(info.get("target_health", 1.0))
        if th < min_tgt_hp:
            min_tgt_hp = th
    outcome = info.get("outcome")
    tag = ""
    if outcome == "loss" and min_tgt_hp < 0.3:
        tag = "  <-- NEAR-MISS (had them low, still lost)"
        nearmiss_losses.append(seed)
    print(f"[SCAN] seed={seed} outcome={outcome} own={info.get('ownship_health'):.3f} "
          f"tgt_final={info.get('target_health'):.3f} tgt_min={min_tgt_hp:.3f}{tag}", flush=True)
    env.close()

print(f"\n[SUMMARY] near-miss losses (min_tgt_hp<0.3 but outcome=loss): "
      f"{len(nearmiss_losses)}/{NUM_SEEDS} seeds={nearmiss_losses}", flush=True)
