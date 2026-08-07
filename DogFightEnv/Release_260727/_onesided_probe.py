"""One-off: check whether the Lead.cpp closure-throttle fix's reduced local-loss count
is masking "one-sided evasion" per the real competition rule (15s+ of unanswered
damage counts as a loss even if the local env's classify() calls it a draw -- see
project_topgun_rules memory). Tracks, per seed, the longest continuous streak where
ownship is taking damage while dealing none back (proxy for "being one-sidedly beaten").
60Hz sim -> 900 ticks = 15s.
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

SEEDS = list(range(30))
TICKS_15S = 900

results = []

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

    prev_own_hp = None
    prev_tgt_hp = None
    cur_streak = 0
    max_streak = 0

    while not (terminated or truncated):
        _, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
        own_hp = info.get("ownship_health")
        tgt_hp = info.get("target_health")
        if prev_own_hp is not None:
            took_dmg = own_hp < prev_own_hp - 1e-9
            dealt_dmg = tgt_hp < prev_tgt_hp - 1e-9
            if took_dmg and not dealt_dmg:
                cur_streak += 1
                max_streak = max(max_streak, cur_streak)
            elif dealt_dmg:
                cur_streak = 0
            # neither taking nor dealing damage: streak doesn't reset (still "not fighting back"),
            # but also isn't actively "being beaten" -- only count active one-sided damage ticks
        prev_own_hp, prev_tgt_hp = own_hp, tgt_hp

    final_own = prev_own_hp
    final_tgt = prev_tgt_hp
    flagged = max_streak >= TICKS_15S
    results.append((seed, max_streak, max_streak / 60.0, final_own, final_tgt, flagged))
    print(f"[ONESIDED] seed={seed} max_streak_ticks={max_streak} ({max_streak/60.0:.1f}s) "
          f"final_own_hp={final_own} final_tgt_hp={final_tgt} FLAG_15s={flagged}", flush=True)
    env.close()

flagged_count = sum(1 for r in results if r[5])
print(f"\n[SUMMARY] seeds={len(results)} flagged(>=15s one-sided)={flagged_count}", flush=True)
