"""One-off: v32 loss traces show their_ata wobbling in and out of the 45deg counter-
attack threshold constantly, yet my_ata never converges down from 130-180deg over a
200s episode even though the wobble-check should be firing whenever their_ata>=45.
Hypothesis: the wobble check flips FAILURE/SUCCESS (and therefore the tree flips
between DBFM evasion and Lead counter-attack) too rapidly tick-to-tick for either
maneuver to actually complete a turn -- chatter, not commitment. This probe logs every
single tick's their_ata and whether the wobble condition (>=45) is true, for the first
90s of a known-loss seed (seed=0 vs v32), to measure how often it flips state and
whether my_ata trends down at all during sustained "counter-attack triggered" spans.
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

SEED = 0
MAX_STEPS = 5400  # 90s at 60Hz

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
env.reset(seed=SEED)
terminated = truncated = False
step = 0
flips = 0
prev_state = None
state_durations = []  # (state, tick_count) runs
cur_run_len = 0

while not (terminated or truncated) and step < MAX_STEPS:
    _, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
    step += 1
    their_ata = abs(float(env._geo_info._get_antenna_train_angle(env._target_state, env._ownship_state, True)))
    my_ata = abs(float(env._geo_info._get_antenna_train_angle(env._ownship_state, env._target_state, True)))
    state = their_ata >= 45.0  # True = wobble-check would fire (counter-attack)

    if prev_state is not None and state != prev_state:
        flips += 1
        state_durations.append((prev_state, cur_run_len))
        cur_run_len = 0
    cur_run_len += 1
    prev_state = state

    if step % 60 == 0:  # once per second
        print(f"t={step/60:5.1f}s their_ata={their_ata:6.1f} my_ata={my_ata:6.1f} "
              f"wobble_active={state}", flush=True)

state_durations.append((prev_state, cur_run_len))
print(f"\n[SUMMARY] total_flips={flips} over {step} ticks ({step/60:.1f}s)", flush=True)
run_lens_true = [d for s, d in state_durations if s is True]
run_lens_false = [d for s, d in state_durations if s is False]
if run_lens_true:
    print(f"  counter-attack-active runs: n={len(run_lens_true)} "
          f"mean={np.mean(run_lens_true)/60:.2f}s max={max(run_lens_true)/60:.2f}s", flush=True)
if run_lens_false:
    print(f"  evasion-active runs: n={len(run_lens_false)} "
          f"mean={np.mean(run_lens_false)/60:.2f}s max={max(run_lens_false)/60:.2f}s", flush=True)
env.close()
