"""Check whether RelatchSeconds=1.5 (vs default 3.0) helps the specific v22c seeds
that previously showed ZERO aim convergence (seed 9, 12, 16 -- min_my_ata stayed
above 24deg or barely dipped to 2.36deg once, for the entire 200s episode).
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

for seed in [9, 12, 16]:
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
        target_action_provider=BTActionProvider(dll_name="AIP_DCS_ryujan.dll"),
    )
    env.reset(seed=seed)
    terminated = truncated = False
    my_atas = []
    while not (terminated or truncated):
        _, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
        my_ata = abs(float(env._geo_info._get_antenna_train_angle(env._ownship_state, env._target_state, True)))
        my_atas.append(my_ata)
    arr = np.array(my_atas)
    dt = 1.0 / 60.0
    t_under_10 = np.sum(arr < 10.0) * dt
    print(f"[RELATCH_CHECK] seed={seed} outcome={info.get('outcome')} "
          f"final_tgt={info.get('target_health'):.3f} min_my_ata={arr.min():.2f} "
          f"t<10deg={t_under_10:.1f}s", flush=True)
    env.close()
