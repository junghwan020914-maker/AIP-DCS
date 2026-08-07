"""v5전 진단 후속: 3D 재진단에서 "10도까지는 가는데 3도로 못 조인다"가 확인됐고,
평균 최소거리가 52~63m로 WEZ 최소사거리(152m)를 크게 밑돈다. 득점 불가 구간(<152m)에
얼마나 머무는지, 그게 승패와 상관있는지 측정한다.

가설: 과도하게 파고들어 152m 밑(득점불가) + 각속도 폭증 구간에서 시간을 낭비하고,
그 사이 v5는 적정 거리에서 우리를 추적한다.
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

NUM_SEEDS = 30
rows = []

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
        target_action_provider=BTActionProvider(dll_name="AIP_v5.dll"),
    )
    env.reset(seed=seed)
    terminated = truncated = False
    t_too_close = t_band = t_far = 0
    # 사거리 안에 있으면서 조준이 안 되는 시간 (기회 낭비) 도 따로 집계
    band_but_bad_aim = 0
    while not (terminated or truncated):
        _, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
        d = env._geo_info._get_distance(env._ownship_state, env._target_state)
        if d < 152.0:
            t_too_close += 1
        elif d <= 914.0:
            t_band += 1
            my_ata = abs(float(env._geo_info._get_antenna_train_angle(
                env._ownship_state, env._target_state, False)))
            if my_ata >= 3.0:
                band_but_bad_aim += 1
        else:
            t_far += 1

    outcome = info.get("outcome")
    rows.append((seed, outcome, t_too_close, t_band, t_far, band_but_bad_aim))
    print(f"[BAND] seed={seed:>2} {str(outcome):<5} "
          f"<152m={t_too_close/60:6.1f}s  152~914m={t_band/60:6.1f}s  >914m={t_far/60:6.1f}s "
          f"| 사거리내 조준실패={band_but_bad_aim/60:6.1f}s", flush=True)
    env.close()

def agg(rs, label):
    if not rs:
        return
    tc = np.mean([r[2] for r in rs]) / 60
    tb = np.mean([r[3] for r in rs]) / 60
    tf = np.mean([r[4] for r in rs]) / 60
    bb = np.mean([r[5] for r in rs]) / 60
    print(f"  {label}(n={len(rs)}): <152m={tc:.1f}s  152~914m={tb:.1f}s  >914m={tf:.1f}s "
          f"| 사거리내 조준실패={bb:.1f}s", flush=True)

print("\n[요약] 거리대별 체류시간", flush=True)
agg([r for r in rows if r[1] == "win"], "승리")
agg([r for r in rows if r[1] == "loss"], "패배")
agg(rows, "전체")
