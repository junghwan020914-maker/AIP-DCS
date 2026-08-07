"""v5(SmoothPursuit, 회피형)는 유일하게 승률이 낮은 상대(16/14/0). WEZ는 25/30시드에서
발생하는데도 승률이 안 나온다 — "근접은 하는데 지는" 역전 패턴. 기존 진단은 전부
2D 각도(proj=True) 버그 영향권이라 무효였으므로 3D(proj=False, 실제 데미지 계산과 동일)로
재진단한다.

핵심 질문: 우리가 상대를 조준하는 시간 vs 상대가 우리를 조준하는 시간, 어느 쪽이 긴가?
- 우리가 더 길면 → 조준은 이기는데 데미지 전환이 안 되는 문제
- 상대가 더 길면 → 애초에 각도 싸움에서 지고 있는 것
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
    my_lock = their_lock = 0      # 3도 이내 + 사거리내 틱수
    my_near = their_near = 0      # 10도 이내 + 사거리내 틱수
    while not (terminated or truncated):
        _, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
        dist = env._geo_info._get_distance(env._ownship_state, env._target_state)
        if not (152.0 <= dist <= 914.0):
            continue
        my_ata = abs(float(env._geo_info._get_antenna_train_angle(
            env._ownship_state, env._target_state, False)))
        their_ata = abs(float(env._geo_info._get_antenna_train_angle(
            env._target_state, env._ownship_state, False)))
        if my_ata < 3.0:
            my_lock += 1
        if my_ata < 10.0:
            my_near += 1
        if their_ata < 3.0:
            their_lock += 1
        if their_ata < 10.0:
            their_near += 1

    oh = float(info.get("ownship_health", 1.0))
    th = float(info.get("target_health", 1.0))
    rows.append((seed, info.get("outcome"), oh, th, my_lock, their_lock, my_near, their_near))
    print(f"[V5] seed={seed:>2} {str(info.get('outcome')):<5} own={oh:.3f} tgt={th:.3f} "
          f"| 내조준<3도={my_lock/60:5.1f}s 상대조준<3도={their_lock/60:5.1f}s "
          f"| 내<10도={my_near/60:5.1f}s 상대<10도={their_near/60:5.1f}s", flush=True)
    env.close()

wins = [r for r in rows if r[1] == "win"]
losses = [r for r in rows if r[1] == "loss"]

def agg(rs, label):
    if not rs:
        print(f"  {label}: 없음", flush=True)
        return
    ml = np.mean([r[4] for r in rs]) / 60
    tl = np.mean([r[5] for r in rs]) / 60
    mn = np.mean([r[6] for r in rs]) / 60
    tn = np.mean([r[7] for r in rs]) / 60
    print(f"  {label}(n={len(rs)}): 내조준<3도 평균={ml:.2f}s 상대조준<3도 평균={tl:.2f}s "
          f"| 내<10도={mn:.2f}s 상대<10도={tn:.2f}s", flush=True)

print("\n[요약] 사거리(152~914m) 안에서의 조준 우위 비교", flush=True)
agg(wins, "승리")
agg(losses, "패배")
agg(rows, "전체")
