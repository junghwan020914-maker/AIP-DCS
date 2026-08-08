"""공식 초기조건에서의 병목 진단.

로컬조건 트레이스로 세운 가설(최상위 Lead가 사거리 내 76.5% 점유)이 공식조건에서는
성립하지 않음이 확인됐다(사거리 게이트 무효). 공식조건에서 직접 측정한다.

측정 항목(전부 3D, proj=False — 실제 데미지 계산과 동일):
  - 사거리대별 체류시간
  - 내 조준 vs 상대 조준 (3도/10도 이내 시간)
  - 상대LOS 분포(최상위 반격전환 발동조건 45도를 얼마나 넘는지)
  - 승/패 그룹별 비교
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

FEET = 0.3048
SEP_FT = (2000.0, 2500.0, 3000.0)
TARGET_DLL = sys.argv[1] if len(sys.argv) > 1 else "AIP_v32.dll"
NUM = int(sys.argv[2]) if len(sys.argv) > 2 else 20


def make_state(rng):
    sep_m = float(rng.choice(SEP_FT)) * FEET
    alt_m = float(rng.uniform(2000.0, 30000.0)) * FEET
    spd = float(rng.uniform(200.0, 300.0))
    hdg = float(rng.uniform(0.0, 360.0))
    # 08-08 정정: 2000/2500/3000ft는 **기체 간 거리**이고, 두 기체는 서로의 3-9 라인
    # (날개 축 = 기수 기준 좌우)에 나란히 놓여 **반대 방향**을 본다. 기수 대 기수(헤드온)가
    # 아니다. 이전 구현은 헤드온이라 시작하자마자 서로 WEZ 안에 있었고, 그래서 초반부터
    # 득점이 났다 — 실제로는 둘 다 90도 이상 선회해야 교전이 시작된다.
    # 분리 방향 = 내 기수의 수직(hdg+90). 상대 기수 = hdg+180.
    half = sep_m / 2.0
    rad = np.deg2rad(hdg)
    prad = np.deg2rad(hdg + 90.0)
    pn, pe = np.cos(prad) * half, np.sin(prad) * half
    return ([-pn, -pe, -alt_m, 0.0, 0.0, hdg, spd],
            [pn, pe, -alt_m, 0.0, 0.0, (hdg + 180.0) % 360.0, spd])


rows = []
for seed in range(NUM):
    rng = np.random.default_rng(seed)
    own, tgt = make_state(rng)
    env = DogFightWrapper(
        env_config={
            "observation_mode": "tactical16", "ownship_control_mode": "rl", "target_mode": "rl",
            "max_engage_time": 200.0, "min_altitude": 300.0,
            "ownship": own, "target": tgt,
            "initial_scenario": {"mode": "default"},
        },
        ownship_action_provider=BTActionProvider(dll_name="AIP_DCS.dll"),
        target_action_provider=BTActionProvider(dll_name=TARGET_DLL),
    )
    env.reset(seed=seed)
    terminated = truncated = False
    my3 = th3 = my10 = th10 = 0
    inband = tooclose = toofar = 0
    los_tgt_over45 = 0
    while not (terminated or truncated):
        _, _, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
        d = env._geo_info._get_distance(env._ownship_state, env._target_state)
        my = abs(float(env._geo_info._get_antenna_train_angle(
            env._ownship_state, env._target_state, False)))
        their = abs(float(env._geo_info._get_antenna_train_angle(
            env._target_state, env._ownship_state, False)))
        if their >= 45.0:
            los_tgt_over45 += 1
        if d < 152.0:
            tooclose += 1
        elif d <= 914.0:
            inband += 1
            if my < 3.0: my3 += 1
            if my < 10.0: my10 += 1
            if their < 3.0: th3 += 1
            if their < 10.0: th10 += 1
        else:
            toofar += 1
    total = inband + tooclose + toofar
    rows.append(dict(seed=seed, out=info.get("outcome"),
                     oh=float(info.get("ownship_health", 1.0)),
                     th=float(info.get("target_health", 1.0)),
                     inband=inband, tooclose=tooclose, toofar=toofar, total=total,
                     my3=my3, th3=th3, my10=my10, th10=th10, over45=los_tgt_over45))
    r = rows[-1]
    print(f"[OFF] seed={seed:>2} {str(r['out']):<5} oh={r['oh']:.3f} th={r['th']:.3f} "
          f"| 사거리내={inband/60:5.1f}s <152m={tooclose/60:4.1f}s >914m={toofar/60:5.1f}s "
          f"| 내<3도={my3/60:5.2f}s 상대<3도={th3/60:5.2f}s "
          f"| 상대LOS>45 비율={100*los_tgt_over45/max(total,1):4.1f}%", flush=True)
    env.close()


def agg(rs, label):
    if not rs: return
    f = lambda k: np.mean([r[k] for r in rs]) / 60
    pct = np.mean([100 * r["over45"] / max(r["total"], 1) for r in rs])
    print(f"  {label}(n={len(rs)}): 사거리내={f('inband'):5.1f}s  "
          f"내<3도={f('my3'):5.2f}s 상대<3도={f('th3'):5.2f}s  "
          f"내<10도={f('my10'):5.1f}s 상대<10도={f('th10'):5.1f}s  "
          f"상대LOS>45={pct:4.1f}%", flush=True)

print(f"\n[요약] 공식조건 vs {TARGET_DLL}", flush=True)
agg([r for r in rows if r["out"] == "win"], "승리")
agg([r for r in rows if r["out"] == "loss"], "패배")
agg(rows, "전체")
