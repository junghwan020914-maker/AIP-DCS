"""상대 기수 각속도가 **우리 능력(18.4도/s)을 얼마나 초과하는가** — 추종 지연의 물리 한계 확인.

08-14 배경. `_aimerr_probe.py`로 v5와 arcD의 조준 오차를 같은 조건에서 분해했더니
**병목의 성격이 정반대**였다:

    지표                        v5        arcD
    ATA 중앙                 27.41도     2.79도
    고도 el 중앙            -26.57도    -2.07도
    부호반전율(고도)          0.00%      0.54%
    **corr(상대각속도, ATA)  +0.472**   +0.082
    회귀 el~AOA            -0.54, 절편 **-16.2**   -1.02, 절편 +0.21

arcD는 **고정 편향**(받음각으로 온전히 설명, 상대 각속도 무관)인데
v5는 **추종 지연**이다 — 상대가 빠르게 돌수록 우리가 뒤처지고(상관 6배),
받음각으로 설명 안 되는 -16.2도가 남는다. 부호반전율 0.00%는 **한 번도 따라잡아
넘어선 적이 없다**는 뜻으로 지연의 극단이다.

우리 최대 선회율은 실측 **18.4도/s**다(`_aoa_probe.py`: KTAS 260~280에서 최대,
기하 계측과 이론식 g*sqrt(Nz^2-1)/V가 1~2도/s 이내로 일치).
상대가 그보다 빠르게 기수를 돌리면 **원리상 못 따라간다.**

⚠️ 판정 기준을 미리 박는다:
  · 초과가 흔하면(예: 시간의 30% 이상) **물리적 한계**다. arcV에서 이미 같은 결론을
    낸 적이 있다(요구 35.5도/s vs 가능 16.6도/s). 그러면 추격이 아니라 다른 전략이 필요하다.
  · 초과가 드문데도 뒤처지면 **제어 대역폭** 문제이고 고칠 여지가 있다.
  이 둘은 처방이 완전히 다르므로 재고 나서 결정한다.

⚠️ 기수 각속도는 **위치 양자화**(1e-6도 = 약 0.11m)에 취약하다. 자세(오일러각)에서
   직접 계산하고 12틱(0.2초) 창으로 평활한다 — `_aoa_probe.py`에서 1틱 차분이
   105도/s의 가짜 신호를 만든 전례가 있다.

## ❌ 08-15 측정 — **물리적 한계 가설 기각. 그리고 관계가 거꾸로다.**

12시드(오프셋 30), 공세+밴드 구간만:

    상대            표본     중앙   p75   p90   최대 |  초과%  2배초과% | 내ATA중앙
    AIP_v5.dll      4540    10.2  12.4  14.3  27.1 |   1.1%    0.0% |   24.67
    AIP_arcD.dll   19282    13.2  16.5  21.3  35.5 |  18.3%    0.0% |    2.60
    AIP_v7.dll     16440    13.0  16.1  20.5  31.1 |  15.6%    0.0% |    4.22

v5 초과율 **1.1%**, p90 14.3도/s — 우리 능력 18.4도/s에 한참 못 미친다.
사전 등록 기준(30%)으로 **물리적 한계가 아니다.** v5는 우리를 따돌릴 만큼 빨리 돌지 않는다.

**그런데 세 상대를 나란히 놓으면 관계가 반대다:**

    가장 느리게 도는 v5(10.2)에게 조준이 가장 나쁘고(24.67도, 20시드 8.0점)
    가장 빠르게 도는 arcD(13.2)에게 조준이 가장 좋다(2.60도, 20.0점)
    v7(13.0)도 20.0점.  **각속도로는 설명이 안 된다.**

➡️ `_aimerr_probe.py`의 corr(상대각속도, ATA)=+0.472는 **인과가 아니라 증상**이었다.
   v5가 빨리 도는 순간은 곧 우리가 뒤에 붙어 상대가 브레이크하는 순간이고,
   그때 ATA가 커지는 건 당연하다. 상관을 처방으로 바꾸면 0/6인 그 패턴이다.
   압축 전에 박아둔 경고("초과율이 낮아도 왜 27도에서 시작하는지를 다시 볼 것")가 작동했다.

⚠️ **표본 수가 더 큰 이야기를 한다.** 판당 공세+밴드 시간(=득점 기회):

    v5 6.3초   |   v7 22.8초   |   arcD 26.8초

   v5 상대로는 **기회 자체가 1/4**이고, 그 좁은 기회 안에서 조준도 10배 나쁘다.
   병목은 조준이 아니라 **밴드에 들어가는 것** — `_factor_probe.py`가 v5를
   "위치(공세를 못 만든다)"로 재분류한 것과 같은 결론에 독립적으로 도달했다.

   이건 v5 고유가 아니다. 저득점 상대 넷(prev 7.0 / v5 8.0 / arcV 10.0 / v42 10.5)이
   모두 같은 모양이다 — arcV는 수직으로(요구 35.5도/s), v42는 기하로(교차마다 그들이
   우리 뒤, 필요 개선폭 133도), v5는 밴드 시간 1/4로. **셋 다 우리가 뒤를 못 잡는다.**
   조준 정밀도 처방이 계속 기각된 이유가 이것이다.

사용:
    python _omega_probe.py --targets AIP_v5.dll,AIP_arcD.dll --num-seeds 12 --seed-offset 30
Not part of the student template; safe to delete after use.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
SRC = ROOT / "src"
for p in (ROOT, SRC):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from _aim_time_probe import DT, make_state  # noqa: E402
from DogFightEnvWrapper import DogFightWrapper  # noqa: E402
from dogfight.ai.bt_action_provider import BTActionProvider  # noqa: E402

BAND_MIN, BAND_MAX = 152.4, 914.4
FRONT, REAR = 50.0, 130.0
OUR_MAX = 18.4          # 실측 최대 선회율(deg/s)
HIST = 12               # 0.2초 창


def fwd_vec(roll, pitch, yaw):
    p, y = np.radians([pitch, yaw])
    return np.array([np.cos(p) * np.cos(y), np.cos(p) * np.sin(y), np.sin(p)])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ownship", default="AIP_DCS.dll")
    ap.add_argument("--targets", required=True)
    ap.add_argument("--num-seeds", type=int, default=12)
    ap.add_argument("--seed-offset", type=int, default=0)
    args = ap.parse_args()

    print("=" * 100)
    print(f"[상대 각속도 대 우리 능력 {OUR_MAX:.1f}도/s]  {args.num_seeds}시드"
          f"(오프셋 {args.seed_offset})   공세+밴드 구간만")
    print(f"  {'상대':<18} {'표본':>7} {'중앙':>7} {'p75':>7} {'p90':>7} {'최대':>7} | "
          f"{'초과%':>7} {'2배초과%':>9} | {'내ATA중앙':>10}")
    for tname in [x.strip() for x in args.targets.split(",") if x.strip()]:
        if not (ROOT / tname).exists():
            print(f"  (없음) {tname}")
            continue
        OM, MA = [], []
        for seed in range(args.seed_offset, args.seed_offset + args.num_seeds):
            rng = np.random.default_rng(seed)
            own, tgt = make_state(rng)
            env = DogFightWrapper(
                env_config={
                    "observation_mode": "tactical16", "ownship_control_mode": "rl",
                    "target_mode": "rl", "max_engage_time": 200.0, "min_altitude": 300.0,
                    "ownship": own, "target": tgt,
                    "initial_scenario": {"mode": "default"},
                },
                ownship_action_provider=BTActionProvider(dll_name=args.ownship),
                target_action_provider=BTActionProvider(dll_name=tname),
            )
            env.reset(seed=seed)
            buf = []
            terminated = truncated = False
            while not (terminated or truncated):
                _, _, terminated, truncated, _ = env.step(np.zeros(4, dtype=np.float32))
                g, o, t = env._geo_info, env._ownship_state, env._target_state
                buf.append(fwd_vec(float(t[3]), float(t[4]), float(t[5])))
                if len(buf) > HIST + 1:
                    buf.pop(0)
                if len(buf) <= HIST:
                    continue
                d = float(g._get_distance(o, t))
                ma = abs(float(g._get_antenna_train_angle(o, t, False)))
                ta = abs(float(g._get_antenna_train_angle(t, o, False)))
                if not (BAND_MIN <= d <= BAND_MAX and ma < FRONT and ta > REAR):
                    continue
                c = float(np.clip(np.dot(buf[0], buf[-1]), -1.0, 1.0))
                OM.append(np.degrees(np.arccos(c)) / (HIST * DT))
                MA.append(ma)
            env.close()
        if not OM:
            print(f"  {tname:<18} 공세+밴드 표본 없음")
            continue
        om = np.array(OM); ma = np.array(MA)
        print(f"  {tname:<18} {len(om):>7} {np.median(om):>7.1f} "
              f"{np.percentile(om,75):>7.1f} {np.percentile(om,90):>7.1f} {om.max():>7.1f} | "
              f"{100*np.mean(om > OUR_MAX):>6.1f}% {100*np.mean(om > 2*OUR_MAX):>8.1f}% | "
              f"{np.median(ma):>10.2f}")
    print("\n  ➜ 초과%가 30% 이상이면 **물리적 한계** — 추격이 아닌 다른 전략이 필요하다")
    print("     드문데도 뒤처지면 **제어 대역폭** 문제 — 고칠 여지가 있다")
    print("=" * 100)


if __name__ == "__main__":
    main()
