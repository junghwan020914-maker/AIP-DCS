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
