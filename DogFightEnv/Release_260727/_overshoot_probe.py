"""오버슛 빈도 측정 — "뒤를 잡았다가 지나쳐서 역전당하는" 판이 얼마나 되는가.

08-10 아웃오브샘플(시드 30~59) 패배 5판을 조사하다 나온 것. 4판은 알려진 유형
(공세 0%, 수세 고착)이었지만 arcA seed58은 전혀 달랐다:

    t=80   572m  내ATA  3.5  적ATA 159.5   <- 우리가 뒤를 잡음
    t=90   407m  내ATA  6.6  적ATA 175.1   <- 내득점 0.8303 (83% 격추)
    t=100  308m  내ATA 168.0 적ATA  54.9   <- 역전. 10초 만에 앞뒤가 바뀜
    t=110  440m  내ATA 176.1 적ATA   4.0   <- 상대가 조준
    종료   내 0.8303  상대 1.0046  -> 패배

**83% 격추까지 해놓고 과근접에서 지나쳐 역전당했다.**
오늘 거리 분석과 맞물린다 — 152~450m는 P(ATA<1도)가 사실상 0인 구간인데
거기로 스스로 파고든다. 트리의 오버슛 방지(`Lag`)는 `LOS > 40도`에서만 걸려서
**조준이 잘 될수록 Pure로 직진해 관통한다.**

시드 하나로 처방을 만들면 오늘 반복한 실수이므로 **빈도부터 잰다.**

오버슛 이벤트 정의: 공세(내ATA<50 & 적ATA>130) + 밴드 안이었다가,
  WindowSec 안에 수세(내ATA>130 & 적ATA<50)로 뒤집힌 경우.

사용:
    python _overshoot_probe.py --ownship AIP_DCS.dll --target AIP_arcA.dll \
        --num-seeds 30 --seed-offset 30
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

from _aim_time_probe import DT, make_state, score_rate  # noqa: E402
from DogFightEnvWrapper import DogFightWrapper  # noqa: E402
from dogfight.ai.bt_action_provider import BTActionProvider  # noqa: E402

BAND_MIN, BAND_MAX = 152.4, 914.4
FRONT, REAR = 50.0, 130.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ownship", default="AIP_DCS.dll")
    ap.add_argument("--target", required=True)
    ap.add_argument("--num-seeds", type=int, default=30)
    ap.add_argument("--seed-offset", type=int, default=0)
    ap.add_argument("--window", type=float, default=15.0, help="역전 인정 시간(초)")
    args = ap.parse_args()

    tot_ev = 0
    seeds_with_ev = 0
    ev_dmin = []
    ev_spd = []
    lost_after = 0
    rows = []
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
            target_action_provider=BTActionProvider(dll_name=args.target),
        )
        env.reset(seed=seed)
        terminated = truncated = False
        step = 0
        my_hp = th_hp = 0.0
        last_off = -1e9        # 마지막으로 공세+밴드였던 시각
        off_dmin = 1e9         # 그 공세 구간의 최소거리
        off_spd = 0.0
        events = 0
        while not (terminated or truncated):
            _, _, terminated, truncated, _ = env.step(np.zeros(4, dtype=np.float32))
            step += 1
            t_s = step * DT
            g, o, t = env._geo_info, env._ownship_state, env._target_state
            d = float(g._get_distance(o, t))
            ma = abs(float(g._get_antenna_train_angle(o, t, False)))
            ta = abs(float(g._get_antenna_train_angle(t, o, False)))
            my_hp += score_rate(d, ma, t_s)[0] * DT
            th_hp += score_rate(d, ta, t_s)[0] * DT
            inband = BAND_MIN <= d <= BAND_MAX
            if inband and ma < FRONT and ta > REAR:          # 공세
                last_off = t_s
                off_dmin = min(off_dmin, d)
                off_spd = float(o[6])
            elif ma > REAR and ta < FRONT:                    # 수세로 뒤집힘
                if t_s - last_off <= args.window:
                    events += 1
                    ev_dmin.append(off_dmin)
                    ev_spd.append(off_spd)
                    last_off = -1e9                            # 중복 계상 방지
                    off_dmin = 1e9
        env.close()
        lost = th_hp > my_hp
        tot_ev += events
        if events:
            seeds_with_ev += 1
            if lost:
                lost_after += 1
        rows.append((seed, events, my_hp, th_hp, lost))
        if events:
            print(f"  seed {seed:>2}  오버슛 {events}회  내{my_hp:.3f} 상{th_hp:.3f}"
                  f"{'  🔴패배' if lost else ''}", flush=True)

    n = args.num_seeds
    print("\n" + "=" * 92)
    print(f"[오버슛 빈도] {n}시드(오프셋 {args.seed_offset})  나={args.ownship}  상대={args.target}")
    print(f"  오버슛 총 {tot_ev}회 / 발생 시드 {seeds_with_ev}/{n}판 ({seeds_with_ev/n*100:.0f}%)")
    if ev_dmin:
        print(f"  오버슛 직전 최소거리 중앙 {np.median(ev_dmin):5.0f}m  "
              f"내속도 중앙 {np.median(ev_spd):5.1f}m/s")
        print(f"  오버슛 난 시드 중 패배 {lost_after}/{seeds_with_ev}판")
        near = float((np.array(ev_dmin) < 450).mean())
        print(f"  450m 안쪽에서 발생한 비율 {near*100:.0f}%  "
              f"({'과근접이 원인' if near > 0.6 else '과근접 특정 안 됨'})")
    else:
        print("  오버슛 없음")
    print("=" * 92)


if __name__ == "__main__":
    main()
