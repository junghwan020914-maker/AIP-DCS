"""추락 안전성 확인 — 규정상 고도 300m 이하는 **즉시 패배**다.

08-08 제어기 2차 수정(Roll_Effect 코사인)으로 중간 UT각 피치 권한이 늘었다. 더 급격히
당긴다는 뜻이라 고도 손실이 커졌을 수 있다. 공식 최저 시작고도는 610m이고 우리
PreventLandCrash FloorHard는 800m라 여유가 크지 않다.

각 시드의 최저 고도와 종료 사유를 기록해 추락/근접 사례를 센다.
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

from DogFightEnvWrapper import DogFightWrapper
from dogfight.ai.bt_action_provider import BTActionProvider

FEET = 0.3048
SEP_FT = (2000.0, 2500.0, 3000.0)


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
            [pn, pe, -alt_m, 0.0, 0.0, (hdg + 180.0) % 360.0, spd]), alt_m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ownship", default="AIP_DCS.dll")
    ap.add_argument("--target", default="AIP_v32.dll")
    ap.add_argument("--num-seeds", type=int, default=40)
    args = ap.parse_args()

    rows = []
    for seed in range(args.num_seeds):
        rng = np.random.default_rng(seed)
        (own, tgt), alt0 = make_state(rng)
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
        my_min = tgt_min = 1e9
        steps = 0
        while not (terminated or truncated):
            _, _, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
            steps += 1
            # 고도: state의 D 성분이 음수 = 위쪽 (NED)
            my_min = min(my_min, -float(env._ownship_state[2]))
            tgt_min = min(tgt_min, -float(env._target_state[2]))
        rows.append(dict(seed=seed, alt0=alt0, my_min=my_min, tgt_min=tgt_min,
                         secs=steps / 60.0,
                         oh=float(info.get("ownship_health", 1.0)),
                         th=float(info.get("target_health", 1.0))))
        env.close()
        print(f"  seed {seed:>2}  시작고도 {alt0:6.0f}m  내최저 {my_min:6.0f}m  "
              f"상대최저 {tgt_min:6.0f}m  {rows[-1]['secs']:5.1f}s", flush=True)

    mm = np.array([r["my_min"] for r in rows])
    tm = np.array([r["tgt_min"] for r in rows])
    a0 = np.array([r["alt0"] for r in rows])
    sec = np.array([r["secs"] for r in rows])
    print("\n" + "=" * 70)
    print(f"[추락 안전성] {args.num_seeds}시드  나={args.ownship}  상대={args.target}")
    print(f"  내 최저고도   : 중앙값 {np.median(mm):6.0f}m  최소 {mm.min():6.0f}m  "
          f"<800m {int((mm<800).sum())}판  <500m {int((mm<500).sum())}판  "
          f"<300m(추락) {int((mm<300).sum())}판")
    print(f"  상대 최저고도 : 중앙값 {np.median(tm):6.0f}m  최소 {tm.min():6.0f}m  "
          f"<300m(추락) {int((tm<300).sum())}판")
    print(f"  시작고도 1500m 미만 시드 {int((a0<1500).sum())}판, "
          f"그중 내 최저 <500m {int(((a0<1500)&(mm<500)).sum())}판")
    print(f"  조기종료(200초 미만) {int((sec<199).sum())}판")
    print("=" * 70)


if __name__ == "__main__":
    main()
