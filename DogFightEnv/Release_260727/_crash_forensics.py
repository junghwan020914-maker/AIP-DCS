"""추락 경위 추적 — 왜 PreventLandCrash가 못 막는가.

08-08 실측: 플로어를 800/1500 -> 1000/2000 으로 올렸는데 **추락이 8판 그대로**였고
성능만 무너졌다(순이득 평균 +0.6247 -> +0.4599). 즉 추락 원인은 발동 시점이 아니다.

가설: 급강하에 들어간 뒤엔 물리적으로 못 빠져나온다.
  PreventLandCrash는 하드플로어 아래에서 VP를 '내위치 + 수평1500 + 상방8000'으로 덮지만,
  그 뒤 CPPBehaviorTree의 **75도 보어사이트 클램프**가 걸린다. 기수가 아래로 60도 이상
  꽂혀 있으면 상승 VP가 기수에서 75도를 넘어 잘리고, 회복이 틱마다 조금씩만 된다.
  300m/s로 꽂히는 중이면 남은 고도가 회복 시간보다 부족하다.

측정: 추락 시드에서 고도 1500m/1000m/500m 통과 시점의 **피치각·속도·강하율**과
      최저점까지 걸린 시간을 기록한다. 강하각이 크고 속도가 높다면 가설 지지.
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

from _aim_time_probe import make_state  # noqa: E402
from DogFightEnvWrapper import DogFightWrapper  # noqa: E402
from dogfight.ai.bt_action_provider import BTActionProvider  # noqa: E402

DT = 1.0 / 60.0
GATES = (1500.0, 1000.0, 500.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ownship", default="AIP_DCS.dll")
    ap.add_argument("--target", default="AIP_v29.dll")
    ap.add_argument("--num-seeds", type=int, default=30)
    args = ap.parse_args()

    crashes = []
    for seed in range(args.num_seeds):
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
        prev_alt = None
        marks = {}           # gate -> (t, pitch, spd, vs)
        min_alt = 1e9
        while not (terminated or truncated):
            _, _, terminated, truncated, _ = env.step(np.zeros(4, dtype=np.float32))
            step += 1
            s = env._ownship_state
            alt = -float(s[2])
            pitch = float(np.degrees(s[4])) if abs(float(s[4])) <= 3.2 else float(s[4])
            spd = float(np.linalg.norm([s[6], s[7], s[8]]))
            vs = (alt - prev_alt) / DT if prev_alt is not None else 0.0
            prev_alt = alt
            min_alt = min(min_alt, alt)
            for g in GATES:
                if g not in marks and alt < g:
                    marks[g] = (step * DT, pitch, spd, vs)
        env.close()
        if min_alt < 300.0:
            crashes.append((seed, min_alt, marks))
            m = marks.get(1500.0)
            g5 = marks.get(500.0)
            span = (g5[0] - m[0]) if (m and g5) else float("nan")
            print(f"  [추락] seed {seed:>2}  최저 {min_alt:5.0f}m", flush=True)
            for g in GATES:
                if g in marks:
                    t, p, sp, vs = marks[g]
                    print(f"        {g:>5.0f}m 통과  t={t:6.1f}s  피치 {p:+6.1f}도  "
                          f"속도 {sp:5.0f}m/s  강하율 {vs:+7.0f}m/s", flush=True)
            print(f"        1500m→500m 소요 {span:.1f}s", flush=True)

    print("\n" + "=" * 74)
    print(f"[추락 경위] {args.num_seeds}시드  나={args.ownship}  상대={args.target}")
    print(f"  추락 {len(crashes)}판")
    if crashes:
        p1500 = [c[2][1500.0][1] for c in crashes if 1500.0 in c[2]]
        s1500 = [c[2][1500.0][2] for c in crashes if 1500.0 in c[2]]
        v1500 = [c[2][1500.0][3] for c in crashes if 1500.0 in c[2]]
        if p1500:
            print(f"  1500m 통과 시점 — 피치 중앙값 {np.median(p1500):+.1f}도  "
                  f"속도 {np.median(s1500):.0f}m/s  강하율 {np.median(v1500):+.0f}m/s")
            print(f"  ※ 강하율이 크고 피치가 깊게 음수면 '이미 꽂힌 뒤라 못 뺀다' 가설 지지")
    print("=" * 74)


if __name__ == "__main__":
    main()
