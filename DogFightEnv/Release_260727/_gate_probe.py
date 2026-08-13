"""LastDitch 발동 게이트가 실제로 몇 %에서 열리는지, 어느 조건이 막는지 센다.

08-13. `LastDitch`를 넣었더니 v42 상대 30시드가 **기준선과 소수 4자리까지 동일**했다.
= 한 번도 발동하지 않았다. 게이트를 추측으로 넓히지 않고 어느 조건이 막는지 먼저 잰다.

게이트(현재 값):
    상대ATA < 30도    내가 상대의 조준선 안에 있다
    내ATA  > 120도    나는 상대를 못 겨눈다 (확실한 수세)
    거리   < 600m     오버슛이 실제로 일어날 수 있는 근접
    고도  >= 1500m
    위 조건 2초 연속 유지

각 조건을 **단독**으로도 세고 **누적 AND**로도 세서, 어디서 급감하는지 본다.
연속 유지 요구가 얼마나 깎는지도 따로 본다.

사용:
    python _gate_probe.py --target AIP_ryujan_v42.dll --num-seeds 10
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ownship", default="AIP_DCS.dll")
    ap.add_argument("--target", required=True)
    ap.add_argument("--num-seeds", type=int, default=10)
    ap.add_argument("--seed-offset", type=int, default=0)
    args = ap.parse_args()

    ma_l, ta_l, d_l, alt_l = [], [], [], []
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
        while not (terminated or truncated):
            _, _, terminated, truncated, _ = env.step(np.zeros(4, dtype=np.float32))
            o, t = env._ownship_state, env._target_state
            g = env._geo_info
            ma_l.append(abs(float(g._get_antenna_train_angle(o, t, False))))
            ta_l.append(abs(float(g._get_antenna_train_angle(t, o, False))))
            d_l.append(float(g._get_distance(o, t)))
            alt_l.append(-float(o[2]))
        env.close()
        print(f"  seed {seed} 완료", flush=True)

    ma = np.array(ma_l); ta = np.array(ta_l); d = np.array(d_l); alt = np.array(alt_l)
    n = len(ma)

    def run_lengths(mask, min_steps):
        """mask가 min_steps 이상 연속인 구간의 총 스텝 수와 구간 개수."""
        cnt = tot = 0
        run = 0
        for v in mask:
            if v:
                run += 1
            else:
                if run >= min_steps:
                    cnt += 1
                    tot += run
                run = 0
        if run >= min_steps:
            cnt += 1
            tot += run
        return cnt, tot

    print("\n" + "=" * 92)
    print(f"[LastDitch 게이트 진단] {args.num_seeds}시드  상대={args.target}  샘플 {n}")
    print("\n  단독 조건 통과율")
    conds = [
        ("상대ATA < 30도", ta < 30.0),
        ("상대ATA < 50도", ta < 50.0),
        ("상대ATA < 90도", ta < 90.0),
        ("내ATA  > 120도", ma > 120.0),
        ("내ATA  > 90도",  ma > 90.0),
        ("거리   < 600m",  d < 600.0),
        ("거리   < 914m",  d < 914.4),
        ("거리   < 1500m", d < 1500.0),
        ("고도  >= 1500m", alt >= 1500.0),
    ]
    for name, m in conds:
        print(f"    {name:<16} {100*m.mean():6.2f}%")

    print("\n  누적 AND (현재 게이트를 하나씩 쌓는다)")
    acc = np.ones(n, dtype=bool)
    for name, m in [("상대ATA<30", ta < 30.0), ("+ 내ATA>120", ma > 120.0),
                    ("+ 거리<600", d < 600.0), ("+ 고도>=1500", alt >= 1500.0)]:
        acc = acc & m
        c2, t2 = run_lengths(acc, int(2.0 / DT))
        print(f"    {name:<14} {100*acc.mean():6.2f}%   2초연속 구간 {c2:>3}개 "
              f"({t2*DT:7.1f}s)")

    print("\n  게이트를 하나씩 푼 조합 (2초 연속 구간 개수 = 실제 발동 횟수)")
    variants = [
        ("현재값                 ATA30/120/600m", (ta < 30) & (ma > 120) & (d < 600)),
        ("거리만 완화            ATA30/120/914m", (ta < 30) & (ma > 120) & (d < 914.4)),
        ("거리·각도 완화         ATA50/90 /914m", (ta < 50) & (ma > 90) & (d < 914.4)),
        ("거리 크게 완화         ATA50/90 /1500m", (ta < 50) & (ma > 90) & (d < 1500)),
        ("수세 정의 그대로       ATA50/130/거리무관", (ta < 50) & (ma > 130)),
    ]
    for name, m in variants:
        m = m & (alt >= 1500.0)
        c2, t2 = run_lengths(m, int(2.0 / DT))
        c1, _ = run_lengths(m, int(1.0 / DT))
        print(f"    {name:<38} {100*m.mean():6.2f}%  "
              f"1초연속 {c1:>3}회  2초연속 {c2:>3}회 ({t2*DT:7.1f}s)")
    print("\n  ➜ 30판에서 2초연속이 0회면 그 조합으로는 영원히 안 켜진다.")
    print("=" * 92)


if __name__ == "__main__":
    main()
