"""조준 정밀도 지표 — 제어기 작업의 주 계측기.

승률은 이미 90%대로 포화돼 있어 제어기 미세개선을 분해하지 못한다(40시드에서
1~2판 차이는 노이즈). 대신 **점수에 직접 비례하는 연속량**을 잰다.

08-07 확정된 채점식:
    d_wez(r) = (3000ft - r) / 2500ft     (500~3000ft, 밖은 0)
    Phase1 t>=0s   |ATA|<1도  152~914m   계수 1.0
    Phase2 t>=100s |ATA|<2도  152~1067m  계수 0.3
    Phase3 t>=150s |ATA|<3도  152~1219m  계수 0.1
    (안쪽 phase 우선, 중첩 합산 아님)

실측(공식조건 20시드 vs v32)으로 확인된 병목:
    밴드 체류 49.8s/판 -> |ATA|<10도 18.4s -> <3도 7.11s -> <1도 1.43s
즉 3도에서 1도로 좁히는 동안 시간의 80%가 날아간다. 이 구간을 회수하는 것이
현재 가장 큰 배율이고, 그건 VP 위치가 아니라 **제어기의 VP 추종 정확도** 문제다.

출력의 `HP` 열이 실제 채점과 같은 단위이므로 이것을 제어기 개선의 판정 기준으로 쓴다.
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
DT = 1.0 / 60.0

# (콘 반각 deg, 최소m, 최대m, 계수, 활성시각s)  -- 안쪽(높은 계수)부터
PHASES = (
    (1.0, 500.0 * FEET, 3000.0 * FEET, 1.0, 0.0),
    (2.0, 500.0 * FEET, 3500.0 * FEET, 0.3, 100.0),
    (3.0, 500.0 * FEET, 4000.0 * FEET, 0.1, 150.0),
)


def score_rate(dis_m, ata_deg, t_s):
    """실제 채점과 동일: 활성 phase 중 안쪽 우선, 첫 매칭만.

    반환: (점수율, phase번호)  phase번호 0=득점없음, 1/2/3
    """
    a = abs(ata_deg)
    for idx, (cone, mn, mx, coef, start) in enumerate(PHASES, start=1):
        if t_s < start:
            continue
        if mn <= dis_m <= mx and a <= cone:
            return coef * (mx - dis_m) / (mx - mn), idx
    return 0.0, 0


def make_state(rng):
    sep_m = float(rng.choice(SEP_FT)) * FEET
    alt_m = float(rng.uniform(2000.0, 30000.0)) * FEET
    spd = float(rng.uniform(200.0, 300.0))
    hdg = float(rng.uniform(0.0, 360.0))
    half = sep_m / 2.0
    rad = np.deg2rad(hdg)
    dn, de = np.cos(rad) * half, np.sin(rad) * half
    return ([-dn, -de, -alt_m, 0.0, 0.0, hdg, spd],
            [dn, de, -alt_m, 0.0, 0.0, (hdg + 180.0) % 360.0, spd])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ownship", default="AIP_DCS.dll")
    ap.add_argument("--target", default="AIP_v32.dll")
    ap.add_argument("--num-seeds", type=int, default=20)
    args = ap.parse_args()

    n = args.num_seeds
    # 누적 카운터
    my_cone = {1: 0, 2: 0, 3: 0, 5: 0, 10: 0}
    th_cone = {1: 0, 2: 0, 3: 0, 5: 0, 10: 0}
    my_hp = th_hp = 0.0
    my_phase = {1: 0.0, 2: 0.0, 3: 0.0}
    th_phase = {1: 0.0, 2: 0.0, 3: 0.0}
    band = 0
    total = 0
    wins = losses = 0

    for seed in range(n):
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
        info = {}
        step = 0
        while not (terminated or truncated):
            _, _, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
            step += 1
            t_s = step * DT
            g = env._geo_info
            d = float(g._get_distance(env._ownship_state, env._target_state))
            ma = abs(float(g._get_antenna_train_angle(
                env._ownship_state, env._target_state, False)))
            ta = abs(float(g._get_antenna_train_angle(
                env._target_state, env._ownship_state, False)))
            total += 1
            if 152.4 <= d <= 914.4:
                band += 1
                for c in my_cone:
                    if ma <= c:
                        my_cone[c] += 1
                    if ta <= c:
                        th_cone[c] += 1
            r, ph = score_rate(d, ma, t_s)
            my_hp += r * DT
            if ph:
                my_phase[ph] += r * DT
            r2, ph2 = score_rate(d, ta, t_s)
            th_hp += r2 * DT
            if ph2:
                th_phase[ph2] += r2 * DT

        oh = float(info.get("ownship_health", 1.0))
        th_h = float(info.get("target_health", 1.0))
        if th_h < oh:
            wins += 1
        elif oh < th_h:
            losses += 1
        env.close()
        print(f"  seed {seed:>2} 완료", flush=True)

    print("\n" + "=" * 70)
    print(f"[조준 정밀도] {n}시드  나={args.ownship}  상대={args.target}")
    print(f"  승/패 = {wins}/{losses}   밴드 체류 {band/60/n:5.1f}s/판 "
          f"({100*band/max(total,1):4.1f}%)")
    print(f"  {'콘':>6} | {'내 시간/판':>11} | {'상대 시간/판':>12} | {'비':>6}")
    for c in (10, 5, 3, 2, 1):
        m, t = my_cone[c] / 60 / n, th_cone[c] / 60 / n
        ratio = f"{m/t:5.1f}:1" if t > 1e-9 else "  inf "
        print(f"  {c:>4}도 | {m:>10.2f}s | {t:>11.2f}s | {ratio:>6}")
    print(f"\n  ★ 채점단위 누적 데미지(HP/판):  나 {my_hp/n:.4f}   상대 {th_hp/n:.4f}   "
          f"순이득 {(my_hp-th_hp)/n:+.4f}")
    print(f"  Phase별 내 득점 :  P1 {my_phase[1]/n:.4f}({100*my_phase[1]/max(my_hp,1e-9):4.1f}%)"
          f"  P2 {my_phase[2]/n:.4f}({100*my_phase[2]/max(my_hp,1e-9):4.1f}%)"
          f"  P3 {my_phase[3]/n:.4f}({100*my_phase[3]/max(my_hp,1e-9):4.1f}%)")
    print(f"  Phase별 상대득점:  P1 {th_phase[1]/n:.4f}  P2 {th_phase[2]/n:.4f}  "
          f"P3 {th_phase[3]/n:.4f}")
    print("=" * 70)


if __name__ == "__main__":
    main()
