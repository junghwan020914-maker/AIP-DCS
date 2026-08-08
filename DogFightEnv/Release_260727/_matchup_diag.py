"""매치업 진단 — 득점이 '상호 교환'인지 '일방 우위'인지 가른다.

08-08 자체 아키타입 arcA(앵글 파이터) 상대로 순이득이 -0.0074(17승13패)까지 무너졌다.
콘이 좁아질수록 우위가 사라지는 패턴(10도 3.4:1 -> 1도 0.9:1)이고, 조준기회를 득점으로
바꾸는 변환율이 우리 10.3% vs arcA 39.9%로 4배 차이난다.

가설 두 개가 대응이 완전히 다르다:
  (A) 상호 정면교환 — arcA가 항상 우리 쪽으로 선회해 헤드온을 강요 -> 양쪽이 **동시에**
      득점 -> 대등 교환 반복 -> 순이득 0 수렴. 대응: 머지 전에 각을 벌거나 수직 사용.
  (B) 일방 우위 — arcA가 우리와 **별개 시점**에 각을 잡음. 대응: BFM 자체 개선.

동시 득점 틱을 세면 바로 갈린다. 추가로 득점 시점의 거리·상대각 분포도 본다.
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
PHASES = (
    (1.0, 500.0 * FEET, 3000.0 * FEET, 1.0, 0.0),
    (2.0, 500.0 * FEET, 3500.0 * FEET, 0.3, 100.0),
    (3.0, 500.0 * FEET, 4000.0 * FEET, 0.1, 150.0),
)


def score_rate(dis_m, ata_deg, t_s):
    a = abs(ata_deg)
    for cone, mn, mx, coef, start in PHASES:
        if t_s < start:
            continue
        if mn <= dis_m <= mx and a <= cone:
            return coef * (mx - dis_m) / (mx - mn)
    return 0.0


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ownship", default="AIP_DCS.dll")
    ap.add_argument("--target", required=True)
    ap.add_argument("--num-seeds", type=int, default=30)
    args = ap.parse_args()

    n = args.num_seeds
    both = mine = theirs = 0                 # 득점 틱 분류
    hp_both_me = hp_both_th = 0.0            # 동시 구간에서 각자 번 HP
    hp_mine = hp_theirs = 0.0                # 일방 구간
    d_both, d_mine, d_theirs = [], [], []
    # 상대가 득점할 때 나는 어디를 보고 있었나(내 ATA 분포)
    my_ata_when_hit = []

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
        step = 0
        while not (terminated or truncated):
            _, _, terminated, truncated, _ = env.step(np.zeros(4, dtype=np.float32))
            step += 1
            t_s = step * DT
            g = env._geo_info
            d = float(g._get_distance(env._ownship_state, env._target_state))
            ma = abs(float(g._get_antenna_train_angle(
                env._ownship_state, env._target_state, False)))
            ta = abs(float(g._get_antenna_train_angle(
                env._target_state, env._ownship_state, False)))
            rm = score_rate(d, ma, t_s)
            rt = score_rate(d, ta, t_s)
            if rm > 0 and rt > 0:
                both += 1
                hp_both_me += rm * DT
                hp_both_th += rt * DT
                d_both.append(d)
            elif rm > 0:
                mine += 1
                hp_mine += rm * DT
                d_mine.append(d)
            elif rt > 0:
                theirs += 1
                hp_theirs += rt * DT
                d_theirs.append(d)
                my_ata_when_hit.append(ma)
        env.close()
        print(f"  seed {seed:>2} 완료", flush=True)

    def line(label, cnt, hp_a, hp_b, ds):
        md = f"{np.median(ds):6.1f}m" if ds else "   -  "
        b = f" / 상대 {hp_b/n:.4f}" if hp_b is not None else ""
        print(f"  {label:<16} {cnt/60/n:6.2f}s/판  나 {hp_a/n:.4f}{b}   거리중앙값 {md}")

    tot = both + mine + theirs
    print("\n" + "=" * 74)
    print(f"[매치업 진단] {n}시드  나={args.ownship}  상대={args.target}")
    print(f"  득점 틱 분류 (총 {tot/60/n:.2f}s/판)")
    line("동시 득점", both, hp_both_me, hp_both_th, d_both)
    line("나만 득점", mine, hp_mine, None, d_mine)
    line("상대만 득점", theirs, hp_theirs, None, d_theirs)
    if tot:
        print(f"\n  동시 비율: {100*both/tot:4.1f}%   나만 {100*mine/tot:4.1f}%   "
              f"상대만 {100*theirs/tot:4.1f}%")
    net = (hp_both_me + hp_mine - hp_both_th - hp_theirs) / n
    print(f"  순이득 {net:+.4f} HP/판  "
          f"(동시구간 {(hp_both_me-hp_both_th)/n:+.4f}, 일방구간 {(hp_mine-hp_theirs)/n:+.4f})")
    if my_ata_when_hit:
        a = np.array(my_ata_when_hit)
        print(f"\n  상대만 득점할 때 내 ATA: 중앙값 {np.median(a):5.1f}도  "
              f"<10도 {100*(a<10).mean():4.1f}%  <30도 {100*(a<30).mean():4.1f}%  "
              f">90도 {100*(a>90).mean():4.1f}%")
    print("=" * 74)


if __name__ == "__main__":
    main()
