"""득점이 실제로 '어느 거리에서' 발생하는지 측정한다.

08-07 주최측 추가 Q&A로 데미지 산식이 확정됐다(HP=100, 거리에 반비례):
    d_wez = 0                      (r > 3000ft)
    d_wez = (3000 - r) / 2500      (500ft <= r <= 3000ft)
    d_wez = 0                      (r < 500ft)
즉 152m에서 계수 1.0, 914m에서 계수 0.0인 선형 램프다. 우리 트리는 지금까지
152~914m를 균질한 "WEZ"로 취급하고 그 안에 들어가면 Pure로 조준만 해왔는데,
밴드 안에서도 어디서 맞추느냐에 따라 득점이 최대 수십 배 차이난다.

가설: 우리 득점 틱은 밴드의 바깥쪽(800m대)에 몰려있어 계수가 0.05 수준이다.
      같은 조준을 200m에서 했다면 계수 0.94로 ~20배였다.

측정: 매 틱 양측 Phase1 판정을 복제해 득점 틱의 거리·계수 분포를 낸다.
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
MIN_M, MAX_M = 500.0 * FEET, 3000.0 * FEET      # 152.4 , 914.4
BASE_M = MAX_M - MIN_M                           # 762.0


def coef(r):
    """공식 데미지 계수(거리항). 밴드 밖이면 0."""
    if r < MIN_M or r > MAX_M:
        return 0.0
    return (MAX_M - r) / BASE_M


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
    ap.add_argument("--target", default="AIP_v32.dll")
    ap.add_argument("--num-seeds", type=int, default=20)
    args = ap.parse_args()

    my_hits, their_hits = [], []      # (거리, 계수)
    band_dwell = []                   # 밴드 안 체류 거리 전부

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
        while not (terminated or truncated):
            _, _, terminated, truncated, _ = env.step(np.zeros(4, dtype=np.float32))
            g = env._geo_info
            d = float(g._get_distance(env._ownship_state, env._target_state))
            if not (MIN_M <= d <= MAX_M):
                continue
            band_dwell.append(d)
            c = coef(d)
            # Phase1: 3D ATA(proj=False) 1도 이내 -- 실제 데미지 판정과 동일
            if abs(float(g._get_antenna_train_angle(
                    env._ownship_state, env._target_state, False))) <= 1.0:
                my_hits.append((d, c))
            if abs(float(g._get_antenna_train_angle(
                    env._target_state, env._ownship_state, False))) <= 1.0:
                their_hits.append((d, c))
        env.close()
        print(f"  seed {seed:>2} 완료  (내 득점틱 {len(my_hits)} / 상대 {len(their_hits)})",
              flush=True)

    def report(hits, label):
        print(f"\n[{label}]  득점틱 {len(hits)}개")
        if not hits:
            print("  (없음)")
            return
        r = np.array([h[0] for h in hits])
        c = np.array([h[1] for h in hits])
        print(f"  거리   중앙값 {np.median(r):6.1f}m   평균 {r.mean():6.1f}m   "
              f"최소 {r.min():6.1f}m  최대 {r.max():6.1f}m")
        print(f"  계수   중앙값 {np.median(c):6.3f}    평균 {c.mean():6.3f}   "
              f"(1.0=152m 최대,  0.0=914m 무효)")
        print(f"  실제 누적데미지 = {c.sum() / 60:.4f} HP   "
              f"(같은 조준을 200m에서 했다면 {0.937 * len(c) / 60:.4f} HP, "
              f"{0.937 * len(c) / max(c.sum(), 1e-9):.1f}배)")

    print("\n" + "=" * 72)
    print(f"공식조건 {args.num_seeds}시드   나={args.ownship}  상대={args.target}")
    report(my_hits, "내 득점")
    report(their_hits, "상대 득점")
    if band_dwell:
        b = np.array(band_dwell)
        print(f"\n[밴드(152~914m) 체류]  총 {len(b)/60:.1f}s   거리 중앙값 {np.median(b):6.1f}m"
              f"   평균계수 {np.mean([coef(x) for x in b]):.3f}")
        for lo, hi in ((152, 300), (300, 500), (500, 700), (700, 914)):
            n = int(((b >= lo) & (b < hi)).sum())
            print(f"    {lo:>3}~{hi:>3}m : {n/60:6.1f}s  ({100*n/len(b):4.1f}%)  "
                  f"계수 {coef((lo+hi)/2):.2f}")
    print("=" * 72)


if __name__ == "__main__":
    main()
