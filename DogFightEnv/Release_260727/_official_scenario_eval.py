"""대회 공식 초기조건으로 평가하는 배치 스크립트.

08-06 주최측 Q&A(질문 98번) 확인 결과, 실제 대회 초기조건은 로컬 기본 시나리오풀
(ref_old_random)과 크게 다르다:
    거리   2000/2500/3000ft (610/762/914m)   <- 로컬은 1000~2000m
    고도   2000~30000ft 랜덤 (610~9144m)      <- 로컬은 8000m 고정
    속도   200~300m/s 랜덤                    <- 로컬은 250m/s 고정
    구도   3-9 라인 정면(헤드온)

로컬 풀에 맞춰 튜닝하면 대회에서 안 나오는 조건에 과적합될 수 있으므로,
공식 조건을 재현해 별도 기준선으로 삼는다.

사용:
    python _official_scenario_eval.py --target AIP_v32.dll --num-seeds 30
Not part of the student template; safe to delete after use.
"""
from __future__ import annotations

import argparse
import statistics
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
SEP_FT = (2000.0, 2500.0, 3000.0)      # 뷰어에서 고를 수 있는 3가지 이격거리


def make_state(rng):
    """공식 초기조건 1세트 생성. [N, E, D, roll, pitch, heading, speed] 두 대."""
    sep_m = float(rng.choice(SEP_FT)) * FEET
    alt_m = float(rng.uniform(2000.0, 30000.0)) * FEET      # 610~9144m
    spd = float(rng.uniform(200.0, 300.0))                  # 200~300 m/s
    hdg = float(rng.uniform(0.0, 360.0))                    # 전장 방위는 무작위

    # 3-9 라인 정면: 서로 마주보고 sep_m 만큼 떨어뜨린다.
    half = sep_m / 2.0
    rad = np.deg2rad(hdg)
    # 08-08 정정: 2000/2500/3000ft는 **기체 간 거리**이고 두 기체는 서로의 3-9 라인
    # (날개 축)에 나란히 놓여 **반대 방향**을 본다. 기수 대 기수(헤드온)가 아니다.
    # 이전 구현은 헤드온이라 시작 즉시 서로 WEZ 안이었다.
    prad = np.deg2rad(hdg + 90.0)
    pn, pe = np.cos(prad) * half, np.sin(prad) * half
    own = [-pn, -pe, -alt_m, 0.0, 0.0, hdg, spd]
    tgt = [pn, pe, -alt_m, 0.0, 0.0, (hdg + 180.0) % 360.0, spd]
    return own, tgt, sep_m, alt_m, spd


def classify(info):
    oh = float(info.get("ownship_health", 1.0))
    th = float(info.get("target_health", 1.0))
    if th <= 0.0 and oh > 0.0:
        return "WIN"
    if oh <= 0.0 and th > 0.0:
        return "LOSS"
    if oh <= 0.0 and th <= 0.0:
        return "BOTH"
    if oh > th:
        return "WIN"
    if th > oh:
        return "LOSS"
    return "DRAW"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ownship", default="AIP_DCS.dll")
    ap.add_argument("--target", required=True)
    ap.add_argument("--num-seeds", type=int, default=30)
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    rows = []
    for seed in range(args.num_seeds):
        rng = np.random.default_rng(seed)
        own, tgt, sep_m, alt_m, spd = make_state(rng)
        env = DogFightWrapper(
            env_config={
                "observation_mode": "tactical16",
                "ownship_control_mode": "rl",
                "target_mode": "rl",
                "max_engage_time": 200.0,
                "min_altitude": 300.0,
                "ownship": own,
                "target": tgt,
                "initial_scenario": {"mode": "default"},   # 위 좌표를 그대로 사용
            },
            ownship_action_provider=BTActionProvider(dll_name=args.ownship),
            target_action_provider=BTActionProvider(dll_name=args.target),
        )
        env.reset(seed=seed)
        terminated = truncated = False
        info = {}
        while not (terminated or truncated):
            _, _, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
        res = classify(info)
        rows.append({
            "seed": seed, "res": res,
            "oh": float(info.get("ownship_health", 1.0)),
            "th": float(info.get("target_health", 1.0)),
            "wez": int(info.get("ep_wez_steps", 0)),
            "sep": sep_m, "alt": alt_m, "spd": spd,
        })
        if not args.quiet:
            print(f"seed {seed:>3} | {res:<5} | oh {rows[-1]['oh']:.3f} th {rows[-1]['th']:.3f} "
                  f"| WEZ {rows[-1]['wez']:>4} | 거리 {sep_m:.0f}m 고도 {alt_m:.0f}m 속도 {spd:.0f}",
                  flush=True)
        env.close()

    w = sum(1 for r in rows if r["res"] == "WIN")
    l = sum(1 for r in rows if r["res"] == "LOSS")
    d = len(rows) - w - l
    print("=" * 74)
    print(f"[대회 공식 초기조건] seeds={len(rows)}  ownship={args.ownship}  target={args.target}")
    print(f"  win/loss/draw = {w}/{l}/{d}")
    print(f"  WEZ 발생시드 = {sum(1 for r in rows if r['wez'] > 0)}/{len(rows)}  "
          f"평균 WEZ스텝 = {statistics.fmean([r['wez'] for r in rows]):.1f}")
    print("=" * 74)


if __name__ == "__main__":
    main()
