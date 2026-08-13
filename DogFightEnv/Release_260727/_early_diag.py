"""**초반 국면**이 승패를 어떻게 가르는지 30시드 집계로 진단한다 — 개별 시드 금지.

08-13 배경. v42 상대 30판이 이분법으로 갈린다:
    seed 0  득점 0.590  공세  7.6%  -> 승
    seed 1  득점 0.000  공세  0.0%  -> 패
    seed 2  득점 0.000  공세  0.0%  -> 패
**패배 판의 공세 시간이 예외 없이 정확히 0.0%**다(ATA 최소 180.0도 = 200초 내내 기수를
한 번도 못 돌림). `Functions.cpp`에 v29 상대로 기록된 것과 같은 패턴이다.
=> 승부는 초반에 결정되고 이후 200초는 되돌릴 수 없다.

⚠️ **과적합 금지.** 시드 1 시계열만 보면 "우리가 8721m -> 1340m로 급강하해 에너지를
잃는다"는 이야기가 만들어지는데, 교착 진단의 고도차 평균은 승리 -89 / 패배 -99·+181·+124로
**부호가 갈린다.** 한 시드의 사정을 법칙으로 읽으면 안 된다. 그래서 이 도구는 개별
시계열을 찍지 않고 **승리군 대 패배군의 분포만** 비교한다.

재는 것 (전부 초반 WindowSec 구간):
  · 먼저 공세를 잡는 쪽과 그 시각
  · 초반 고도차·속도차·거리의 중앙값
  · 초반 선회 방향 일치(동선회) 비율 -> one-circle 대 two-circle
  · 첫 근접 교차(머지) 시각과 그때의 양쪽 ATA

사용:
    python _early_diag.py --target AIP_ryujan_v42.dll --num-seeds 30
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

FRONT, REAR = 50.0, 130.0


def wrap180(x):
    return (x + 180.0) % 360.0 - 180.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ownship", default="AIP_DCS.dll")
    ap.add_argument("--target", required=True)
    ap.add_argument("--num-seeds", type=int, default=30)
    ap.add_argument("--seed-offset", type=int, default=0)
    ap.add_argument("--window", type=float, default=40.0, help="초반으로 볼 구간(초)")
    args = ap.parse_args()

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
        t_off = t_def = None          # 먼저 공세/수세가 성립한 시각
        eh, ev, ed = [], [], []       # 초반 고도차·속도차·거리
        corot = neu = 0
        prev_h = prev_th = None
        merge_t = None                # 첫 근접 교차(거리 최소점 후보)
        merge_ma = merge_ta = None
        min_d_early = 1e9
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
            if t_off is None and ma < FRONT and ta > REAR:
                t_off = t_s
            if t_def is None and ta < FRONT and ma > REAR:
                t_def = t_s
            if t_s <= args.window:
                eh.append(-float(o[2]) - (-float(t[2])))
                ev.append(float(o[6]) - float(t[6]))
                ed.append(d)
                if d < min_d_early:
                    min_d_early, merge_t, merge_ma, merge_ta = d, t_s, ma, ta
                h, th = float(o[5]), float(t[5])
                if prev_h is not None:
                    r1, r2 = wrap180(h - prev_h), wrap180(th - prev_th)
                    if abs(r1) > 0.02 and abs(r2) > 0.02:
                        neu += 1
                        if r1 * r2 > 0:
                            corot += 1
                prev_h, prev_th = h, th
        env.close()
        res = "패" if th_hp > my_hp else ("승" if my_hp > th_hp else "무")
        rows.append(dict(
            seed=seed, res=res, my=my_hp, th=th_hp,
            t_off=(t_off if t_off is not None else 999.0),
            t_def=(t_def if t_def is not None else 999.0),
            dH=float(np.median(eh)), dV=float(np.median(ev)), dist=float(np.median(ed)),
            corot=(100.0 * corot / neu if neu else 0.0),
            merge_t=merge_t or 0.0, merge_d=min_d_early,
            merge_ma=merge_ma or 180.0, merge_ta=merge_ta or 180.0))
        r = rows[-1]
        print(f"  seed {seed:>2} {r['res']} 공세시각 {r['t_off']:5.1f} 수세시각 {r['t_def']:5.1f} | "
              f"초반 dH{r['dH']:+6.0f} dV{r['dV']:+5.1f} 거리{r['dist']:5.0f} 동선회{r['corot']:3.0f}% | "
              f"머지 t{r['merge_t']:5.1f} d{r['merge_d']:5.0f} 내ATA{r['merge_ma']:5.1f} 적ATA{r['merge_ta']:5.1f}",
              flush=True)

    print("\n" + "=" * 104)
    print(f"[초반 국면 진단] {args.num_seeds}시드  상대={args.target}  초반={args.window:.0f}초")
    groups = [("승", [r for r in rows if r["res"] == "승"]),
              ("무", [r for r in rows if r["res"] == "무"]),
              ("패", [r for r in rows if r["res"] == "패"])]
    print(f"  {'군':<3} {'n':>3} | {'먼저공세%':>8} {'공세시각':>8} {'수세시각':>8} | "
          f"{'초반dH':>7} {'초반dV':>7} {'초반거리':>8} {'동선회%':>7} | "
          f"{'머지시각':>8} {'머지거리':>8} {'머지내ATA':>9} {'머지적ATA':>9}")
    for name, rs in groups:
        if not rs:
            print(f"  {name:<3} {0:>3} | 없음")
            continue
        f = lambda k: np.median([r[k] for r in rs])
        first = 100.0 * np.mean([r["t_off"] < r["t_def"] for r in rs])
        print(f"  {name:<3} {len(rs):>3} | {first:>8.0f} {f('t_off'):>8.1f} {f('t_def'):>8.1f} | "
              f"{f('dH'):>+7.0f} {f('dV'):>+7.1f} {f('dist'):>8.0f} {f('corot'):>7.0f} | "
              f"{f('merge_t'):>8.1f} {f('merge_d'):>8.0f} {f('merge_ma'):>9.1f} {f('merge_ta'):>9.1f}")
    print("\n  ➜ 승리군과 패배군에서 **부호가 갈리는 열**만 원인 후보다.")
    print("     한 열이라도 두 군의 분포가 겹치면 그 열은 원인이 아니다(오늘 dH가 그랬다).")
    print("=" * 104)


if __name__ == "__main__":
    main()
