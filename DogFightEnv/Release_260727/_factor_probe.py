"""득점을 **두 요인으로 분해**해 상대별로 잰다 — 밴드 체류 x 사격각 유지.

08-13 실서버 두 판이 정반대 실패를 보였다:

    상대   밴드체류   ATA<1도 시간   결과
    arcE    8.7%      71.23%       67초 격추   <- 조준은 완벽, 못 붙는다
    v29    37.4%       0.00%       200초 HP10  <- 붙기는 하는데 못 겨눈다

**격추에는 둘 다 필요하고, 우리는 상대에 따라 다른 쪽이 무너진다.**
"조준이 문제다" / "거리가 문제다" 어느 한쪽으로 단정하면 절반의 상대에만 듣는 처방이 된다.
(내가 arcE 판 하나를 보고 "문제는 조준이 아니라 거리"라고 단정했다가 v29 판에서
 뒤집혔다 — 상대 하나로 일반화한 오류.)

그래서 상대별로 다음을 나눠 잰다:
    T_band      밴드(152.4~914.4m) 체류 s/판
    P_aim|band  밴드 안에서 ATA<1도인 비율  (= 붙었을 때 겨눌 수 있나)
    T_hit       둘 다 만족 s/판 (= 실제 득점 시간)
    T_waste     ATA<1도인데 밴드 밖 s/판 (= 헛되이 겨눈 시간)
    d_wez평균   득점 순간의 거리계수 (안쪽일수록 크다)
    예상HP      실서버 모델(경기시계 x75) 환산

⚠️ x75는 08-13 실서버 실측 1점(arcE 격추: 경기시계 누적 1.3303 -> 100HP)에서 나온 값이고
   ±2배 불확실하다. **절대값이 아니라 상대별 순위와 병목 판별에 쓸 것.**

사용:
    python _factor_probe.py --targets AIP_v32.dll,AIP_v29.dll,... --num-seeds 10
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
MULT = 75.0


def run(ownship, target, n, off):
    band = hit = waste = 0
    band_t = 0
    dw = []
    my_hp = th_hp = 0.0
    th_band = th_hit = 0
    steps = 0
    for seed in range(off, off + n):
        rng = np.random.default_rng(seed)
        own, tgt = make_state(rng)
        env = DogFightWrapper(
            env_config={
                "observation_mode": "tactical16", "ownship_control_mode": "rl",
                "target_mode": "rl", "max_engage_time": 200.0, "min_altitude": 300.0,
                "ownship": own, "target": tgt,
                "initial_scenario": {"mode": "default"},
            },
            ownship_action_provider=BTActionProvider(dll_name=ownship),
            target_action_provider=BTActionProvider(dll_name=target),
        )
        env.reset(seed=seed)
        terminated = truncated = False
        step = 0
        while not (terminated or truncated):
            _, _, terminated, truncated, _ = env.step(np.zeros(4, dtype=np.float32))
            step += 1
            t_s = step * DT
            g, o, t = env._geo_info, env._ownship_state, env._target_state
            d = float(g._get_distance(o, t))
            ma = abs(float(g._get_antenna_train_angle(o, t, False)))
            ta = abs(float(g._get_antenna_train_angle(t, o, False)))
            inb = BAND_MIN <= d <= BAND_MAX
            if inb:
                band += 1
                if ma <= 1.0:
                    hit += 1
                    dw.append((BAND_MAX - d) / (BAND_MAX - BAND_MIN))
                if ta <= 1.0:
                    th_hit += 1
                th_band += 1
            elif ma <= 1.0:
                waste += 1
            my_hp += score_rate(d, ma, t_s)[0] * DT
            th_hp += score_rate(d, ta, t_s)[0] * DT
        steps += step
        env.close()
    s = lambda c: c * DT / n
    return dict(target=target, n=n,
                T_band=s(band), P_aim=(100.0 * hit / band if band else 0.0),
                T_hit=s(hit), T_waste=s(waste),
                dwez=(float(np.mean(dw)) if dw else 0.0),
                my=my_hp / n, th=th_hp / n,
                th_aim=(100.0 * th_hit / th_band if th_band else 0.0),
                secs=steps * DT / n)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ownship", default="AIP_DCS.dll")
    ap.add_argument("--targets", required=True)
    ap.add_argument("--num-seeds", type=int, default=10)
    ap.add_argument("--seed-offset", type=int, default=0)
    args = ap.parse_args()

    rows = []
    for t in [x.strip() for x in args.targets.split(",") if x.strip()]:
        if not (ROOT / t).exists():
            print(f"  (없음) {t}", flush=True)
            continue
        r = run(args.ownship, t, args.num_seeds, args.seed_offset)
        rows.append(r)
        print(f"  {t:<20} 밴드{r['T_band']:6.1f}s 조준률{r['P_aim']:5.2f}% "
              f"득점{r['T_hit']:5.2f}s 헛조준{r['T_waste']:6.1f}s", flush=True)

    print("\n" + "=" * 112)
    print(f"[득점 요인 분해] {args.num_seeds}시드/상대  나={args.ownship}   "
          f"실서버 환산 x{MULT:.0f}(±2배 불확실)")
    print(f"  {'상대':<20} {'밴드체류':>8} {'밴드내조준률':>12} {'득점시간':>8} "
          f"{'헛조준':>7} {'d_wez':>6} | {'내득점':>8} {'예상HP':>7} | {'피격':>8} {'상대조준률':>10}")
    for r in rows:
        print(f"  {r['target']:<20} {r['T_band']:>7.1f}s {r['P_aim']:>11.2f}% "
              f"{r['T_hit']:>7.2f}s {r['T_waste']:>6.1f}s {r['dwez']:>6.3f} | "
              f"{r['my']:>8.4f} {r['my']*MULT:>7.1f} | {r['th']:>8.4f} {r['th_aim']:>9.2f}%")

    print("\n  ➜ 병목 판별")
    print(f"  {'상대':<20} {'병목':<28} 근거")
    for r in rows:
        if r["T_band"] < 10.0 and r["P_aim"] > 5.0:
            b, why = "접근(못 붙는다)", f"밴드 {r['T_band']:.1f}s인데 붙으면 조준률 {r['P_aim']:.1f}%"
        elif r["T_band"] >= 10.0 and r["P_aim"] < 1.0:
            b, why = "조준(못 겨눈다)", f"밴드 {r['T_band']:.1f}s나 되는데 조준률 {r['P_aim']:.2f}%"
        elif r["T_band"] < 10.0 and r["P_aim"] < 1.0:
            b, why = "둘 다", f"밴드 {r['T_band']:.1f}s / 조준률 {r['P_aim']:.2f}%"
        else:
            b, why = "없음(득점 중)", f"득점시간 {r['T_hit']:.2f}s"
        print(f"  {r['target']:<20} {b:<28} {why}")
    print("=" * 112)


if __name__ == "__main__":
    main()
