"""v42전 승패를 가르는 **Phase 2/3 미세 피해**를 판별로 분해한다.

08-15 배경. `_viewer_score.py`의 결함 3건을 고치고 규정 모델(P1+P2+P3)이 실서버 5판과
모두 일치함을 확인했다. 그 결과 v42 매치업의 성격이 바뀌었다:

    뷰어 모델(철회)  1승  0패 19무  ->  10.5
    규정 모델(정답)  1승  4패 15무  ->   8.5

**200초 무승부 중 5판이 Phase 2/3의 미세 피해로 갈린다.** 그런데 숫자가 이상하다 —
평균 피해는 우리가 앞서는데(0.0295 대 0.0018) 4패 1승이다. **우리 피해는 이긴 한 판에
몰려 있고 그들은 여러 판에 얇게 깔고 있다**는 뜻이다.

이건 지금까지 v42에서 본 문제와 **종류가 다르다.** `_cross_probe.py`가 낸 결론은
"교차마다 그들이 우리 뒤, 필요 개선폭 133도 = 기하 문제"였고 그건 **Phase 1(1도) 격추**
이야기다. 그런데 규정 6조2항은 크기가 아니라 **부호**다. 0.0001만 앞서도 이긴다.

그리고 Phase 2/3은 훨씬 헐겁다:

    Phase 2  t>=100s  ATA<2도  152~1067m  계수 0.3
    Phase 3  t>=150s  ATA<3도  152~1219m  계수 0.1

`_cross_probe.py`에서 밴드 안 교차 시 우리 ATA 최솟값은 중앙 133.72도지만
**p10이 21.83도, 최소 1.51도**다. 5도 안에 든 교차가 3.0% 있다. 즉 **가끔은 들어간다.**
그 순간이 100초/150초 뒤였다면 점수가 된다.

재는 것 — 판마다 양쪽의 Phase별 피해와 체류시간:
  · P1/P2/P3 각각의 피해량과 그 조건을 만족한 시간
  · 승패와 **승부 마진**(내 총피해 - 상대 총피해)
  · 지는 판에서 상대 피해가 **어느 Phase에서** 나오는가
  · 후반 100초/50초의 sub-2도 / sub-3도 체류시간 양쪽 비교

⚠️ **판정 기준을 데이터 보기 전에 박는다.**

  · 패배 마진 중앙값이 **작고**(< 0.01) 우리 sub-3도 시간이 그들과 비슷하면
    -> **뒤집을 수 있는 동전던지기**다. 값싼 변경으로 4패가 움직인다.
  · 패배 마진이 크거나, 그들의 sub-2도 시간이 우리보다 **배수로** 많으면
    -> 후반에도 그들이 뒤에 눌러앉아 있다는 뜻이고 **133도 기하 문제와 같은 벽**이다.
    Phase 2/3은 탈출구가 아니다.

  둘은 처방이 완전히 다르므로 재고 나서 결정한다. 그리고 이번에도 **처방을 먼저 만들지
  않는다** — 이 프로젝트에서 "그럴듯한 기전 -> 즉시 처방"은 여섯 번 실패했다.

사용:
    python _phase23_probe.py --target AIP_ryujan_v42.dll --num-seeds 20
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

from _aim_time_probe import DT, PHASES, make_state, score_rate  # noqa: E402
from DogFightEnvWrapper import DogFightWrapper  # noqa: E402
from dogfight.ai.bt_action_provider import BTActionProvider  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ownship", default="AIP_DCS.dll")
    ap.add_argument("--target", required=True)
    ap.add_argument("--num-seeds", type=int, default=20)
    ap.add_argument("--seed-offset", type=int, default=0)
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
        md = {1: 0.0, 2: 0.0, 3: 0.0}      # 내 Phase별 피해
        td = {1: 0.0, 2: 0.0, 3: 0.0}
        mt = {1: 0.0, 2: 0.0, 3: 0.0}      # 내 Phase별 조건충족 시간
        tt = {1: 0.0, 2: 0.0, 3: 0.0}
        # 후반 구간 sub-각도 체류 (게이트 무관하게 순수 기하만)
        m2 = m3 = t2 = t3 = 0.0
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
            r, ph = score_rate(d, ma, t_s)
            if ph:
                md[ph] += r * DT
                mt[ph] += DT
            r, ph = score_rate(d, ta, t_s)
            if ph:
                td[ph] += r * DT
                tt[ph] += DT
            if t_s >= 100.0 and 152.4 <= d <= 1066.8:
                m2 += DT if ma < 2.0 else 0.0
                t2 += DT if ta < 2.0 else 0.0
            if t_s >= 150.0 and 152.4 <= d <= 1219.2:
                m3 += DT if ma < 3.0 else 0.0
                t3 += DT if ta < 3.0 else 0.0
        env.close()
        mine, them = sum(md.values()), sum(td.values())
        res = "승" if mine > them else ("패" if them > mine else "무")
        rows.append(dict(seed=seed, res=res, mine=mine, them=them, margin=mine - them,
                         md=md, td=td, mt=mt, tt=tt, m2=m2, t2=t2, m3=m3, t3=t3,
                         secs=step * DT))
        print(f"  seed {seed:>2} {res}  내 {mine:.5f} (P1 {md[1]:.4f} P2 {md[2]:.4f} "
              f"P3 {md[3]:.4f})  상대 {them:.5f} (P1 {td[1]:.4f} P2 {td[2]:.4f} "
              f"P3 {td[3]:.4f})", flush=True)

    n = len(rows)
    W = [r for r in rows if r["res"] == "승"]
    L = [r for r in rows if r["res"] == "패"]
    D = [r for r in rows if r["res"] == "무"]
    print("\n" + "=" * 100)
    print(f"[Phase 2/3 분해] {n}시드(오프셋 {args.seed_offset})  상대={args.target}  "
          f"나={args.ownship}")
    print(f"  전적 {len(W)}승 {len(L)}패 {len(D)}무   승점 "
          f"{len(W) + 0.5 * len(D):.1f}/{n}")

    print(f"\n  Phase별 피해 합계 (판당 평균)")
    print(f"  {'':<6} {'P1':>10} {'P2':>10} {'P3':>10} {'합':>10}")
    for nm, k in (("나", "md"), ("상대", "td")):
        v = [sum(r[k][p] for r in rows) / n for p in (1, 2, 3)]
        print(f"  {nm:<6} {v[0]:>10.5f} {v[1]:>10.5f} {v[2]:>10.5f} {sum(v):>10.5f}")
    print(f"\n  Phase별 조건충족 시간 s/판")
    print(f"  {'':<6} {'P1':>10} {'P2':>10} {'P3':>10}")
    for nm, k in (("나", "mt"), ("상대", "tt")):
        v = [sum(r[k][p] for r in rows) / n for p in (1, 2, 3)]
        print(f"  {nm:<6} {v[0]:>10.3f} {v[1]:>10.3f} {v[2]:>10.3f}")

    print(f"\n  후반 순수 기하 체류 s/판 (게이트 시각 이후, 사거리 안)")
    print(f"  {'':<24} {'나':>9} {'상대':>9} {'비(상대/나)':>12}")
    for nm, a, b in (("t>=100s  ATA<2도", "m2", "t2"), ("t>=150s  ATA<3도", "m3", "t3")):
        A = sum(r[a] for r in rows) / n
        B = sum(r[b] for r in rows) / n
        rt = (B / A) if A > 1e-9 else float("inf")
        print(f"  {nm:<24} {A:>9.3f} {B:>9.3f} "
              f"{('inf' if rt == float('inf') else f'{rt:.2f}'):>12}")

    if L:
        mg = np.array([-r["margin"] for r in L])   # 패배 마진(양수)
        print(f"\n  ▶ 패배 {len(L)}판의 마진(상대피해 - 내피해)")
        print(f"    중앙 {np.median(mg):.5f}   최소 {mg.min():.5f}   최대 {mg.max():.5f}")
        pl = {p: sum(r["td"][p] for r in L) / len(L) for p in (1, 2, 3)}
        ml = {p: sum(r["md"][p] for r in L) / len(L) for p in (1, 2, 3)}
        print(f"    그 판에서 상대 피해 출처  P1 {pl[1]:.5f}  P2 {pl[2]:.5f}  P3 {pl[3]:.5f}")
        print(f"                내 피해      P1 {ml[1]:.5f}  P2 {ml[2]:.5f}  P3 {ml[3]:.5f}")
        A = sum(r["m2"] + r["m3"] for r in L) / len(L)
        B = sum(r["t2"] + r["t3"] for r in L) / len(L)
        print(f"    후반 sub각 체류 합  나 {A:.3f}s  상대 {B:.3f}s  "
              f"비 {('inf' if A < 1e-9 else f'{B/A:.2f}')}")

        print(f"\n  ➜ 사전 등록 판정")
        tight = np.median(mg) < 0.01
        near = (A > 1e-9) and (B / A < 2.0)
        if tight and near:
            print(f"     **뒤집을 수 있는 동전던지기다.** 마진 중앙 {np.median(mg):.5f} < 0.01 이고")
            print(f"     후반 체류가 {B/A:.2f}배로 배수 차이가 아니다. 값싼 변경으로 움직인다.")
        elif not tight:
            print(f"     마진 중앙 {np.median(mg):.5f} >= 0.01 — 미세 차이가 아니다.")
            print(f"     후반에도 그들이 실질적으로 앞선다. 133도 기하 문제와 같은 벽이다.")
        else:
            print(f"     마진은 작으나 후반 체류가 {B/A:.2f}배다 — 그들이 뒤에 눌러앉아 있다.")
            print(f"     Phase 2/3은 탈출구가 아니다. 기하를 바꿔야 한다.")
    else:
        print("\n  패배 없음 — 분해할 대상이 없다")
    print("=" * 100)


if __name__ == "__main__":
    main()
