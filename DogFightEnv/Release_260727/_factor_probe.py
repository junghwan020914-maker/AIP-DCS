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
    예상HP      실서버 HP 환산

✅ 08-13 확정: 배율은 **1**이다(단위 차이였다). 로컬은 체력을 0~1로 정규화해 쓰고
   실서버는 같은 값을 HP 0~100으로 표시한다. 즉 **로컬 피해량 1.0 = HP 100 = 격추**.
   (한때 "프레임당 누산 60~75배"로 적었으나 **틀렸다. 철회했다.**)
⚠️ 다만 여기 `내득점`은 Phase 1+2+3 합이다. **뷰어는 Phase 1만 준다**(실서버 4판 확인).
   Phase 1 기준 판정은 `_suite_eval.py`의 [뷰어 실측 모델] 표를 볼 것.

## 📊 08-13 첫 전수 측정 결과 (12상대 x 10시드, AIP_DCS.dll 기준)

    상대     밴드체류  밴드내조준률  득점시간  헛조준     성적
    arcE       4.6s      93.33%     4.31s   49.8s    20/20 격추
    v29       19.8       14.62%     2.89    11.7     19승0패
    v32       18.4       12.75%     2.34    11.4     17승0패
    v7        40.2        6.74%     2.71     0.6     20승0패
    arcA      17.3        6.52%     1.13     3.5     20/20 (14 격추)
    arcD      29.7        6.41%     1.91     8.2     순수 방어형
    v0        26.5        4.44%     1.18    15.0     16승0패
    prev      24.2        1.42%     0.34     0.2     6승12패 (7번 피격추)
    v42       52.2        0.19%     0.10     0.0     3승8패
    arcV      24.2        0.00%     0.00     2.0     교착
    v5        25.1        0.00%     0.00     0.0     6승6패

**밴드 체류는 문제가 아니다.** 거의 모든 상대에게 17~52초를 사거리 안에서 보낸다
(arcE만 4.6s로 예외). **조준률이 성적과 단조 대응한다** — 93%(격추) -> 14%(압승) ->
6%(승) -> 1.4%(6승12패) -> 0.19%(3승8패) -> 0%(못 이김).

**v42가 결정적이다. 밴드 52.2초로 전 상대 중 가장 오래 있는데 조준률 0.19%다.**
못 붙어서 못 이기는 게 전혀 아니다. 가장 잘 붙는데 가장 못 겨눈다.
(내가 arcE 실서버 판 하나를 보고 "문제는 조준이 아니라 거리"라고 한 것은 틀렸다.
 arcE가 유일한 예외였다.)

⚠️ **교란 요인 — 반드시 같이 읽을 것.** 조준률이 낮은 게 "못 겨눠서"인지 "겨눌 위치가
아니라서"인지 이 지표만으로는 안 갈린다. v42전 우리 공세 시간은 0.76%뿐이라 밴드
52.2초의 대부분은 **우리가 쫓기는 중**일 수 있다.

➡️ **그래서 `arcD`가 결정적 사례다.** arcD는 절대 공격하지 않는 순수 방어형이라
   밴드 29.7초가 **전부 우리가 뒤에 있는 시간**이다. 교란이 없다. 그 조건에서
   조준률 **6.41%**, 득점 1.91초다. 위치도 거리도 다 갖춰준 상태에서 못 겨눈다 =
   **순수한 추적 실패**다. 다음 개선의 벤치는 arcD로 잡을 것.

TODO: 공세 조건(내ATA<50 & 적ATA>130)을 추가해 교란을 제거한 순수 조준률을 재측정할 것.

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
MULT = 100.0   # 정규화 1.0 -> HP 100


FRONT, REAR = 50.0, 130.0   # 공세 판정 임계 (`_stalemate_diag.py`와 동일)


def run(ownship, target, n, off):
    band = hit = waste = 0
    oband = ohit = 0            # 공세 중 밴드 / 그 안에서 조준 성공
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
            # 공세 = 내가 상대 뒤에 있다. 이 조건을 걸어야 "못 겨눠서"와
            # "겨눌 위치가 아니라서"가 갈린다(v42는 밴드 52.2s인데 공세는 0.76%뿐이었다).
            off_now = (ma < FRONT) and (ta > REAR)
            if inb:
                band += 1
                if off_now:
                    oband += 1
                    if ma <= 1.0:
                        ohit += 1
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
                T_oband=s(oband),
                P_oaim=(100.0 * ohit / oband if oband else 0.0),
                T_ohit=s(ohit),
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
        print(f"  {t:<20} 밴드{r['T_band']:6.1f}s 조준률{r['P_aim']:5.2f}% | "
              f"공세밴드{r['T_oband']:6.1f}s **순수조준률{r['P_oaim']:6.2f}%** | "
              f"득점{r['T_hit']:5.2f}s 헛조준{r['T_waste']:6.1f}s", flush=True)

    print("\n" + "=" * 112)
    print(f"[득점 요인 분해] {args.num_seeds}시드/상대  나={args.ownship}   "
          f"HP 환산(정규화 1.0 = HP 100)")
    print(f"  {'상대':<20} {'밴드체류':>8} {'조준률':>7} | {'공세밴드':>8} "
          f"{'순수조준률':>10} {'공세득점':>8} | {'헛조준':>7} | {'내득점':>8} {'예상HP':>7} | {'피격':>8}")
    for r in rows:
        print(f"  {r['target']:<20} {r['T_band']:>7.1f}s {r['P_aim']:>6.2f}% | "
              f"{r['T_oband']:>7.1f}s {r['P_oaim']:>9.2f}% {r['T_ohit']:>7.2f}s | "
              f"{r['T_waste']:>6.1f}s | {r['my']:>8.4f} {r['my']*MULT:>7.1f} | {r['th']:>8.4f}")

    print("\n  ➜ 병목 판별")
    print(f"  {'상대':<20} {'병목':<28} 근거")
    # 🔴 08-14 교훈: v5를 "조준(못 겨눈다)"으로 **오분류했다.** 실제로는 밴드 21.2s 중
    # 공세가 2.1s뿐이라 **겨눌 위치 자체가 없는** 경우였다(같은 계보 v7 22.9s / v0 40.1s와
    # 대조하면 11~19분의 1이다). 밴드체류만 보면 '위치 열세'가 '조준 실패'로 잘못 읽힌다.
    # **공세밴드를 가장 먼저 본다.**
    for r in rows:
        if r["T_oband"] < 3.0:
            b, why = "위치(공세를 못 만든다)", (
                f"밴드 {r['T_band']:.1f}s인데 공세는 {r['T_oband']:.1f}s뿐 — 조준률은 무의미")
        elif r["P_oaim"] < 1.0:
            b, why = "조준(못 겨눈다)", (
                f"공세밴드 {r['T_oband']:.1f}s인데 순수조준률 {r['P_oaim']:.2f}%")
        elif r["T_band"] < 10.0 and r["P_aim"] > 5.0:
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
