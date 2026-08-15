"""**양쪽 다** 잰다 — 우리가 뒤를 잡는 시간 대 그들이 뒤를 잡는 시간.

08-15 배경. 저득점 상대 넷이 각기 다른 경로로 같은 결론에 도달했다:

    arcV  요구 선회율 35.5도/s vs 가능 16.6도/s   -> 수직으로 못 따라간다
    v42   교차마다 그들이 우리 뒤, 필요 개선폭 133도 -> 기하가 뒤집혀 있다
    v5    판당 공세밴드 6.3초 (v7은 22.8초)        -> 기회 자체가 1/4
    prev  격추당한 판은 초반 각도가 갈렸다          -> 홀드아웃 전이 실패

넷을 "우리가 뒤를 못 잡는다"로 묶었다. **그런데 그건 내가 사후에 묶은 것이다.**
서로 다른 세 지표를 하나의 서사로 꿴 것이므로, 이 프로젝트에서 여섯 번 실패한
"그럴듯한 기전" 패턴과 형태가 같다. 그래서 **한 지표로, 전 상대에게, 한꺼번에** 잰다.

재는 것 — 매치업마다 네 열. 공세 = 내ATA<50도 & 적ATA>130도 (내가 상대 뒤):

    T_off_me    우리가 공세+밴드인 시간 s/판      <- 우리가 뒤를 잡는가
    P_aim_me    그 안에서 ATA<=1도인 비율          <- 잡은 걸 조준으로 바꾸는가
    T_off_th    그들이 공세+밴드인 시간 s/판      <- 그들이 뒤를 잡는가
    P_aim_th    그 안에서 적ATA<=1도인 비율        <- 그들은 바꾸는가

⚠️ **판정 규칙을 데이터 보기 전에 박는다.** 상대가 11이고 열이 4라 사후에 아무 이야기나
   만들 수 있다. 그래서 기계적으로 정한다:

   승리군 = 20시드에서 만점(20.0)인 상대  [arcA arcE arcD v7]
   저득점군 = 12.0 미만                    [prev 7.0, v5 8.0, arcV 10.0, v42 10.5]

   네 열 각각에 대해 **분리도 = |군평균 차| / 두 군 표준편차 합**을 계산하고,
   **분리도가 가장 큰 열 하나**를 병목으로 지목한다. 눈으로 고르지 않는다.

   · T_off_me가 1등이면 **위치** 문제다 — 처방은 뒤를 잡는 기동이다
   · P_aim_me가 1등이면 **전환** 문제다 — 조준 처방이 여섯 번 기각된 게 이상해진다
   · T_off_th가 1등이면 **수세** 문제다 — 우리가 공세를 못 만드는 게 아니라
     그들에게 잡히는 게 문제이고, 처방은 방어 거부다(TwoCircle 채택이 그렇게 작동했다)
   · P_aim_th가 1등이면 상대의 마무리력 차이이고 **우리가 고칠 수 있는 게 아니다**

⚠️ v29/v32/v0는 중간군(15.5~18.5)이라 어느 군에도 넣지 않는다. 표에는 찍되 판정에서 뺀다.
   사후에 경계를 옮겨 원하는 답을 만드는 걸 막기 위해서다.

사용:
    python _symmetry_probe.py --num-seeds 12 --seed-offset 30
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

BAND_MIN, BAND_MAX = 152.4, 914.4
FRONT, REAR = 50.0, 130.0

DEFAULT_TARGETS = ("AIP_arcA.dll,AIP_arcE.dll,AIP_arcD.dll,AIP_v7.dll,"
                   "AIP_v29.dll,AIP_v32.dll,AIP_v0.dll,"
                   "AIP_ryujan_v42.dll,AIP_arcV.dll,AIP_v5.dll,AIP_prev.dll")
WIN_GROUP = {"AIP_arcA.dll", "AIP_arcE.dll", "AIP_arcD.dll", "AIP_v7.dll"}
LOW_GROUP = {"AIP_prev.dll", "AIP_v5.dll", "AIP_arcV.dll", "AIP_ryujan_v42.dll"}


def run(ownship, target, n, off):
    off_me = aim_me = off_th = aim_th = 0
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
        while not (terminated or truncated):
            _, _, terminated, truncated, _ = env.step(np.zeros(4, dtype=np.float32))
            g, o, t = env._geo_info, env._ownship_state, env._target_state
            d = float(g._get_distance(o, t))
            if not (BAND_MIN <= d <= BAND_MAX):
                continue
            ma = abs(float(g._get_antenna_train_angle(o, t, False)))
            ta = abs(float(g._get_antenna_train_angle(t, o, False)))
            if ma < FRONT and ta > REAR:          # 우리가 그들 뒤
                off_me += 1
                if ma <= 1.0:
                    aim_me += 1
            if ta < FRONT and ma > REAR:          # 그들이 우리 뒤
                off_th += 1
                if ta <= 1.0:
                    aim_th += 1
        env.close()
    return dict(
        target=target,
        T_off_me=off_me * DT / n,
        P_aim_me=(100.0 * aim_me / off_me if off_me else 0.0),
        T_off_th=off_th * DT / n,
        P_aim_th=(100.0 * aim_th / off_th if off_th else 0.0),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ownship", default="AIP_DCS.dll")
    ap.add_argument("--targets", default=DEFAULT_TARGETS)
    ap.add_argument("--num-seeds", type=int, default=12)
    ap.add_argument("--seed-offset", type=int, default=0)
    args = ap.parse_args()

    rows = []
    for t in [x.strip() for x in args.targets.split(",") if x.strip()]:
        if not (ROOT / t).exists():
            print(f"  (없음) {t}", flush=True)
            continue
        r = run(args.ownship, t, args.num_seeds, args.seed_offset)
        rows.append(r)
        grp = "승" if t in WIN_GROUP else ("저" if t in LOW_GROUP else "중")
        print(f"  [{grp}] {t:<20} 우리공세{r['T_off_me']:6.1f}s "
              f"조준{r['P_aim_me']:5.2f}% | 그들공세{r['T_off_th']:6.1f}s "
              f"조준{r['P_aim_th']:5.2f}%", flush=True)

    cols = ["T_off_me", "P_aim_me", "T_off_th", "P_aim_th"]
    label = {"T_off_me": "우리 공세시간", "P_aim_me": "우리 전환률",
             "T_off_th": "그들 공세시간", "P_aim_th": "그들 전환률"}

    print("\n" + "=" * 104)
    print(f"[공세 대칭성] {args.num_seeds}시드(오프셋 {args.seed_offset})  나={args.ownship}")
    print(f"  {'군':<3} {'상대':<20} {'우리공세s':>9} {'우리전환%':>9} "
          f"{'그들공세s':>9} {'그들전환%':>9} | {'공세비(우리/그들)':>16}")
    for r in rows:
        t = r["target"]
        grp = "승" if t in WIN_GROUP else ("저" if t in LOW_GROUP else "중")
        ratio = (r["T_off_me"] / r["T_off_th"]) if r["T_off_th"] > 0.05 else float("inf")
        rs = "  inf" if ratio == float("inf") else f"{ratio:6.2f}"
        print(f"  {grp:<3} {t:<20} {r['T_off_me']:>9.1f} {r['P_aim_me']:>9.2f} "
              f"{r['T_off_th']:>9.1f} {r['P_aim_th']:>9.2f} | {rs:>16}")

    W = [r for r in rows if r["target"] in WIN_GROUP]
    L = [r for r in rows if r["target"] in LOW_GROUP]
    if len(W) < 2 or len(L) < 2:
        print("\n  판정 불가 — 군 표본 부족")
        print("=" * 104)
        return

    print(f"\n  ▶ 사전 등록 판정: 분리도 = |군평균 차| / (승군 표준편차 + 저군 표준편차)")
    print(f"  {'열':<14} {'승군 평균':>10} {'저군 평균':>10} {'분리도':>8}")
    sep = {}
    for c in cols:
        a = np.array([r[c] for r in W], dtype=float)
        b = np.array([r[c] for r in L], dtype=float)
        denom = a.std(ddof=1) + b.std(ddof=1)
        sep[c] = abs(a.mean() - b.mean()) / denom if denom > 1e-9 else float("inf")
        print(f"  {label[c]:<14} {a.mean():>10.2f} {b.mean():>10.2f} {sep[c]:>8.2f}")

    top = max(sep, key=sep.get)
    order = sorted(sep, key=sep.get, reverse=True)
    print(f"\n  ➜ 병목 = **{label[top]}** (분리도 {sep[top]:.2f}, "
          f"2위 {label[order[1]]} {sep[order[1]]:.2f})")
    verdict = {
        "T_off_me": "**위치** 문제 — 처방은 뒤를 잡는 기동이다",
        "P_aim_me": "**전환** 문제 — 조준 처방이 여섯 번 기각된 게 이상해진다. 재검토할 것",
        "T_off_th": "**수세** 문제 — 공세를 못 만드는 게 아니라 잡히는 게 문제다. "
                    "처방은 방어 거부(TwoCircle 채택이 그렇게 작동했다)",
        "P_aim_th": "상대의 마무리력 차이 — **우리가 고칠 수 있는 게 아니다**",
    }[top]
    print(f"     {verdict}")
    if sep[top] - sep[order[1]] < 0.3:
        print(f"     ⚠️ 1·2위 차가 {sep[top]-sep[order[1]]:.2f}로 작다 — 단독 지목은 근거가 약하다")
    print("=" * 104)


if __name__ == "__main__":
    main()
