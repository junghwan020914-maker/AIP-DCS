"""**교차(머지) 순간**의 기하를 잰다 — TwoCircle 채택 후 남은 유일한 득점 기회.

08-14 배경. HABFM에서 OneCircle 분기를 제거해(항상 TwoCircle) v42전 패배가 6->3으로
줄었다(커밋 87bcf7f). 그러나 **20판 중 19무**로 이기지는 못한다.

TwoCircle은 반대 방향으로 돌아 **매 바퀴 정면에서 교차**한다. 각도가 리셋되므로 지는
싸움을 반복하지 않지만, 뒤집어 말하면 **그 교차 순간이 유일한 득점 기회**다.
지금은 그 순간에 양쪽 다 못 쏜다.

그리고 눈금은 이미 있다 — **1.78초의 Phase1 조준이 격추**다(08-13 실서버 검증).
19무를 승으로 바꾸는 데 필요한 건 판당 2초짜리 창 하나다.

재는 것 — 각 교차(국소 최근접점)마다:
  · 시각, 최근접 거리, 그때 양쪽 ATA
  · 교차 전후 5초의 ATA 최솟값 (창이 얼마나 열렸나)
  · 밴드(152~914m) 안에서 일어난 교차인가
  · 교차 간격(주기)과 판당 횟수
  · **누가 반 박자 유리한가** — 최근접 시점의 ATA 차

⚠️ 이번에도 **처방을 먼저 만들지 않는다.** 교차가 몇 번 일어나고 그때 우리가 얼마나
   못 미치는지부터 확정한다. 필요한 개선폭이 0.5도인지 20도인지에 따라 처방이 전혀
   달라진다(0.5도면 조준 정밀도, 20도면 기하 자체를 바꿔야 한다).

## 🔴 08-14 첫 측정 — **"교차" 프레임 자체가 틀렸다**

v42 20시드, 교차 181회(판당 9.0회):

    교차 시점 ATA        우리 141.6도   상대  32.9도
    전후 5초 최솟값      우리 122.9도   상대  12.8도

    밴드 안 교차 101회(56%), 최근접 중앙 790m
      ATA 최솟값  우리 중앙 **133.72도**  p10 21.83  최소 1.51
                  상대 중앙 ** 12.37도**  p10  6.77  최소 0.23
      1도 안:  우리 0.0%   상대 2.0%     5도 안: 우리 3.0%  상대 8.9%

**이건 정면 교차가 아니다.** 최근접 순간마다 **v42가 우리 뒤에 있다.**
"TwoCircle이 매 바퀴 정면 교차를 만들어 각도를 리셋한다"는 서술은 사실이 아니었다.
국소 최근접점은 **그들의 사격 패스**다.

이걸로 19무가 설명된다 — **그들은 뒤를 잡지만 마무리를 못 하고**(12.37도, 1도 안은 2%)
**우리는 근처도 못 간다**(133.72도).

➡️ **필요한 개선폭 133도 = 기하 문제다.** 조준 정밀도로는 못 고친다.
   판정 기준(5도 미만이면 조준, 수십도면 기하)을 미리 박아둔 것이 이번에 작동했다.
   안 그랬으면 "최소 1.51도까지 갔으니 조금만 더"라는 이야기를 만들었을 것이다.

⚠️ 부수 확인 — **TwoCircle 채택이 효과가 있었던 이유도 이걸로 설명된다.**
   반대로 돌면 그들이 뒤에 눌러앉지 못하고 **스쳐 지나간다.** 그래서 패배가 6->3으로
   줄었다. **공세를 만들어준 게 아니라 그들의 전환을 방해한 것**이다.

사용:
    python _cross_probe.py --target AIP_ryujan_v42.dll --num-seeds 20
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
WIN = int(round(5.0 / DT))          # 교차 전후 창(5초)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ownship", default="AIP_DCS.dll")
    ap.add_argument("--target", required=True)
    ap.add_argument("--num-seeds", type=int, default=20)
    ap.add_argument("--seed-offset", type=int, default=0)
    args = ap.parse_args()

    ev = []
    per_seed = []
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
        D, MA, TA = [], [], []
        terminated = truncated = False
        while not (terminated or truncated):
            _, _, terminated, truncated, _ = env.step(np.zeros(4, dtype=np.float32))
            g, o, t = env._geo_info, env._ownship_state, env._target_state
            D.append(float(g._get_distance(o, t)))
            MA.append(abs(float(g._get_antenna_train_angle(o, t, False))))
            TA.append(abs(float(g._get_antenna_train_angle(t, o, False))))
        env.close()
        d = np.array(D); ma = np.array(MA); ta = np.array(TA)
        # 국소 최소(교차) 검출 — 앞뒤 WIN 안에서 가장 가까운 지점
        n = len(d)
        idx = []
        i = WIN
        while i < n - WIN:
            w = d[i - WIN:i + WIN + 1]
            if d[i] == w.min() and d[i] < 3000.0:
                idx.append(i)
                i += WIN          # 중복 계상 방지
            else:
                i += 1
        cnt = 0
        for i in idx:
            lo, hi = max(0, i - WIN), min(n, i + WIN + 1)
            ev.append(dict(seed=seed, t=i * DT, d=d[i], ma=ma[i], ta=ta[i],
                           ma_min=float(ma[lo:hi].min()), ta_min=float(ta[lo:hi].min()),
                           inband=(BAND_MIN <= d[i] <= BAND_MAX)))
            cnt += 1
        per_seed.append(cnt)
        print(f"  seed {seed:>2} 교차 {cnt}회", flush=True)

    if not ev:
        print("교차 없음")
        return
    f = lambda k: np.median([e[k] for e in ev])
    inb = [e for e in ev if e["inband"]]
    print("\n" + "=" * 96)
    print(f"[교차 기하] {args.num_seeds}시드  상대={args.target}  교차 {len(ev)}회 "
          f"(판당 {np.mean(per_seed):.1f}회)")
    print(f"  최근접 거리 중앙 {f('d'):6.0f}m   밴드(152~914m) 안 교차 "
          f"{len(inb)}회 ({100*len(inb)/len(ev):.0f}%)")
    print(f"  교차 시점 ATA  우리 {f('ma'):6.1f}도   상대 {f('ta'):6.1f}도")
    print(f"  전후 5초 최솟값 우리 {f('ma_min'):6.1f}도   상대 {f('ta_min'):6.1f}도")

    if inb:
        g = lambda k: np.median([e[k] for e in inb])
        a = np.array([e["ma_min"] for e in inb])
        b = np.array([e["ta_min"] for e in inb])
        print(f"\n  ▶ 밴드 안 교차 {len(inb)}회만 — 여기가 유일한 득점 기회다")
        print(f"    최근접 거리 중앙 {g('d'):6.0f}m")
        print(f"    전후 5초 ATA 최솟값  우리 중앙 {np.median(a):5.2f}도  "
              f"p10 {np.percentile(a,10):5.2f}도  최소 {a.min():5.2f}도")
        print(f"                        상대 중앙 {np.median(b):5.2f}도  "
              f"p10 {np.percentile(b,10):5.2f}도  최소 {b.min():5.2f}도")
        print(f"    1도 안에 든 교차   우리 {100*np.mean(a<1):5.1f}%   상대 {100*np.mean(b<1):5.1f}%")
        print(f"    2도 안            우리 {100*np.mean(a<2):5.1f}%   상대 {100*np.mean(b<2):5.1f}%")
        print(f"    5도 안            우리 {100*np.mean(a<5):5.1f}%   상대 {100*np.mean(b<5):5.1f}%")
        print(f"\n    ➜ **필요한 개선폭 = 우리 중앙 {np.median(a):.2f}도 -> 1.0도**")
        print(f"       {'조준 정밀도 문제(수도 이내)' if np.median(a) < 5 else '기하 문제(수십도 부족)'}")
    print("=" * 96)


if __name__ == "__main__":
    main()
