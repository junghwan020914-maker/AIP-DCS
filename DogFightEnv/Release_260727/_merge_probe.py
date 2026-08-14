"""초기 머지에서 **각도가 언제 갈리는지** 시계열로 잰다 — 남은 마지막 레버.

08-14 배경. `_death_probe.py`가 승패를 가르는 유일한 열을 찾았다. 격추당한 7판과
생존 13판의 초반 40초를 비교하면:

    군        n | 고도차  속도차   거리   내ATA   적ATA
    격추당함  7 |   -2   +0.3   1885  **111.0**  **55.7**
    생존     13 |   +2   -0.0   1875  ** 68.6**  ** 97.1**

**고도차·속도차·거리는 완전히 겹치고 각도만 갈린다.** 에너지 상태가 동일한데
위치만 다르다. 두 기체는 동일 성능 F-16이고 어빔에서 대칭으로 출발하는데도 그렇다.

`Functions.cpp`에 이미 적혀 있던 결론과 만난다 — 남은 레버는 (1) 수세->공세 반전
(2) **초기 머지 첫 선회** 둘뿐이고, (1)은 LastDitch와 방어 전추력으로 두 번 기각됐다.
(2)는 아직 한 번도 시도하지 않았다.

⚠️ **이번엔 처방을 먼저 만들지 않는다.** 오늘 기각된 둘 다 "그럴듯한 기전 -> 즉시 처방"
   순서였고, 상관을 정책으로 바꾸면 그 상관이 사라진다는 걸 또 확인했다(5전 5패).
   여기서는 **갈리는 순간이 언제인지만** 확정한다.

재는 것 — 초반 60초를 1초 간격으로, 승/패 군별 집계:
  · ATA 차(적ATA - 내ATA). 양수면 우리가 유리하다. **이게 언제 갈리는가.**
  · 각 기체의 선회율(기수 방향의 회전 각속도)과 그 방향(좌/우)
  · 두 기체가 같은 방향으로 도는가(동선회)
  · 상대 기준 우리 위치(상대의 좌/우 어느 쪽에 있나)

## ❌ 08-14 첫 측정 결과 — **판별력 없음. 원인 미확립으로 종결.**

v42 20시드(승2/무12/패6). ATA 우위(적ATA - 내ATA, 양수=우리 유리) 중앙값:

     t(s) |  승(n=2)  무(n=12)  패(n=6)
       15 |    -2.2     +3.1     **+10.5**
       20 |   -62.8    -28.9     -12.8
       30 |  -131.9    -55.0     -24.8
       60 |   -88.2   -123.0     -95.7

**패배군이 오히려 초반 각도 우위다**(t=15에 +10.5 vs 무승부 +3.1). 승리군은 n=2라
해석 불가. 즉 **v42 상대로는 초반 각도가 승패를 예측하지 못하고 방향이 오히려 반대다.**

`_death_probe.py`가 prev에서 찾은 "초반 각도가 갈린다"(격추당함 내ATA 111.0 / 적ATA 55.7
vs 생존 68.6 / 97.1)는 **v42로 전이되지 않는다.** 그 발견은 prev 매치업 고유이고
시드 0~29 전용이었다(홀드아웃에서는 같은 매치업이 6.5 -> 15.5로 뒤집힌다).

부수 확인: 선회율이 우리 11~18도/s, 상대 11~19도/s로 거의 같다 — **성능 대칭**이
다시 확인된다. 그리고 t=20 이후 모든 군에서 ATA 우위가 깊게 음수다.
**v42는 어느 판에서든 우리에게 각도를 잡고, 승패는 그 뒤 다른 무언가로 갈린다.**

➡️ `Functions.cpp`가 남긴 두 레버 중 (2) "초기 머지 첫 선회"도 이로써 **근거가 없다.**
   (1) "수세->공세 반전"은 LastDitch·방어 전추력으로 두 번 기각됐다.
   **두 레버 모두 소진됐다.** 다음 탐색은 이 프레임 밖에서 시작해야 한다.

사용:
    python _merge_probe.py --target AIP_prev.dll --num-seeds 20
    python _merge_probe.py --target AIP_prev.dll --num-seeds 20 --seed-offset 30
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

WINDOW = 60.0
STRIDE = int(round(1.0 / DT))          # 1초 간격


def wrap180(x):
    return (x + 180.0) % 360.0 - 180.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ownship", default="AIP_DCS.dll")
    ap.add_argument("--target", required=True)
    ap.add_argument("--num-seeds", type=int, default=20)
    ap.add_argument("--seed-offset", type=int, default=0)
    args = ap.parse_args()

    n_pts = int(WINDOW) + 1
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
        mine = them = 0.0
        adv = np.full(n_pts, np.nan)      # 적ATA - 내ATA (양수 = 우리 유리)
        myrate = np.full(n_pts, np.nan)   # 내 선회율(부호 = 좌/우)
        thrate = np.full(n_pts, np.nan)
        ph = th = None
        while not (terminated or truncated):
            _, _, terminated, truncated, _ = env.step(np.zeros(4, dtype=np.float32))
            step += 1
            t_s = step * DT
            g, o, t = env._geo_info, env._ownship_state, env._target_state
            d = float(g._get_distance(o, t))
            ma = abs(float(g._get_antenna_train_angle(o, t, False)))
            ta = abs(float(g._get_antenna_train_angle(t, o, False)))
            mine += score_rate(d, ma, t_s)[0] * DT
            them += score_rate(d, ta, t_s)[0] * DT
            h, hh = float(o[5]), float(t[5])
            if step % STRIDE == 0:
                i = step // STRIDE
                if i < n_pts:
                    adv[i] = ta - ma
                    if ph is not None:
                        myrate[i] = wrap180(h - ph) / (STRIDE * DT)
                        thrate[i] = wrap180(hh - th) / (STRIDE * DT)
                    ph, th = h, hh
        env.close()
        res = "패" if them > mine else ("승" if mine > them else "무")
        rows.append((res, adv, myrate, thrate))
        print(f"  seed {seed:>2} {res}  내{mine:.4f} 적{them:.4f}", flush=True)

    print("\n" + "=" * 96)
    print(f"[초기 머지 시계열] {args.num_seeds}시드(오프셋 {args.seed_offset})  "
          f"상대={args.target}")
    groups = [(nm, [r for r in rows if r[0] == nm]) for nm in ("승", "무", "패")]
    print(f"  ATA 우위(적ATA - 내ATA, 양수=우리 유리) 중앙값")
    print(f"  {'t(s)':>5} | " + " ".join(f"{nm}(n={len(rs):>2})" for nm, rs in groups))
    for t in range(0, int(WINDOW) + 1, 5):
        cells = []
        for nm, rs in groups:
            if not rs:
                cells.append("     -   ")
                continue
            v = [r[1][t] for r in rs if np.isfinite(r[1][t])]
            cells.append(f"{np.median(v):+8.1f} " if v else "     -   ")
        print(f"  {t:>5} | " + " ".join(cells))

    print(f"\n  선회율 크기(도/s) 중앙값 — 우리 / 상대")
    print(f"  {'t(s)':>5} | " + " ".join(f"{nm:>13}" for nm, _ in groups))
    for t in range(1, 21, 2):
        cells = []
        for nm, rs in groups:
            if not rs:
                cells.append(f"{'-':>13}")
                continue
            a = [abs(r[2][t]) for r in rs if np.isfinite(r[2][t])]
            b = [abs(r[3][t]) for r in rs if np.isfinite(r[3][t])]
            cells.append(f"{np.median(a):5.1f}/{np.median(b):<7.1f}" if a and b
                         else f"{'-':>13}")
        print(f"  {t:>5} | " + " ".join(cells))

    print("\n  ➜ 판정: 승/패 군의 ATA 우위 곡선이 **몇 초에 갈리기 시작하는가**.")
    print("     그 시각 이전이면 초기 선택의 문제, 이후면 누적된 기동의 문제다.")
    print("=" * 96)


if __name__ == "__main__":
    main()
