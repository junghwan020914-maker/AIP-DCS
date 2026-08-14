"""우리가 **맞는 순간**을 해부한다 — 언제, 어느 기하에서, 얼마나 오래.

08-13 배경. 채점기를 Phase 1 기준으로 고치자 이전에 안 보이던 것이 나왔다:
**prev 상대 20판에서 우리가 7번 격추당한다.** 그전 척도로는 피격이 늘 0.0000 근처라
방어를 볼 이유가 없어 보였다. `prev`는 우리 직전 세대이고 현재 최저값(6.5/20)이다.

⚠️ **해석 주의 — 두 가지를 섞지 말 것.**
  (a) prev가 일반적으로 잘 죽이는 기체다  -> 우리 방어의 문제가 아닐 수 있다
  (b) prev만 우리를 죽인다                 -> 우리 혈통이 공유하는 맹점을 찌르는 것
  `_suite_eval.py` 주석의 경고 그대로다 — **헤드투헤드는 비이행적**이고 prev는 같은
  혈통이라 같은 맹점을 공유한다. 그래서 이 도구와 별개로 **prev를 ownship으로 돌려
  다른 상대도 죽이는지** 대조해야 판정이 선다.

재는 것 (Phase 1만: ATA<1도, 152.4~914.4m — 뷰어 실측 모델):
  · 피격 이벤트: 시각, 거리, 내ATA, 적ATA, 내속도, 속도차, 고도차, 연속 지속시간
  · 시드별: 총 피격량, 격추 여부와 시각
  · **격추당한 시드 대 생존 시드**의 초반 40초 지표 비교(원인 후보 찾기)

사용:
    python _death_probe.py --target AIP_prev.dll --num-seeds 20
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
EARLY = 40.0
I_THR = 21


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ownship", default="AIP_DCS.dll")
    ap.add_argument("--target", required=True)
    ap.add_argument("--num-seeds", type=int, default=20)
    ap.add_argument("--seed-offset", type=int, default=0)
    args = ap.parse_args()

    ev = []          # 피격 이벤트 (연속 구간 단위)
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
        taken = dealt = 0.0
        death_t = kill_t = None
        run_len = 0
        run_start = None
        run_d = []
        eh = ev_ = ed = []
        early_dh, early_dv, early_dist, early_ma, early_ta = [], [], [], [], []
        n_ev = 0
        while not (terminated or truncated):
            _, _, terminated, truncated, _ = env.step(np.zeros(4, dtype=np.float32))
            step += 1
            t_s = step * DT
            g, o, t = env._geo_info, env._ownship_state, env._target_state
            d = float(g._get_distance(o, t))
            ma = abs(float(g._get_antenna_train_angle(o, t, False)))
            ta = abs(float(g._get_antenna_train_angle(t, o, False)))
            inb = BAND_MIN <= d <= BAND_MAX
            dw = (BAND_MAX - d) / (BAND_MAX - BAND_MIN) if inb else 0.0
            hitme = inb and ta <= 1.0
            hitthem = inb and ma <= 1.0
            if hitme:
                taken += dw * DT
                if run_len == 0:
                    run_start = t_s
                    run_d = []
                run_len += 1
                run_d.append((d, float(o[6]) - float(t[6]),
                              -float(o[2]) - (-float(t[2])), ma, float(o[I_THR])))
            else:
                if run_len > 0:
                    a = np.array(run_d)
                    ev.append(dict(seed=seed, t=run_start, dur=run_len * DT,
                                   d=float(a[:, 0].mean()), dv=float(a[:, 1].mean()),
                                   dh=float(a[:, 2].mean()), ma=float(a[:, 3].mean()),
                                   thr=float(a[:, 4].mean())))
                    n_ev += 1
                    run_len = 0
            if hitthem:
                dealt += dw * DT
            if death_t is None and taken >= 1.0:
                death_t = t_s
            if kill_t is None and dealt >= 1.0:
                kill_t = t_s
            if t_s <= EARLY:
                early_dh.append(-float(o[2]) - (-float(t[2])))
                early_dv.append(float(o[6]) - float(t[6]))
                early_dist.append(d)
                early_ma.append(ma)
                early_ta.append(ta)
        if run_len > 0:
            a = np.array(run_d)
            ev.append(dict(seed=seed, t=run_start, dur=run_len * DT,
                           d=float(a[:, 0].mean()), dv=float(a[:, 1].mean()),
                           dh=float(a[:, 2].mean()), ma=float(a[:, 3].mean()),
                           thr=float(a[:, 4].mean())))
            n_ev += 1
        env.close()
        died = death_t is not None
        killed = kill_t is not None and (death_t is None or kill_t < death_t)
        rows.append(dict(seed=seed, taken=taken, dealt=dealt, died=died, killed=killed,
                         death_t=death_t, n_ev=n_ev,
                         e_dh=float(np.median(early_dh)), e_dv=float(np.median(early_dv)),
                         e_dist=float(np.median(early_dist)),
                         e_ma=float(np.median(early_ma)), e_ta=float(np.median(early_ta))))
        r = rows[-1]
        mark = "💀격추당함" if died and not killed else ("✅격추함" if killed else "        ")
        print(f"  seed {seed:>2} {mark} 피격{r['taken']:.4f} 가함{r['dealt']:.4f} "
              f"이벤트{r['n_ev']:>2}"
              + (f" 사망{r['death_t']:.0f}s" if died else ""), flush=True)

    n = args.num_seeds
    dead = [r for r in rows if r["died"] and not r["killed"]]
    alive = [r for r in rows if not (r["died"] and not r["killed"])]
    print("\n" + "=" * 104)
    print(f"[피격 해부] {n}시드  나={args.ownship}  상대={args.target}   Phase1만")
    print(f"  격추당함 {len(dead)}판 / 격추함 {sum(1 for r in rows if r['killed'])}판 "
          f"/ 총 피격 이벤트 {len(ev)}건")

    if ev:
        a = ev
        f = lambda k: np.median([x[k] for x in a])
        print(f"\n  피격 이벤트(연속 구간) {len(a)}건 — 중앙값")
        print(f"    시각 {f('t'):5.0f}s   지속 {f('dur'):5.2f}s   거리 {f('d'):5.0f}m")
        print(f"    내ATA {f('ma'):5.1f}도   속도차(내-적) {f('dv'):+6.1f}   "
              f"고도차 {f('dh'):+6.0f}m   내스로틀 {f('thr'):.2f}")
        durs = np.array([x["dur"] for x in a])
        print(f"    지속 분포: 0.5s미만 {100*np.mean(durs<0.5):4.0f}%  "
              f"0.5~2s {100*np.mean((durs>=0.5)&(durs<2)):4.0f}%  "
              f"2s이상 {100*np.mean(durs>=2):4.0f}%   최장 {durs.max():.2f}s")
        ts = np.array([x["t"] for x in a])
        print(f"    시각 분포: 0~50s {100*np.mean(ts<50):4.0f}%  "
              f"50~100s {100*np.mean((ts>=50)&(ts<100)):4.0f}%  "
              f"100~150s {100*np.mean((ts>=100)&(ts<150)):4.0f}%  "
              f"150s+ {100*np.mean(ts>=150):4.0f}%")

    if dead and alive:
        print(f"\n  초반 {EARLY:.0f}초 비교 (중앙값)")
        print(f"  {'군':<10} {'n':>3} | {'고도차':>7} {'속도차':>7} {'거리':>7} "
              f"{'내ATA':>7} {'적ATA':>7}")
        for nm, rs in (("격추당함", dead), ("생존", alive)):
            g = lambda k: np.median([r[k] for r in rs])
            print(f"  {nm:<10} {len(rs):>3} | {g('e_dh'):>+7.0f} {g('e_dv'):>+7.1f} "
                  f"{g('e_dist'):>7.0f} {g('e_ma'):>7.1f} {g('e_ta'):>7.1f}")
        print("\n  ➜ 두 군에서 **부호가 갈리는 열**만 원인 후보다. 겹치면 기각.")
    print("=" * 104)


if __name__ == "__main__":
    main()
