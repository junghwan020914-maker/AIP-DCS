"""실제 교전 중 **받음각·G·스로틀**을 계측한다 — 비행 포락선을 얼마나 쓰고 있나.

08-13 배경. "뒤를 잡히면 못 벗어나고 계속 도망만 간다"는 실서버 관찰에 대해
감속으로 오버슛을 강제하는 최후수단(LastDitch) 노드를 검토 중이다. 설계 전에
**우리가 지금 포락선의 어디에 있는지부터** 재야 한다(오늘 계측 없이 설계해서
실패한 처방이 여러 건이다).

이 기체(`aircraft/f16/f16.xml`)의 한계는 소스에서 확인했다:
  · `fcs/elevator-scheduler` — 승강타 게인이 0.5rad(28.6도)에서 0.11,
    0.5236rad(30도)에서 **0.0**. 30도에서 피치 권한이 사라진다.
  · `fcs/alpha-limiter-norm` — 받음각에 비례(gain 1.0472)한 기수내림을
    피치 명령에 **더한다**. 주석: "Command full pitch down when approaching 30 deg alpha".
  · 공력 테이블은 45도까지만 있고 JSBSim은 외삽하지 않는다.
  → 즉 **진짜 post-stall은 불가능**하고, 쓸 수 있는 상한은 약 30도다.

그래서 답해야 할 질문:
  (1) 교전 중 실제 받음각이 얼마인가. 30도 상한에 붙어 있나, 여유가 있나.
  (2) Nz(G)는 9G 한계에 붙어 있나. 즉 선회가 G 제한인가 받음각 제한인가.
  (3) **수세일 때** 스로틀을 뭘로 쓰고 있나. (트리에 감속 분기가 없다는 정적 확인의 실측)

상태 인덱스(FighterSim._update_state 기준):
  [2] D(NED)  [6] u(body x, m/s)  [12] KCAS  [13] AOA deg  [14] AOS deg
  [27] KTAS   [31] Nz(G)  [21] throttle cmd  [40] speedbrake deg

사용:
    python _aoa_probe.py --target AIP_v42.dll --num-seeds 10
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

I_AOA, I_AOS, I_NZ, I_KTAS, I_THR, I_SB = 13, 14, 31, 27, 21, 40
FRONT, REAR = 50.0, 130.0


def pct(a, q):
    return float(np.percentile(a, q)) if len(a) else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ownship", default="AIP_DCS.dll")
    ap.add_argument("--target", required=True)
    ap.add_argument("--num-seeds", type=int, default=10)
    ap.add_argument("--seed-offset", type=int, default=0)
    args = ap.parse_args()

    aoa_all, nz_all, thr_all, sb_all, ktas_all = [], [], [], [], []
    aoa_def, thr_def, ktas_def = [], [], []     # 수세(내 6시에 상대)
    aoa_off, thr_off = [], []                   # 공세
    dv_def = []                                 # 수세일 때 속도차(내-상대)
    # 코너속도 확인용: 속도 대비 실제 선회율. 속도벡터의 회전각속도로 잰다.
    spd_rate = []                               # (KTAS, 선회율 deg/s, Nz, AOA)

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
        # ⚠️ 위치는 1e-6도(약 0.11m)로 양자화돼 있다(FighterSim이 out_fdm.Lat/1e6을 쓴다).
        # 1스텝(3.6m) 차분으로 각도를 재면 잡음이 1.75도/스텝 = 105도/s로 신호를 덮는다.
        # 그래서 **12스텝(0.2초, 약 56m)** 기준선으로 잰다. 잡음은 1/12로 줄어든다.
        # 교차검증용으로 Nz·속도에서 나오는 이론 선회율도 같이 남긴다: w = g*sqrt(Nz^2-1)/V.
        BASE = 12
        hist = []
        while not (terminated or truncated):
            _, _, terminated, truncated, _ = env.step(np.zeros(4, dtype=np.float32))
            o, t = env._ownship_state, env._target_state
            hist.append(np.array([float(o[0]), float(o[1]), float(o[2])]))
            if len(hist) > 2 * BASE + 1:
                hist.pop(0)
            if len(hist) == 2 * BASE + 1:
                d0 = hist[BASE] - hist[0]
                d1 = hist[2 * BASE] - hist[BASE]
                n0, n1 = np.linalg.norm(d0), np.linalg.norm(d1)
                if n0 > 1.0 and n1 > 1.0:
                    c = float(np.clip(np.dot(d0 / n0, d1 / n1), -1.0, 1.0))
                    geo = np.degrees(np.arccos(c)) / (BASE * DT)
                    v = max(float(o[I_KTAS]), 1.0)
                    nzv = abs(float(o[I_NZ]))
                    theo = np.degrees(9.80665 * np.sqrt(max(nzv * nzv - 1.0, 0.0)) / v)
                    spd_rate.append((v, geo, nzv, abs(float(o[I_AOA])), theo))
            g = env._geo_info
            ma = abs(float(g._get_antenna_train_angle(o, t, False)))
            ta = abs(float(g._get_antenna_train_angle(t, o, False)))
            aoa = abs(float(o[I_AOA]))
            nz = float(o[I_NZ])
            thr = float(o[I_THR])
            kt = float(o[I_KTAS])
            aoa_all.append(aoa); nz_all.append(nz); thr_all.append(thr)
            sb_all.append(float(o[I_SB])); ktas_all.append(kt)
            if ma > REAR and ta < FRONT:                 # 수세
                aoa_def.append(aoa); thr_def.append(thr); ktas_def.append(kt)
                dv_def.append(kt - float(t[I_KTAS]))
            elif ma < FRONT and ta > REAR:               # 공세
                aoa_off.append(aoa); thr_off.append(thr)
        env.close()
        print(f"  seed {seed} 완료", flush=True)

    a = np.array(aoa_all); nz = np.array(nz_all); th = np.array(thr_all)
    print("\n" + "=" * 92)
    print(f"[포락선 계측] {args.num_seeds}시드  나={args.ownship}  상대={args.target}"
          f"   샘플 {len(a)}")
    print(f"  받음각(deg)  중앙 {np.median(a):5.1f}  p90 {pct(a,90):5.1f}  "
          f"p99 {pct(a,99):5.1f}  최대 {a.max():5.1f}")
    print(f"    25도 초과 {100*np.mean(a>25):5.2f}%   28도 초과 {100*np.mean(a>28):5.2f}%"
          f"   30도 초과 {100*np.mean(a>30):5.2f}%")
    print(f"  Nz(G)        중앙 {np.median(nz):5.2f}  p90 {pct(nz,90):5.2f}  "
          f"p99 {pct(nz,99):5.2f}  최대 {nz.max():5.2f}   8G초과 {100*np.mean(nz>8):5.2f}%")
    print(f"  스로틀       중앙 {np.median(th):5.2f}  최소 {th.min():5.2f}  "
          f"0.9미만 {100*np.mean(th<0.9):5.2f}%")
    print(f"  스피드브레이크 최대 {max(sb_all):5.1f}도  전개(>1도) {100*np.mean(np.array(sb_all)>1):5.2f}%")
    print(f"  속도(KTAS)   중앙 {np.median(ktas_all):5.1f} m/s")

    print()
    for label, aa, tt in (("수세", aoa_def, thr_def), ("공세", aoa_off, thr_off)):
        if not aa:
            print(f"  {label}: 표본 없음")
            continue
        aa = np.array(aa); tt = np.array(tt)
        print(f"  {label}(n={len(aa):>6})  받음각 중앙 {np.median(aa):5.1f} p90 {pct(aa,90):5.1f} "
              f"최대 {aa.max():5.1f} | 스로틀 중앙 {np.median(tt):5.2f} 최소 {tt.min():5.2f} "
              f"0.9미만 {100*np.mean(tt<0.9):5.1f}%")
    if dv_def:
        d = np.array(dv_def)
        print(f"  수세 속도차(내-상대) 중앙 {np.median(d):+6.1f} m/s  "
              f"(양수면 내가 더 빠르다 = 오버슛 유도 여지)")

    # 코너속도: 속도 구간별 **상위 선회율**(그 속도에서 낼 수 있는 최대치의 대리값)
    if spd_rate:
        sr = np.array(spd_rate)
        print()
        print("  [속도 대비 선회율]  코너속도 = 선회율이 최대인 속도.")
        print("  기하=위치 12스텝 차분 / 이론=g*sqrt(Nz^2-1)/V. 둘이 맞으면 신뢰할 수 있다.")
        print(f"  {'KTAS 구간':>12} {'표본':>7} | {'기하 p95':>8} {'기하중앙':>8} | "
              f"{'이론 p95':>8} {'이론중앙':>8} | {'Nz p95':>7} {'AOA p95':>8}")
        edges = [0, 120, 150, 180, 200, 220, 240, 260, 280, 300, 320, 1e9]
        for lo, hi in zip(edges[:-1], edges[1:]):
            m = (sr[:, 0] >= lo) & (sr[:, 0] < hi)
            if m.sum() < 30:
                continue
            s = sr[m]
            print(f"  {lo:>5.0f}~{min(hi,999):<5.0f} {int(m.sum()):>7} | "
                  f"{pct(s[:,1],95):>8.1f} {np.median(s[:,1]):>8.1f} | "
                  f"{pct(s[:,4],95):>8.1f} {np.median(s[:,4]):>8.1f} | "
                  f"{pct(s[:,2],95):>7.2f} {pct(s[:,3],95):>8.1f}")

    print()
    print("  ➜ 판정 기준")
    print("     · 받음각 p99가 25도 근처면 이미 한계를 쓰고 있다 -> 감속 카드는 약하다.")
    print("     · p99가 15도 안팎이면 포락선을 크게 남기고 있다 -> LastDitch 여지가 크다.")
    print("     · Nz가 8G를 자주 넘으면 선회는 **G 제한**이지 받음각 제한이 아니다.")
    print("     · 수세 스로틀이 계속 1.0이면 감속 분기가 없다는 정적 확인이 실측으로 확정된다.")
    print("=" * 92)


if __name__ == "__main__":
    main()
