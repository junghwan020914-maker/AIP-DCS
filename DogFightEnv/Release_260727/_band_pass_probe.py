"""밴드 통과 구조 분석 — "붙는데 왜 못 쏘는가"를 가른다.

08-10 arcV(수직 파이터)는 우리 최악 매치업이다(19.0/30). `_stalemate_diag.py`는
"공세밴드 0.0s"까지만 말했고 `_match_trace.py`는 t=60에 487m, t=150에 776m로
**밴드 안까지는 붙는다**고 말했다. 그 순간 ATA가 66도/32도라 못 쏠 뿐이다.

여기서 처방이 갈린다:
  (A) **오버슛** — 밴드 체류가 짧다(빠르게 스쳐 지나간다). 폐쇄율/거리 관리 문제.
  (B) **조준 불가** — 체류는 긴데 ATA가 안 줄어든다. 선회율/기하 문제.
  (C) **진입 자체가 드물다** — 통과 횟수가 적다. 접근 문제.
셋은 완전히 다른 처방이므로 반드시 갈라야 한다.

🔴 08-10 이 프로브로 **arcV 교착의 정체가 확정됐다 — 물리적 한계다.**

  (1) 접근은 문제가 아니다. arcV 상대 밴드 체류가 오히려 **5배**다
      (24.2s/판 vs arcE 4.6s/판, 통과 5.0회/판 vs 1.3회/판).
      그런데 50번 통과해 1도 안에 든 게 **0번**이다(arcE는 13번 중 10번).

  (2) 폐쇄율 가설 **기각**. 매치업 간에는 204 vs 67m/s로 그럴듯했지만
      **같은 매치업 안에서는 r = -0.514** — 폐쇄율이 높을수록 조준이 좋다.
      높은 폐쇄율은 이미 기수가 물렸을 때 나오는 **결과이지 원인이 아니다.**
      ⚠️ 매치업 간 상관을 인과로 읽지 말 것. 반드시 매치업 내부에서 검증한다.

  (3) 코너속도 가설 **기각**. arcV 내부 r = +0.159로 무관계다.
      오히려 arcE는 **151m/s로 느릴 때 완파**한다(r = +0.926).

  (4) **진짜 원인**: 요구 시선각속도가 우리 능력을 2배 넘는다.
          요구 p90 35.5도/s   실제 기수각속도 p90 16.6도/s   부족 49/50회
      거리 구간별로 보면 정확히 1/r로 스케일한다:
          152~ 500m  요구 66.5  가능 17.3  부족   ATA p10 23.5도
          500~ 914m  요구 37.4  가능 17.4  부족   ATA p10  9.7도
          914~1067m  요구 27.4  가능 17.4  부족   ATA p10 **4.8도**(최선)
          1067~1219m 요구 24.8  가능 17.5  부족   ATA p10  6.1도
          1219~1600m 요구 20.3  가능 17.7  부족   ATA p10 10.9도
          1600~2500m 요구 15.5  가능 17.8  **충족** ATA p10 24.3도
      **추적이 가능해지는 거리(1600m+)가 득점 최대 사거리(Phase3 1219m)보다 멀다.**
      두 구간이 겹치지 않는다.

  (5) 제어기 탓이 아니다. 231m/s에서 9G 순간선회율은 9.81*8.94/231 = **21.8도/s**이고
      우리는 17.8도/s로 그 **82%**를 내고 있다. 완벽한 제어기라도 22도/s가 상한이라
      필요한 27~37도/s에 못 미친다.

  ➡️ **arcV 교착은 조준 개선으로 이길 수 없다.** 이 매치업에 시도한 처방 13건이
     전부 실패한 이유가 여기 있다. 현재 동작(피격 0.0025, 패배 0)이 사실상 최적이다.
  ➡️ 부수 발견: **파고들수록 조준이 나빠진다**(ATA p10이 914~1067m에서 4.8도로 최선,
     152~500m에서 23.5도로 최악). "가까울수록 데미지 계수가 높다"는 직관이 역효과다.
     격렬하게 기동하는 상대 일반에 해당하므로 실전에서도 유효한 원리다.

각 "밴드 통과"(152~914m 연속 구간)마다 재는 것:
  지속시간 / 최소거리 / 그 구간 최소 ATA / 진입 시 폐쇄율 / 최소거리 시점의 ATA

사용:
    python _band_pass_probe.py --ownship AIP_DCS.dll --target AIP_arcV.dll --num-seeds 10
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ownship", default="AIP_DCS.dll")
    ap.add_argument("--target", required=True)
    ap.add_argument("--num-seeds", type=int, default=10)
    args = ap.parse_args()

    all_passes = []
    per_seed = []
    dist_los = []       # (거리, 시선각속도, 기수각속도, ATA) 전 틱
    for seed in range(args.num_seeds):
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
        my_hp = 0.0
        prev_d = None
        prev_los = None     # 시선 단위벡터
        prev_fwd = None     # 내 기수 단위벡터
        cur = None          # 진행 중인 통과
        passes = []
        while not (terminated or truncated):
            _, _, terminated, truncated, _ = env.step(np.zeros(4, dtype=np.float32))
            step += 1
            t_s = step * DT
            g, o, t = env._geo_info, env._ownship_state, env._target_state
            d = float(g._get_distance(o, t))
            ma = abs(float(g._get_antenna_train_angle(o, t, False)))
            my_hp += score_rate(d, ma, t_s)[0] * DT

            # 시선 회전율(deg/s) = 표적을 계속 겨누려면 필요한 최소 각속도
            los = np.array(t[0:3]) - np.array(o[0:3])
            los = los / max(np.linalg.norm(los), 1e-9)
            losrate = 0.0
            if prev_los is not None:
                c = float(np.clip(np.dot(los, prev_los), -1.0, 1.0))
                losrate = np.degrees(np.arccos(c)) / DT
            prev_los = los
            # 내 기수 회전율(deg/s) = 실제로 내는 각속도
            hd, pt = np.radians(float(o[5])), np.radians(float(o[4]))
            fwd = np.array([np.cos(pt)*np.cos(hd), np.cos(pt)*np.sin(hd), np.sin(pt)])
            fwdrate = 0.0
            if prev_fwd is not None:
                c = float(np.clip(np.dot(fwd, prev_fwd), -1.0, 1.0))
                fwdrate = np.degrees(np.arccos(c)) / DT
            prev_fwd = fwd

            dist_los.append((d, losrate, fwdrate, ma))
            inband = BAND_MIN <= d <= BAND_MAX
            if inband and cur is None:
                closure = (prev_d - d) / DT if prev_d is not None else 0.0
                cur = dict(t0=t_s, n=0, dmin=d, amin=ma, a_at_dmin=ma, closure=closure,
                           spd=[], v_at_dmin=float(o[6]), lr=[], fr=[])
            if inband:
                cur["n"] += 1
                cur["spd"].append(float(o[6]))
                cur["lr"].append(losrate)
                cur["fr"].append(fwdrate)
                if d < cur["dmin"]:
                    cur["dmin"] = d
                    cur["a_at_dmin"] = ma
                    cur["v_at_dmin"] = float(o[6])
                cur["amin"] = min(cur["amin"], ma)
            elif cur is not None:
                cur["dur"] = cur["n"] * DT
                passes.append(cur)
                cur = None
            prev_d = d
        if cur is not None:
            cur["dur"] = cur["n"] * DT
            passes.append(cur)
        env.close()

        all_passes.extend(passes)
        tot = sum(p["dur"] for p in passes)
        best = min((p["amin"] for p in passes), default=180.0)
        per_seed.append((seed, len(passes), tot, best, my_hp))
        print(f"  seed {seed:>2}  통과 {len(passes):>2}회  총체류 {tot:5.1f}s  "
              f"최소ATA {best:5.1f}도  내득점 {my_hp:.4f}", flush=True)

    print("\n" + "=" * 96)
    print(f"[밴드 통과 구조] {args.num_seeds}시드  나={args.ownship}  상대={args.target}")
    if not all_passes:
        print("  밴드 진입 0회 -> (C) 접근 실패")
        print("=" * 96)
        return

    dur = np.array([p["dur"] for p in all_passes])
    dmin = np.array([p["dmin"] for p in all_passes])
    amin = np.array([p["amin"] for p in all_passes])
    aatd = np.array([p["a_at_dmin"] for p in all_passes])
    clos = np.array([p["closure"] for p in all_passes])
    vmed = np.array([float(np.median(p["spd"])) if p["spd"] else 0.0 for p in all_passes])
    vatd = np.array([p["v_at_dmin"] for p in all_passes])
    npass = len(all_passes)

    print(f"  통과 {npass}회 (판당 {npass/args.num_seeds:.1f}회)  "
          f"총체류 {dur.sum():.1f}s (판당 {dur.sum()/args.num_seeds:.1f}s)")
    print(f"  통과 지속시간   중앙 {np.median(dur):5.2f}s  "
          f"p90 {np.percentile(dur,90):5.2f}s  최장 {dur.max():5.2f}s")
    print(f"  통과 최소거리   중앙 {np.median(dmin):5.0f}m  최소 {dmin.min():5.0f}m")
    print(f"  통과 최소ATA    중앙 {np.median(amin):5.1f}도  최소 {amin.min():5.1f}도  "
          f"<3도 통과 {int((amin<3).sum())}회  <1도 {int((amin<1).sum())}회")
    print(f"  최소거리 시점ATA 중앙 {np.median(aatd):5.1f}도")
    print(f"  진입 폐쇄율     중앙 {np.median(clos):+6.1f}m/s  p90 {np.percentile(clos,90):+6.1f}")
    print()
    # 매치업 간 상관은 인과가 아니다 — **같은 매치업 안에서** 폐쇄율과 조준이
    # 어떻게 붙는지 봐야 한다. 저폐쇄 통과도 조준이 안 되면 폐쇄율은 원인이 아니다.
    print(f"  통과 중 내속도   중앙 {np.median(vmed):5.1f}m/s  "
          f"최소거리 시점 {np.median(vatd):5.1f}m/s  "
          f"200m/s 미만 통과 {int((vmed<200).sum())}/{npass}회")
    lr90 = np.array([float(np.percentile(p["lr"], 90)) if p["lr"] else 0.0 for p in all_passes])
    fr90 = np.array([float(np.percentile(p["fr"], 90)) if p["fr"] else 0.0 for p in all_passes])
    print(f"  요구 시선각속도 p90 중앙 {np.median(lr90):5.1f}도/s   "
          f"실제 기수각속도 p90 중앙 {np.median(fr90):5.1f}도/s   "
          f"부족한 통과 {int((lr90 > fr90).sum())}/{npass}회")
    # 🔑 시선각속도는 거리에 반비례한다(w = v_perp / r). 멀리서 쏘면 요구량이 준다.
    #    규정상 Phase2는 1067m/2도, Phase3는 1219m/3도까지 득점이므로
    #    "붙지 말고 멀리서 쏜다"가 성립하는지 거리 구간별로 확인한다.
    dl = np.array(dist_los)
    print("  [거리 구간별 요구 시선각속도 — '멀리서 쏜다' 성립 여부]")
    for lo, hi in [(152,500),(500,914),(914,1067),(1067,1219),(1219,1600),(1600,2500)]:
        m = (dl[:,0] >= lo) & (dl[:,0] < hi)
        if m.sum() < 30:
            continue
        need = float(np.percentile(dl[m,1], 90))
        have = float(np.percentile(dl[m,2], 90))
        ata_lo = float(np.percentile(dl[m,3], 10))
        print(f"    {lo:>4}~{hi:>4}m  n={int(m.sum()):>6}  요구 p90 {need:5.1f}도/s  "
              f"가능 p90 {have:5.1f}도/s  {'충족' if need <= have else '부족':>4}  "
              f"ATA p10 {ata_lo:5.1f}도")
    print()
    # 🔑 08-10 최적 교전거리 — 직관이 아니라 데이터로.
    #    데미지 = 계수 x 조준유지시간이고, 계수는 가까울수록 크지만(152m 1.0 -> 914m 0)
    #    요구 시선각속도는 1/r로 줄어 멀수록 조준이 쉽다. 둘의 곱이 어디서 최대인가?
    print("  [거리 구간별 실제 득점률 — 최적 교전거리]")
    print("    구간          체류s   <1도%  <3도%   계수   기대득점률/s   비중")
    tot_t = len(dl) * DT
    for lo, hi in [(152,300),(300,450),(450,600),(600,750),(750,914)]:
        m = (dl[:,0] >= lo) & (dl[:,0] < hi)
        if m.sum() < 30:
            continue
        t = m.sum() * DT
        p1 = float((dl[m,3] < 1.0).mean())
        p3 = float((dl[m,3] < 3.0).mean())
        mid = (lo + hi) / 2.0
        coef = max(0.0, (914.4 - mid) / (914.4 - 152.4))   # Phase1 계수
        print(f"    {lo:>4}~{hi:>4}m  {t:7.1f}  {p1*100:5.2f}  {p3*100:5.2f}  "
              f"{coef:5.3f}  {coef*p1:11.5f}  {t/tot_t*100:5.1f}%")
    print("    (기대득점률 = Phase1 계수 x P(ATA<1도). 이 값이 큰 구간이 실제로 점수를 낸다)")
    print()
    print("  [같은 매치업 내 속도 구간별 — 코너속도 가설]")
    for lo, hi in [(0,200),(200,250),(250,300),(300,1e9)]:
        m = (vmed >= lo) & (vmed < hi)
        if m.sum() == 0:
            continue
        lbl = f"{lo:.0f}~{hi:.0f}" if hi < 1e8 else f"{lo:.0f}+"
        print(f"    속도 {lbl:>8} m/s  n={int(m.sum()):>3}  "
              f"최소ATA 중앙 {np.median(amin[m]):5.1f}도  <3도 {int((amin[m]<3).sum()):>2}회")
    if npass >= 8:
        rv = float(np.corrcoef(vmed, amin)[0, 1])
        print(f"    속도 vs 최소ATA 상관 r = {rv:+.3f}  "
              f"({'빠를수록 조준 좋음' if rv < -0.25 else '느릴수록 조준 좋음' if rv > 0.25 else '관계 약함'})")
    print()
    print("  [같은 매치업 내 폐쇄율 구간별]")
    edges = [0, 100, 200, 300, 1e9]
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (clos >= lo) & (clos < hi)
        if m.sum() == 0:
            continue
        lbl = f"{lo:.0f}~{hi:.0f}" if hi < 1e8 else f"{lo:.0f}+"
        print(f"    폐쇄 {lbl:>9} m/s  n={int(m.sum()):>3}  "
              f"최소ATA 중앙 {np.median(amin[m]):5.1f}도  "
              f"<3도 {int((amin[m]<3).sum()):>2}회  "
              f"지속 중앙 {np.median(dur[m]):4.2f}s  "
              f"최소거리 중앙 {np.median(dmin[m]):4.0f}m")
    if npass >= 8:
        r = float(np.corrcoef(clos, amin)[0, 1])
        print(f"    폐쇄율 vs 최소ATA 상관계수 r = {r:+.3f}  "
              f"({'폐쇄율 높을수록 조준 나쁨' if r > 0.25 else '폐쇄율 높을수록 조준 좋음(=결과이지 원인 아님)' if r < -0.25 else '관계 약함'})")
    print()
    short = float((dur < 2.0).mean())
    print(f"  ➜ 2초 미만 통과 비율 {short*100:.0f}%")
    if short > 0.6:
        print("     **(A) 오버슛** — 빠르게 스쳐 지나간다. 폐쇄율/거리 관리가 문제다.")
    elif np.median(amin) > 10.0:
        print("     **(B) 조준 불가** — 체류는 있는데 ATA가 안 줄어든다. 선회율/기하 문제다.")
    else:
        print("     체류도 조준도 되는데 무득점 -> 거리계수/phase 타이밍을 볼 것.")
    print("=" * 96)


if __name__ == "__main__":
    main()
