"""피치 **제어 권한**을 잰다 — 조준점이 아니라 거기 도달하는 능력이 문제인가.

08-14 배경. 병목이 조준으로 특정됐고(`_factor_probe.py`), 오차의 성격도 나왔다
(`_aimerr_probe.py`, arcD 11745표본):

    오차가 큰 축   방위 10% / **고도 90%**   (|az| 0.34도 vs |el| 1.92도)
    부호반전율     0.58%                      -> 진동이 아니라 고정 편향
    상대각속도 상관 +0.084                     -> 추종 지연도 아니다

**조준점을 받음각만큼 내리는 처방은 기각됐다**(조준률 7.94% -> 0.67% 단조 악화).
그건 상관을 정책으로 뒤집은 것이었고 이 프로젝트에서 그 방식은 5전 5패다.

⚠️ 이번 가설은 **상관이 아니라 코드 구조**에서 나온다. `Controller_CY.cpp`:

    PitchCMD = ERROR_Effect * Roll_Effect * Horizon_Effect * (-1)
    Roll_Effect = clamp(cos(UTAngle), 0, 1)
    UTAngle = angle(내 UpVector,  표적방향의 기수수직 성분)

**`Roll_Effect`는 롤이 정렬되지 않으면 피치 명령을 통째로 0으로 만든다.** 즉 조준점을
어디에 찍든 제어기가 그쪽으로 갈 **권한 자체가 없는** 구간이 존재한다. 파일 주석도
이미 그렇게 적고 있다 — "Roll_Effect가 큰 각도에서 0으로 죽어, VP를 멀리 찍어도
제어기가 그쪽으로 갈 능력 자체가 없다"(75도 클램프를 우회책이 아니라 **제어기를 자기가
잘 동작하는 영역 안에 붙잡아두는 장치**라고 결론낸 근거).

**가설**: 고도 오차가 큰 순간이 곧 `Roll_Effect`가 낮은 순간이다.
   맞으면 처방은 조준점이 아니라 **롤 정렬 우선순위**나 `Roll_Effect` 하한이다.
   틀리면(권한은 충분한데 못 맞춘다면) 남은 건 게인·구조 문제다.

DLL을 안 건드리고 상태 벡터만으로 제어기 공식을 재현해 잰다.

사용:
    python _pitchauth_probe.py --target AIP_arcD.dll --num-seeds 8
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


def body_axes(roll, pitch, yaw):
    r, p, y = np.radians([roll, pitch, yaw])
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    fwd = np.array([cp * cy, cp * sy, sp])
    right = np.array([sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, -sr * cp])
    up = np.cross(right, fwd)
    n = np.linalg.norm(up)
    return fwd, right, (up / n if n > 1e-9 else up)


def pct(a, q):
    return float(np.percentile(a, q)) if len(a) else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ownship", default="AIP_DCS.dll")
    ap.add_argument("--target", required=True)
    ap.add_argument("--num-seeds", type=int, default=8)
    ap.add_argument("--seed-offset", type=int, default=0)
    args = ap.parse_args()

    RE, ATA, EL, AZ = [], [], [], []
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
        while not (terminated or truncated):
            _, _, terminated, truncated, _ = env.step(np.zeros(4, dtype=np.float32))
            g, o, t = env._geo_info, env._ownship_state, env._target_state
            d = float(g._get_distance(o, t))
            ma = abs(float(g._get_antenna_train_angle(o, t, False)))
            ta = abs(float(g._get_antenna_train_angle(t, o, False)))
            if not (BAND_MIN <= d <= BAND_MAX and ma < FRONT and ta > REAR):
                continue
            fwd, right, up = body_axes(float(o[3]), float(o[4]), float(o[5]))
            los = np.array([float(t[0]) - float(o[0]), float(t[1]) - float(o[1]),
                            -(float(t[2]) - float(o[2]))])
            n = np.linalg.norm(los)
            if n < 1e-6:
                continue
            los = los / n
            # 제어기 재현: 표적방향에서 기수 성분을 뺀 나머지(= 당김 평면에 놓인 성분)
            proj = los - fwd * float(np.dot(los, fwd))
            pl = np.linalg.norm(proj)
            if pl < 1e-9:
                continue
            ut = float(np.clip(np.dot(up, proj / pl), -1.0, 1.0))
            RE.append(max(0.0, ut))          # Roll_Effect = clamp(cos(UT),0,1)
            ATA.append(ma)
            EL.append(np.degrees(np.arctan2(float(np.dot(los, up)),
                                            float(np.dot(los, fwd)))))
            AZ.append(np.degrees(np.arctan2(float(np.dot(los, right)),
                                            float(np.dot(los, fwd)))))
        env.close()
        print(f"  seed {seed} 완료", flush=True)

    if not RE:
        print("공세+밴드 표본 없음")
        return
    re_ = np.array(RE); ata = np.array(ATA); el = np.array(EL); az = np.array(AZ)
    print("\n" + "=" * 96)
    print(f"[피치 제어 권한] {args.num_seeds}시드  상대={args.target}  "
          f"공세+밴드 표본 {len(re_)} ({len(re_)*DT:.1f}s)")
    print(f"  Roll_Effect = clamp(cos(UT),0,1)  — 이 값이 곧 피치 명령의 배율이다")
    print(f"    중앙 {np.median(re_):.3f}  p10 {pct(re_,10):.3f}  p25 {pct(re_,25):.3f}  "
          f"p75 {pct(re_,75):.3f}")
    for th in (0.1, 0.3, 0.5, 0.8):
        print(f"    {th:.1f} 미만 비율 {100*np.mean(re_ < th):5.2f}%")

    print(f"\n  ➜ 핵심: **조준 성공/실패에서 Roll_Effect가 갈리는가**")
    ok = ata <= 1.0
    print(f"  {'구분':<14} {'표본':>7} {'Roll_Effect 중앙':>16} {'p25':>7} "
          f"{'|el| 중앙':>10} {'|az| 중앙':>10}")
    for nm, m in (("조준 성공(<1도)", ok), ("조준 실패(>=1도)", ~ok)):
        if m.sum() < 10:
            print(f"  {nm:<14} {int(m.sum()):>7}  표본 부족")
            continue
        print(f"  {nm:<14} {int(m.sum()):>7} {np.median(re_[m]):>16.3f} "
              f"{pct(re_[m],25):>7.3f} {np.median(np.abs(el[m])):>10.2f} "
              f"{np.median(np.abs(az[m])):>10.2f}")

    print(f"\n  Roll_Effect 구간별 조준 성공률과 오차")
    print(f"  {'RE 구간':>12} {'표본':>7} {'조준성공률':>10} {'ATA 중앙':>9} "
          f"{'|el| 중앙':>10} {'|az| 중앙':>10}")
    for lo, hi in [(0, 0.2), (0.2, 0.4), (0.4, 0.6), (0.6, 0.8), (0.8, 1.01)]:
        m = (re_ >= lo) & (re_ < hi)
        if m.sum() < 30:
            continue
        print(f"  {lo:>5.1f}~{hi:<6.1f} {int(m.sum()):>7} {100*np.mean(ata[m]<=1):>9.2f}% "
              f"{np.median(ata[m]):>9.2f} {np.median(np.abs(el[m])):>10.2f} "
              f"{np.median(np.abs(az[m])):>10.2f}")
    if len(re_) > 100:
        print(f"\n    corr(Roll_Effect, |el|) = {np.corrcoef(re_, np.abs(el))[0,1]:+.3f}")
        print(f"    corr(Roll_Effect, ATA)  = {np.corrcoef(re_, ata)[0,1]:+.3f}")

    print("\n  ➜ 판정")
    print("     · RE가 낮은 구간에서 조준성공률이 뚜렷이 낮으면 **권한 부족**이 원인이다")
    print("       -> 처방은 조준점이 아니라 롤 정렬 우선순위 / RE 하한이다")
    print("     · RE와 무관하게 성공률이 낮으면 권한은 충분하고 **게인·구조** 문제다")
    print("=" * 96)


if __name__ == "__main__":
    main()
