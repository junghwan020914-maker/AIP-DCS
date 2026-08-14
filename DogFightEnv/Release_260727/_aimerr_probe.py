"""조준 오차를 **지연 대 진동**으로 분해한다 — 추적 개선의 방향을 정하기 위해.

08-14 배경. `_factor_probe.py`가 병목을 조준으로 특정했다. 특히 `arcD`(절대 공격하지
않는 순수 방어형)에서:

    공세밴드 20.8s (밴드의 94% — 위치 교란이 없다)
    순수조준률 **8.56%**    공세득점 1.78s
    그리고 **1.78초의 조준이 격추(HP 101.9)**다.

즉 위치도 거리도 다 갖춰준 상태에서 91%의 시간을 못 겨눈다. 그 91%에서 무엇이
틀리는지 알아야 처방이 정해진다. 원인이 둘 중 어느 쪽이냐에 따라 **처방이 정반대**다:

    지연(lag)      상대 기동을 못 따라가 뒤처진다      -> 예측/리드가 필요
    진동(oscil.)   목표를 중심으로 흔들린다            -> 감쇠가 필요

⚠️ 이 프로젝트에서 "부드럽게 만들면 나아진다"는 직관은 이미 두 번 틀렸다
   (롤 이동평균 필터 기각 110.5 vs 115.5 / VP 연속성 측정에서 우리가 v42보다 87배
   연속적임이 드러남). 그러니 **감쇠 쪽으로 기울지 말고 숫자로 판정할 것.**

분해 방법 — 공세+밴드 구간에서만:
  · ATA를 방위(az)/고도(el) 성분으로 나눈다. 한쪽에 몰려 있으면 그 축의 문제다.
  · **부호 지속성**: 오차 부호가 오래 유지되면 지연, 자주 뒤집히면 진동이다.
    (부호반전율 = 인접 틱에서 부호가 바뀐 비율)
  · **상대 각속도와의 상관**: 상대가 세게 돌 때 오차가 커지면 추종 지연이다.
  · 정상편차(중앙값 |오차|) 대 변동(표준편차)의 비.

사용:
    python _aimerr_probe.py --target AIP_arcD.dll --num-seeds 10
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
    """도 단위 오일러각 -> (전방, 우측, 상방) 단위벡터. NED 기준, Z는 위가 +."""
    r, p, y = np.radians([roll, pitch, yaw])
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    fwd = np.array([cp * cy, cp * sy, sp])
    right = np.array([sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, -sr * cp])
    up = np.cross(right, fwd)
    n = np.linalg.norm(up)
    if n > 1e-9:
        up = up / n
    return fwd, right, up


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ownship", default="AIP_DCS.dll")
    ap.add_argument("--target", required=True)
    ap.add_argument("--num-seeds", type=int, default=10)
    ap.add_argument("--seed-offset", type=int, default=0)
    args = ap.parse_args()

    AZ, EL, ATA, OMG, AOA, NZ = [], [], [], [], [], []
    flips_az = flips_el = pairs = 0
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
        prev_az = prev_el = None
        prev_tf = None
        while not (terminated or truncated):
            _, _, terminated, truncated, _ = env.step(np.zeros(4, dtype=np.float32))
            g, o, t = env._geo_info, env._ownship_state, env._target_state
            d = float(g._get_distance(o, t))
            ma = abs(float(g._get_antenna_train_angle(o, t, False)))
            ta = abs(float(g._get_antenna_train_angle(t, o, False)))
            # 상대 기수 회전 각속도 (공용: 우리가 따라가야 할 양)
            tf, _, _ = body_axes(float(t[3]), float(t[4]), float(t[5]))
            omega = 0.0
            if prev_tf is not None:
                c = float(np.clip(np.dot(prev_tf, tf), -1.0, 1.0))
                omega = np.degrees(np.arccos(c)) / DT
            prev_tf = tf
            if not (BAND_MIN <= d <= BAND_MAX and ma < FRONT and ta > REAR):
                prev_az = prev_el = None
                continue
            fwd, right, up = body_axes(float(o[3]), float(o[4]), float(o[5]))
            los = np.array([float(t[0]) - float(o[0]), float(t[1]) - float(o[1]),
                            -(float(t[2]) - float(o[2]))])
            n = np.linalg.norm(los)
            if n < 1e-6:
                continue
            los = los / n
            az = np.degrees(np.arctan2(float(np.dot(los, right)), float(np.dot(los, fwd))))
            el = np.degrees(np.arctan2(float(np.dot(los, up)), float(np.dot(los, fwd))))
            AZ.append(az); EL.append(el); ATA.append(ma); OMG.append(omega)
            AOA.append(float(o[13])); NZ.append(abs(float(o[31])))
            if prev_az is not None:
                pairs += 1
                if az * prev_az < 0:
                    flips_az += 1
                if el * prev_el < 0:
                    flips_el += 1
            prev_az, prev_el = az, el
        env.close()
        print(f"  seed {seed} 완료", flush=True)

    if not AZ:
        print("공세+밴드 표본 없음")
        return
    az = np.array(AZ); el = np.array(EL); ata = np.array(ATA); om = np.array(OMG)
    aoa = np.array(AOA); nz = np.array(NZ)
    print("\n" + "=" * 96)
    print(f"[조준 오차 분해] {args.num_seeds}시드  상대={args.target}  "
          f"공세+밴드 표본 {len(az)} ({len(az)*DT:.1f}s)")
    print(f"  ATA        중앙 {np.median(ata):6.2f}도   1도미만 {100*np.mean(ata<1):5.2f}%")
    print(f"  방위 az    중앙 {np.median(az):+6.2f}   |az| 중앙 {np.median(np.abs(az)):6.2f}   "
          f"표준편차 {az.std():6.2f}")
    print(f"  고도 el    중앙 {np.median(el):+6.2f}   |el| 중앙 {np.median(np.abs(el)):6.2f}   "
          f"표준편차 {el.std():6.2f}")
    big = np.abs(az) > np.abs(el)
    print(f"  오차가 큰 축: 방위 {100*np.mean(big):4.0f}%  /  고도 {100*np.mean(~big):4.0f}%")

    print(f"\n  부호반전율 (높으면 진동, 낮으면 지연)")
    print(f"    방위 {100*flips_az/max(pairs,1):5.2f}%   고도 {100*flips_el/max(pairs,1):5.2f}%"
          f"   (표본쌍 {pairs})")

    print(f"\n  상대 각속도와의 관계 (지연이면 상대가 셀수록 오차가 커진다)")
    print(f"  {'상대 각속도':>14} {'표본':>7} {'ATA중앙':>8} {'|az|중앙':>9} {'|el|중앙':>9}")
    for lo, hi in [(0, 2), (2, 5), (5, 10), (10, 20), (20, 1e9)]:
        m = (om >= lo) & (om < hi)
        if m.sum() < 50:
            continue
        print(f"  {lo:>5.0f}~{min(hi,99):<7.0f} {int(m.sum()):>7} {np.median(ata[m]):>8.2f} "
              f"{np.median(np.abs(az[m])):>9.2f} {np.median(np.abs(el[m])):>9.2f}")
    if len(om) > 100:
        r = float(np.corrcoef(om, ata)[0, 1])
        print(f"    상관계수 corr(상대각속도, ATA) = {r:+.3f}")

    # 편향이 상수면 상수 보정, 받음각/G에 비례하면 피드포워드가 답이다.
    print("\n  [편향의 정체] 받음각·G와의 관계")
    k, b = np.polyfit(np.abs(aoa), el, 1)
    print(f"    corr(|AOA|, el) = {np.corrcoef(np.abs(aoa), el)[0,1]:+.3f}   "
          f"corr(Nz, el) = {np.corrcoef(nz, el)[0,1]:+.3f}")
    print(f"    회귀:  el ≈ {k:+.4f} * |AOA| {b:+.3f}    "
          f"(기울기가 -1 근처면 **기수-속도벡터 차이**가 원인)")
    print(f"  {'|AOA| 구간':>12} {'표본':>7} {'el 중앙':>9} {'|el| 중앙':>10} {'ATA 중앙':>9}")
    for lo, hi in [(0, 4), (4, 8), (8, 12), (12, 16), (16, 90)]:
        m = (np.abs(aoa) >= lo) & (np.abs(aoa) < hi)
        if m.sum() < 50:
            continue
        print(f"  {lo:>5.0f}~{hi:<6.0f} {int(m.sum()):>7} {np.median(el[m]):>+9.2f} "
              f"{np.median(np.abs(el[m])):>10.2f} {np.median(ata[m]):>9.2f}")

    print("\n  ➜ 판정")
    print("     · 부호반전율이 높고(>30%) 표준편차가 중앙값보다 크면 **진동** -> 감쇠")
    print("     · 부호반전율이 낮고(<15%) 상대 각속도와 상관이 크면 **지연** -> 예측/리드")
    print("     · 한 축에 몰려 있으면 그 축의 제어 문제(방위=롤/러더, 고도=피치)")
    print("=" * 96)


if __name__ == "__main__":
    main()
