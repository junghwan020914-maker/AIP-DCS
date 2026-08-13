"""추적점(VP) 연속성을 **같은 판에서 양쪽 동시에** 측정한다 — 이산 전환 가설 검증.

08-13 배경. ryujan v42 배포 노트가 자기 설계 의도를 이렇게 적었다:

    "Task_LeadPredict가 650줄이고 그 안에서 리드예측 -> 뱅크 횡예측 -> 궤도추종 ->
     종말조준 게이트 -> 상하 클램프 -> 고도하한 -> dV 폐루프 스로틀을 **연속으로 혼합**한다.
     분기로 끊지 않는 게 설계 의도다(모드 전환의 불연속을 피하려고)."

    행동 노드    ryujan v42: 2개  /  우리: 11개
    게이트       1종            /  6종

우리는 v42 상대 10판 중 9판에서 **공세 국면이 정확히 0.0%**다(공세밴드 0.0s).
"기회 자체가 없다"는 위치/국면 문제인데, 그 원인 후보 중 하나가 **이산 모드 전환의
불연속**이다. 모드가 바뀔 때마다 조준점이 튀면 기수가 한 방향으로 수렴하지 못한다.

⚠️ 이건 아직 **가설**이다. 재설계 전에 증상부터 잰다. 증상이 없으면 가설은 기각된다.

측정: 매 틱 `VP - 내위치`의 **방향 변화각**(deg/tick). 모드 전환이 조준점을 튀게 하면
여기 큰 점프로 나타난다. 두 기체를 **같은 매치에서 동시에** 재므로 기하가 완전히 동일하다.

⚠️ 주의 — 큰 값이 곧 나쁜 것은 아니다. 근거리에서는 같은 각속도라도 방향 변화가
   크게 잡힌다(1/r). 그래서 거리 구간을 나눠서도 본다.

사용:
    python _vp_continuity.py --num-seeds 8
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


class VPSpy:
    """BTActionProvider를 감싸 VP와 액션을 기록한다. env는 원본과 동일하게 동작한다."""

    def __init__(self, dll_name):
        self.inner = BTActionProvider(dll_name=dll_name)
        self.vp = []
        self.act = []

    def compute_action(self, context):
        r = self.inner.compute_action(context)
        v = r.info.get("vp")
        self.vp.append(np.array(v, dtype=float) if v is not None else None)
        self.act.append(np.array(r.action, dtype=float))
        return r

    def reset(self, context=None):
        if hasattr(self.inner, "reset"):
            self.inner.reset(context)

    def close(self):
        if hasattr(self.inner, "close"):
            self.inner.close()


def turn_series(vp_list, pos_list):
    """VP 방향(내위치 -> VP)의 틱당 변화각(deg)."""
    out = []
    prev = None
    for v, p in zip(vp_list, pos_list):
        if v is None or not np.all(np.isfinite(v)):
            prev = None
            out.append(np.nan)
            continue
        d = v - p
        n = np.linalg.norm(d)
        if n < 1e-6:
            prev = None
            out.append(np.nan)
            continue
        u = d / n
        if prev is None:
            out.append(np.nan)
        else:
            c = float(np.clip(np.dot(prev, u), -1.0, 1.0))
            out.append(np.degrees(np.arccos(c)))
        prev = u
    return np.array(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ownship", default="AIP_DCS.dll")
    ap.add_argument("--target", default="AIP_ryujan_v42.dll")
    ap.add_argument("--num-seeds", type=int, default=8)
    args = ap.parse_args()

    A_all, B_all, D_all = [], [], []
    A_thr, B_thr = [], []
    for seed in range(args.num_seeds):
        rng = np.random.default_rng(seed)
        own, tgt = make_state(rng)
        spyA, spyB = VPSpy(args.ownship), VPSpy(args.target)
        env = DogFightWrapper(
            env_config={
                "observation_mode": "tactical16", "ownship_control_mode": "rl",
                "target_mode": "rl", "max_engage_time": 200.0, "min_altitude": 300.0,
                "ownship": own, "target": tgt,
                "initial_scenario": {"mode": "default"},
            },
            ownship_action_provider=spyA, target_action_provider=spyB,
        )
        env.reset(seed=seed)
        posA, posB, dist = [], [], []
        terminated = truncated = False
        while not (terminated or truncated):
            _, _, terminated, truncated, _ = env.step(np.zeros(4, dtype=np.float32))
            o, t = env._ownship_state, env._target_state
            posA.append(np.array([float(o[0]), float(o[1]), -float(o[2])]))
            posB.append(np.array([float(t[0]), float(t[1]), -float(t[2])]))
            dist.append(float(env._geo_info._get_distance(o, t)))
        env.close()
        n = min(len(posA), len(spyA.vp), len(spyB.vp))
        A_all.append(turn_series(spyA.vp[:n], posA[:n]))
        B_all.append(turn_series(spyB.vp[:n], posB[:n]))
        D_all.append(np.array(dist[:n]))
        A_thr.append(np.array([a[3] for a in spyA.act[:n]]))
        B_thr.append(np.array([a[3] for a in spyB.act[:n]]))
        print(f"  seed {seed} 완료 ({n} 틱)", flush=True)

    A = np.concatenate(A_all); B = np.concatenate(B_all); D = np.concatenate(D_all)
    At = np.concatenate(A_thr); Bt = np.concatenate(B_thr)
    m = np.isfinite(A) & np.isfinite(B)
    A, B, D, At, Bt = A[m], B[m], D[m], At[m], Bt[m]

    def line(label, x):
        return (f"  {label:<16} 중앙 {np.median(x):6.3f}  p90 {np.percentile(x,90):6.3f}  "
                f"p99 {np.percentile(x,99):6.3f}  최대 {x.max():7.2f} | "
                f">5도 {100*np.mean(x>5):5.2f}%  >15도 {100*np.mean(x>15):5.2f}%  "
                f">45도 {100*np.mean(x>45):5.2f}%")

    print("\n" + "=" * 104)
    print(f"[VP 연속성] {args.num_seeds}시드 동시측정  샘플 {len(A)}   (틱당 조준점 방향 변화, deg)")
    print(line(f"우리({args.ownship.replace('AIP_','').replace('.dll','')})", A))
    print(line(f"상대({args.target.replace('AIP_','').replace('.dll','')})", B))

    print("\n  거리 구간별 (근거리는 1/r로 커지는 게 정상이므로 반드시 나눠 본다)")
    print(f"  {'거리':>12} {'표본':>7} | {'우리중앙':>8} {'우리>15도%':>10} | "
          f"{'상대중앙':>8} {'상대>15도%':>10}")
    for lo, hi in [(0, 500), (500, 914), (914, 1500), (1500, 3000), (3000, 1e9)]:
        k = (D >= lo) & (D < hi)
        if k.sum() < 100:
            continue
        print(f"  {lo:>5.0f}~{min(hi,9999):<5.0f} {int(k.sum()):>7} | "
              f"{np.median(A[k]):>8.3f} {100*np.mean(A[k]>15):>10.2f} | "
              f"{np.median(B[k]):>8.3f} {100*np.mean(B[k]>15):>10.2f}")

    print("\n  스로틀 변화(틱당 절대변화) — 모드 전환의 또 다른 흔적")
    dA, dB = np.abs(np.diff(At)), np.abs(np.diff(Bt))
    print(f"    우리  중앙 {np.median(dA):.5f}  p99 {np.percentile(dA,99):.4f}  "
          f">0.1 {100*np.mean(dA>0.1):.3f}%")
    print(f"    상대  중앙 {np.median(dB):.5f}  p99 {np.percentile(dB,99):.4f}  "
          f">0.1 {100*np.mean(dB>0.1):.3f}%")

    print("\n  ➜ 판정: 같은 거리 구간에서 우리 쪽 큰 점프(>15도) 비율이 상대보다")
    print("     뚜렷이 높아야 '이산 전환의 불연속' 가설이 지지된다. 비슷하면 기각이다.")
    print("=" * 104)


if __name__ == "__main__":
    main()
