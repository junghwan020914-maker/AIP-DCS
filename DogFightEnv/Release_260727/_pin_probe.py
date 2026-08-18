"""**눌림 구간**을 해부한다 — 뒤를 잡힌 동안 우리는 무엇을 하고 있었나.

08-16 배경. 서버 초기조건 20시드 패널에서 v42전이 이렇게 나왔다:

    우리 vs v42   뒤잡힘 68.1s/판  최장연속 **84.6s**  뒤잡음 **0.0s**   승점 7.5
    yuno vs v42   뒤잡힘  0.0s     최장연속   0.0s     뒤잡음  0.0s     승점 18.0

같은 상대인데 우리만 판의 42%를 눌려 있고 한 번도 뒤를 못 잡는다. 실서버 3파전
결론(병목은 조준이 아니라 자리 지키기)과 같은 그림이다.

## 무엇을 가려야 하나 — 세 이야기, 처방이 전부 다르다

  (A) **에너지**   눌림 진입 시점에 이미 느리고/낮아서 못 벗어난다
                   -> 처방은 진입 전 에너지 관리
  (B) **제어 권한** 최대로 당기는데 선회율이 안 나온다
                   -> 처방은 제어기(Roll_Effect가 피치를 죽이는 구간 등)
  (C) **판단**     애초에 최대로 안 당긴다. 트리가 다른 걸 하고 있다
                   -> 처방은 트리 분기

⚠️ 판정 기준을 데이터 보기 전에 박는다. 핵심 지표는 **암시 하중배수**다:

    n = sqrt( (omega * V / g)^2 + 1 )      omega=실측 선회율, V=실측 속도

  · 눌림 중 n 중앙 >= 6      -> 최대로 당기고 있다. (A) 또는 (B)
      - 그때 속도가 코너속도 밖이면 **(A) 에너지**
      - 코너속도 안인데도 못 벗어나면 **(B) 제어 권한** 또는 물리적 한계
  · 눌림 중 n 중앙 <= 4      -> **(C) 판단**. 당길 수 있는데 안 당기고 있다
  · 4 < n < 6                -> 애매. 구간을 나눠 다시 볼 것

대조군을 같이 돌린다. 눌림이 없는 상대(arcD/v0)에서 같은 지표가 어떻게 다른지
봐야 "눌림 중이라서 그런 것"인지 "우리 기본 성향"인지 갈린다.

사용:
    python _pin_probe.py --target AIP_ryujan_v42.dll --control AIP_v0.dll --num-seeds 6
Not part of the student template; safe to delete after use.
"""
from __future__ import annotations
import argparse, sys
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parent
for p in (ROOT, ROOT / "src"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
from _aim_time_probe import DT  # noqa: E402
from _suite_eval import server_state  # noqa: E402
from DogFightEnvWrapper import DogFightWrapper  # noqa: E402
from dogfight.ai.bt_action_provider import BTActionProvider  # noqa: E402

G = 9.80665
BAND = (152.4, 914.4)
H = 12  # 0.2s 창


def fwd(roll, pitch, yaw):
    p, y = np.radians([pitch, yaw])
    return np.array([np.cos(p) * np.cos(y), np.cos(p) * np.sin(y), np.sin(p)])


def run(own, tgt, n, off):
    ep = []
    for seed in range(off, off + n):
        rng = np.random.default_rng(seed)
        o0, t0 = server_state(rng)
        env = DogFightWrapper(
            env_config={"observation_mode": "tactical16", "ownship_control_mode": "rl",
                        "target_mode": "rl", "max_engage_time": 200.0, "min_altitude": 300.0,
                        "ownship": o0, "target": t0, "initial_scenario": {"mode": "default"}},
            ownship_action_provider=BTActionProvider(dll_name=own),
            target_action_provider=BTActionProvider(dll_name=tgt))
        env.reset(seed=seed)
        F, SP, AL, MA, TA, D = [], [], [], [], [], []
        term = trunc = False
        while not (term or trunc):
            _, _, term, trunc, _ = env.step(np.zeros(4, dtype=np.float32))
            g, o, t = env._geo_info, env._ownship_state, env._target_state
            F.append(fwd(float(o[3]), float(o[4]), float(o[5])))
            SP.append(float(o[6])); AL.append(-float(o[2]))
            D.append(float(g._get_distance(o, t)))
            MA.append(abs(float(g._get_antenna_train_angle(o, t, False))))
            TA.append(abs(float(g._get_antenna_train_angle(t, o, False))))
        env.close()
        F = np.array(F); SP = np.array(SP); AL = np.array(AL)
        MA = np.array(MA); TA = np.array(TA); D = np.array(D)
        c = np.clip(np.einsum('ij,ij->i', F[:-H], F[H:]), -1, 1)
        OM = np.concatenate([np.degrees(np.arccos(c)) / (H * DT), np.full(H, np.nan)])
        pinned = (D >= BAND[0]) & (D <= BAND[1]) & (TA < 50) & (MA > 130)
        i = 0
        while i < len(pinned):
            if not pinned[i]:
                i += 1; continue
            j = i
            while j < len(pinned) and pinned[j]:
                j += 1
            if (j - i) * DT >= 5.0:
                s = slice(i, j)
                pre = slice(max(0, i - int(10 / DT)), i)
                ep.append(dict(seed=seed, t0=i * DT, dur=(j - i) * DT,
                               om=np.nanmedian(OM[s]), sp=np.median(SP[s]),
                               al=np.median(AL[s]), d=np.median(D[s]),
                               ma=np.median(MA[s]),
                               sp_pre=(np.median(SP[pre]) if pre.stop > pre.start else np.nan),
                               al_pre=(np.median(AL[pre]) if pre.stop > pre.start else np.nan),
                               om_pre=(np.nanmedian(OM[pre]) if pre.stop > pre.start else np.nan)))
            i = j
        print(f"  seed {seed:>2} 눌림 {sum(1 for e in ep if e['seed']==seed)}구간", flush=True)
    return ep


def rep(nm, ep):
    if not ep:
        print(f"\n[{nm}] 5초 이상 눌림 구간 없음")
        return
    dur = np.array([e['dur'] for e in ep]); om = np.array([e['om'] for e in ep])
    sp = np.array([e['sp'] for e in ep]); al = np.array([e['al'] for e in ep])
    nz = np.sqrt((np.radians(om) * sp / G) ** 2 + 1)
    spp = np.array([e['sp_pre'] for e in ep]); omp = np.array([e['om_pre'] for e in ep])
    print(f"\n[{nm}] 눌림 구간 {len(ep)}개  총 {dur.sum():.1f}s  최장 {dur.max():.1f}s")
    print(f"   눌림 중  선회율 {np.median(om):6.2f}도/s   속도 {np.median(sp):6.1f}m/s   "
          f"고도 {np.median(al):6.0f}m   거리 {np.median([e['d'] for e in ep]):5.0f}m")
    print(f"            **암시 하중배수 n = {np.median(nz):.2f}**  "
          f"(p90 {np.percentile(nz,90):.2f}, 최대 {nz.max():.2f})")
    print(f"   진입 10초 전  속도 {np.nanmedian(spp):6.1f}m/s  선회율 {np.nanmedian(omp):6.2f}도/s")
    print(f"   진입 전->중 속도 변화 {np.nanmedian(sp - spp):+6.1f} m/s")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ownship", default="AIP_DCS.dll")
    ap.add_argument("--target", required=True)
    ap.add_argument("--control", default="")
    ap.add_argument("--num-seeds", type=int, default=6)
    ap.add_argument("--seed-offset", type=int, default=0)
    a = ap.parse_args()
    print("=" * 92)
    print(f"[눌림 해부] 나={a.ownship}  {a.num_seeds}시드  서버 초기조건(15000ft/2000ft/200m/s)")
    print(f"  판정: n>=6 최대로 당김(에너지/권한) | n<=4 안 당김(판단) | 4<n<6 애매")
    rep(a.target, run(a.ownship, a.target, a.num_seeds, a.seed_offset))
    if a.control:
        rep(f"{a.control} (대조군)", run(a.ownship, a.control, a.num_seeds, a.seed_offset))
    print("=" * 92)


if __name__ == "__main__":
    main()
