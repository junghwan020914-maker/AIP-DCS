"""무득점 판 원인 추적 — 승점을 좌우하는 진짜 병목.

08-10 판정 기준을 승점(규정 제6조2항: 200초 만료 시 잔여 HP 높은 쪽 승리)으로 바꾸자
드러난 사실: 30판 중 **19판에서 한 대도 못 넣는다**(v32 3, v29 6, arcA 9).
순이득 평균은 이기는 판들의 평균이라 이 판들이 묻혀 있었다.

무득점 판이 생기는 경로는 크게 둘이고 대응이 완전히 다르다:
  (A) **접근 실패** — 밴드(152~914m)에 거의 못 들어간다. 거리 문제.
  (B) **조준 실패** — 밴드에는 들어갔는데 ATA가 1도 안으로 안 들어온다. 각도 문제.
둘을 가르려면 득점 판과 무득점 판을 나눠 같은 지표로 비교해야 한다.

출력: 시드별 요약 + 득점/무득점 그룹 비교.
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ownship", default="AIP_DCS.dll")
    ap.add_argument("--target", required=True)
    ap.add_argument("--num-seeds", type=int, default=30)
    args = ap.parse_args()

    rows = []
    for seed in range(args.num_seeds):
        rng = np.random.default_rng(seed)
        own, tgt = make_state(rng)
        alt0, spd0 = -own[2], own[6]
        sep0 = float(np.hypot(tgt[0] - own[0], tgt[1] - own[1]))
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
        my_hp = th_hp = 0.0
        band = 0
        dmin = 1e9
        ata_in_band = []          # 밴드 안에서의 내 ATA
        ata_min_all = 180.0
        while not (terminated or truncated):
            _, _, terminated, truncated, _ = env.step(np.zeros(4, dtype=np.float32))
            step += 1
            t_s = step * DT
            g = env._geo_info
            d = float(g._get_distance(env._ownship_state, env._target_state))
            ma = abs(float(g._get_antenna_train_angle(
                env._ownship_state, env._target_state, False)))
            ta = abs(float(g._get_antenna_train_angle(
                env._target_state, env._ownship_state, False)))
            dmin = min(dmin, d)
            ata_min_all = min(ata_min_all, ma)
            if 152.4 <= d <= 914.4:
                band += 1
                ata_in_band.append(ma)
            my_hp += score_rate(d, ma, t_s)[0] * DT
            th_hp += score_rate(d, ta, t_s)[0] * DT
        env.close()

        ab = np.array(ata_in_band) if ata_in_band else np.array([180.0])
        rows.append(dict(
            seed=seed, my=my_hp, th=th_hp, zero=(my_hp <= 0.0),
            band=band / 60.0, dmin=dmin, secs=step * DT,
            ata_band_min=float(ab.min()), ata_band_p10=float(np.percentile(ab, 10)),
            ata_min_all=ata_min_all, alt0=alt0, spd0=spd0, sep0=sep0))
        r = rows[-1]
        mark = "무득점" if r["zero"] else "  득점"
        print(f"  seed {seed:>2} {mark}  내{r['my']:.4f} 상대{r['th']:.4f} | "
              f"밴드{r['band']:5.1f}s 최소거리{r['dmin']:6.0f}m | "
              f"밴드내ATA 최소{r['ata_band_min']:5.1f}도 p10 {r['ata_band_p10']:5.1f}도 | "
              f"시작 고도{r['alt0']:5.0f}m 거리{r['sep0']:.0f}m", flush=True)

    z = [r for r in rows if r["zero"]]
    s = [r for r in rows if not r["zero"]]

    def grp(rs, label):
        if not rs:
            print(f"  {label}: 없음")
            return
        f = lambda k: np.median([r[k] for r in rs])
        print(f"  {label:<8}(n={len(rs):>2})  밴드체류 {f('band'):5.1f}s  "
              f"최소거리 {f('dmin'):6.0f}m  밴드내ATA 최소 {f('ata_band_min'):5.1f}도  "
              f"p10 {f('ata_band_p10'):5.1f}도  전체ATA최소 {f('ata_min_all'):5.1f}도  "
              f"시작고도 {f('alt0'):5.0f}m")

    print("\n" + "=" * 100)
    print(f"[무득점 판 추적] {args.num_seeds}시드  나={args.ownship}  상대={args.target}")
    print(f"  무득점 {len(z)}판 / 득점 {len(s)}판   (중앙값 비교)")
    grp(z, "무득점")
    grp(s, "득점")
    if z and s:
        bz, bs = np.median([r["band"] for r in z]), np.median([r["band"] for r in s])
        az, as_ = np.median([r["ata_band_min"] for r in z]), np.median([r["ata_band_min"] for r in s])
        print()
        print(f"  ➜ 밴드 체류 비 {bz:.1f}s vs {bs:.1f}s  ({'접근 실패' if bz < bs*0.5 else '접근은 비슷'})")
        print(f"  ➜ 밴드내 최소ATA {az:.1f}도 vs {as_:.1f}도  "
              f"({'조준 실패' if az > 3.0 else '조준은 근접'})")
    print("=" * 100)


if __name__ == "__main__":
    main()
