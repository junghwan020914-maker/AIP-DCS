"""무득점 판의 **국면 구조** 진단 — 공세/수세/교착 중 무엇인가.

`_zeroscore_diag.py`는 "접근 실패 vs 조준 실패"까지만 갈랐다. 그런데 그 둘은
증상이지 원인이 아니다. 같은 "조준 실패"라도 원인이 정반대일 수 있다:

  (a) 내가 뒤를 잡았는데 총구가 안 따라간다  -> 추종 정확도 문제 (제어기)
  (b) 상대가 내 뒤에 있어 방어중이다          -> 방어 BFM 문제
  (c) 둘 다 서로 못 잡고 같이 돈다(러프베리)  -> 대칭을 깨야 함 (수직기동/에너지)

(a)는 제어기 게인, (b)는 BreakTurn, (c)는 트리 국면 선택이 처방이다.
지금까지 무득점 판에 시도한 처방(거리정책·적응형 스로틀·D항)이 전부 실패한 것은
**어느 유형인지 모른 채 (a)만 가정하고 때렸기** 때문일 수 있다.

상태 규약: state = [N, E, D, roll, pitch, heading, speed]  (GeoMathUtil 기준)

출력: 시드별 국면 점유율 + 무득점/득점 그룹 비교.
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
FRONT, REAR = 50.0, 130.0     # 국면 분류 임계 (deg)


def classify(ma, ta):
    """ma=내 ATA(내 기수->상대), ta=상대 ATA(상대 기수->나)."""
    if ma < FRONT and ta > REAR:
        return "off"      # 내가 뒤 = 공세
    if ma > REAR and ta < FRONT:
        return "def"      # 상대가 뒤 = 수세
    if ma < FRONT and ta < FRONT:
        return "head"     # 서로 기수 맞댐
    return "neu"          # 중립 선회 / 러프베리


def wrap180(x):
    return (x + 180.0) % 360.0 - 180.0


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
        cnt = {"off": 0, "def": 0, "head": 0, "neu": 0}
        cnt_band = {"off": 0, "def": 0, "head": 0, "neu": 0}
        off_band_ata = []            # 공세+밴드일 때의 내 ATA -> 추종 정확도
        run = best_run = 0           # 공세+밴드 최장 연속 구간(스텝)
        corot = 0                    # 동방향 선회(러프베리) 스텝
        neu_band = 0
        d_sum = e_sum = 0.0          # 에너지 차 누적 (속도, 고도)
        prev_hdg = prev_thdg = None
        while not (terminated or truncated):
            _, _, terminated, truncated, _ = env.step(np.zeros(4, dtype=np.float32))
            step += 1
            t_s = step * DT
            g = env._geo_info
            os_, ts_ = env._ownship_state, env._target_state
            d = float(g._get_distance(os_, ts_))
            ma = abs(float(g._get_antenna_train_angle(os_, ts_, False)))
            ta = abs(float(g._get_antenna_train_angle(ts_, os_, False)))
            k = classify(ma, ta)
            cnt[k] += 1
            in_band = BAND_MIN <= d <= BAND_MAX
            if in_band:
                cnt_band[k] += 1
            if k == "off" and in_band:
                off_band_ata.append(ma)
                run += 1
                best_run = max(best_run, run)
            else:
                run = 0
            # 러프베리 판정: 중립 + 밴드 + 두 기체가 같은 방향으로 선회
            hdg, thdg = float(os_[5]), float(ts_[5])
            if prev_hdg is not None and k == "neu" and in_band:
                neu_band += 1
                r1 = wrap180(hdg - prev_hdg)
                r2 = wrap180(thdg - prev_thdg)
                if abs(r1) > 0.02 and abs(r2) > 0.02 and (r1 * r2) > 0:
                    corot += 1
            prev_hdg, prev_thdg = hdg, thdg
            d_sum += float(os_[6]) - float(ts_[6])          # 속도차 m/s
            e_sum += (-float(os_[2])) - (-float(ts_[2]))    # 고도차 m
            my_hp += score_rate(d, ma, t_s)[0] * DT
            th_hp += score_rate(d, ta, t_s)[0] * DT
        env.close()

        n = max(step, 1)
        oba = np.array(off_band_ata) if off_band_ata else np.array([180.0])
        rows.append(dict(
            seed=seed, my=my_hp, th=th_hp, zero=(my_hp <= 0.0), secs=step * DT,
            off=cnt["off"] / n * 100, dfn=cnt["def"] / n * 100,
            head=cnt["head"] / n * 100, neu=cnt["neu"] / n * 100,
            offb=cnt_band["off"] / 60.0, defb=cnt_band["def"] / 60.0,
            neub=cnt_band["neu"] / 60.0,
            oba_min=float(oba.min()), oba_med=float(np.median(oba)),
            best_run=best_run / 60.0,
            corot=(corot / neu_band * 100) if neu_band else 0.0,
            dv=d_sum / n, dalt=e_sum / n))
        r = rows[-1]
        mark = "무득점" if r["zero"] else "  득점"
        print(f"  seed {seed:>2} {mark} 내{r['my']:.3f} | 국면% 공{r['off']:4.1f} "
              f"수{r['dfn']:4.1f} 두{r['head']:4.1f} 중{r['neu']:4.1f} | "
              f"공세밴드 {r['offb']:5.1f}s(최장{r['best_run']:4.1f}s) ATA최소{r['oba_min']:5.1f} "
              f"중앙{r['oba_med']:5.1f} | 중립밴드{r['neub']:5.1f}s 동선회{r['corot']:4.0f}% | "
              f"dV{r['dv']:+5.1f} dH{r['dalt']:+6.0f}", flush=True)

    z = [r for r in rows if r["zero"]]
    s = [r for r in rows if not r["zero"]]

    def grp(rs, label):
        if not rs:
            print(f"  {label}: 없음")
            return
        f = lambda k: np.median([r[k] for r in rs])
        print(f"  {label:<7}(n={len(rs):>2}) 국면% 공{f('off'):4.1f} 수{f('dfn'):4.1f} "
              f"두{f('head'):4.1f} 중{f('neu'):4.1f} | 공세밴드 {f('offb'):5.1f}s "
              f"최장{f('best_run'):4.1f}s ATA최소{f('oba_min'):5.1f} 중앙{f('oba_med'):5.1f} | "
              f"중립밴드{f('neub'):5.1f}s 동선회{f('corot'):4.0f}% | "
              f"dV{f('dv'):+5.1f} dH{f('dalt'):+6.0f}")

    print("\n" + "=" * 108)
    print(f"[국면 구조 진단] {args.num_seeds}시드  나={args.ownship}  상대={args.target}")
    print(f"  무득점 {len(z)}판 / 득점 {len(s)}판   (중앙값)")
    grp(z, "무득점")
    grp(s, "득점")
    if z and s:
        ob_z, ob_s = np.median([r["offb"] for r in z]), np.median([r["offb"] for r in s])
        at_z = np.median([r["oba_min"] for r in z])
        co_z = np.median([r["corot"] for r in z])
        nb_z = np.median([r["neub"] for r in z])
        print()
        print(f"  ➜ 공세+밴드 {ob_z:.1f}s vs {ob_s:.1f}s")
        if ob_z < 3.0:
            print("     기회 자체가 없다 -> **위치/국면 문제**. 제어기 게인으로는 못 고친다.")
        elif at_z > 3.0:
            print("     기회는 있는데 총구가 안 붙는다 -> **추종 정확도 문제**(제어기).")
        else:
            print("     기회도 조준도 되는데 무득점 -> 거리계수/시간 문제.")
        print(f"  ➜ 무득점 판 중립밴드 {nb_z:.1f}s 중 동방향선회 {co_z:.0f}% "
              f"({'러프베리 교착' if co_z > 55 else '교착 아님'})")
    print("=" * 108)


if __name__ == "__main__":
    main()
