"""상대 전수 평가 — 한 번에 여러 상대를 돌려 요약표를 낸다.

08-08 문제의식: 파라미터 스캔을 v32 단독으로 판정해왔다(기각 8건). 채택은 4상대
전수 검증을 했지만 기각은 안 했다. 기각도 과적합이다 — 다른 상대에겐 도움이 될
변경을 버렸을 수 있다. **상대마다 따로 실행해야 하는 불편이 그 관행을 만들었으므로
도구로 없앤다.**

⚠️ 시작 기하는 08-08에 정정됐다: 2000/2500/3000ft는 **기체 간 거리**이고 두 기체는
서로의 3-9 라인(날개 축)에 나란히 놓여 **반대 방향**을 본다. 헤드온이 아니다.
이전 헤드온 구현은 시작 즉시 서로 WEZ 안이라 Phase1 득점이 과대계상됐다.

사용:
    python _suite_eval.py --targets AIP_v32.dll,AIP_v29.dll,AIP_arcA.dll,AIP_arcE.dll
    python _suite_eval.py --ownship AIP_cand.dll --preset all --num-seeds 30
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

from _aim_time_probe import PHASES, DT, make_state, score_rate  # noqa: E402
from DogFightEnvWrapper import DogFightWrapper  # noqa: E402
from dogfight.ai.bt_action_provider import BTActionProvider  # noqa: E402

PRESETS = {
    # 기본: 현역 최강(v32) + 차강(v29) + 자체 아키타입 양극단
    # 08-08: 자기 직전 세대(AIP_prev)를 회귀 감시자로 추가.
    # ⚠️ 역할 구분 — prev는 "예전 나보다 나아졌나"만 답한다. 같은 혈통이라 같은 맹점을
    #    공유하므로(우리가 최대선회형에 약하면 prev도 약하다) **여기서 이겼다고 통과가
    #    아니다.** 채택 관문은 어디까지나 다양한 상대의 **최저값**이다.
    #    또 추이성이 성립하지 않는다 — "신형>구형"이 "신형>제3자"를 뜻하지 않는다(arcA 실측).
    # ⚠️ prev는 반드시 **전용 XML**(Rule_prev.xml)을 읽도록 빌드할 것. Rule_mine.xml을
    #    공유하면 트리 실험 때 상대 트리까지 같이 바뀌어 비교가 무효가 된다
    #    (ryujan이 이 충돌로 스파링 상대가 이틀간 죽어 있었다).
    "core": ["AIP_v32.dll", "AIP_v29.dll", "AIP_arcA.dll", "AIP_arcE.dll", "AIP_prev.dll"],
    # 전체: 위 + 구버전 계열 + 팀원 yuno
    "all": ["AIP_v32.dll", "AIP_v29.dll", "AIP_arcA.dll", "AIP_arcE.dll",
            "AIP_v7.dll", "AIP_v6.dll", "AIP_v5.dll", "AIP_v1.dll",
            "AIP_v0.dll", "AIP_yuno.dll", "AIP_prev.dll"],
}


def run_one(ownship: str, target: str, n: int) -> dict:
    my_hp = th_hp = 0.0
    my_phase = {1: 0.0, 2: 0.0, 3: 0.0}
    cone1 = 0
    wins = losses = 0
    crash = 0
    secs_total = 0.0

    for seed in range(n):
        rng = np.random.default_rng(seed)
        own, tgt = make_state(rng)
        env = DogFightWrapper(
            env_config={
                "observation_mode": "tactical16", "ownship_control_mode": "rl",
                "target_mode": "rl", "max_engage_time": 200.0, "min_altitude": 300.0,
                "ownship": own, "target": tgt,
                "initial_scenario": {"mode": "default"},
            },
            ownship_action_provider=BTActionProvider(dll_name=ownship),
            target_action_provider=BTActionProvider(dll_name=target),
        )
        env.reset(seed=seed)
        terminated = truncated = False
        info = {}
        step = 0
        my_min_alt = 1e9
        while not (terminated or truncated):
            _, _, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
            step += 1
            t_s = step * DT
            g = env._geo_info
            d = float(g._get_distance(env._ownship_state, env._target_state))
            ma = abs(float(g._get_antenna_train_angle(
                env._ownship_state, env._target_state, False)))
            ta = abs(float(g._get_antenna_train_angle(
                env._target_state, env._ownship_state, False)))
            my_min_alt = min(my_min_alt, -float(env._ownship_state[2]))
            if 152.4 <= d <= 914.4 and ma <= 1.0:
                cone1 += 1
            r, ph = score_rate(d, ma, t_s)
            my_hp += r * DT
            if ph:
                my_phase[ph] += r * DT
            r2, _ = score_rate(d, ta, t_s)
            th_hp += r2 * DT

        secs_total += step * DT
        if my_min_alt < 300.0:
            crash += 1
        oh = float(info.get("ownship_health", 1.0))
        thh = float(info.get("target_health", 1.0))
        if thh < oh:
            wins += 1
        elif oh < thh:
            losses += 1
        env.close()

    tot = max(my_hp, 1e-9)
    return dict(target=target, w=wins, l=losses, n=n,
                my=my_hp / n, th=th_hp / n, net=(my_hp - th_hp) / n,
                cone1=cone1 / 60 / n, crash=crash, secs=secs_total / n,
                p1=100 * my_phase[1] / tot, p2=100 * my_phase[2] / tot,
                p3=100 * my_phase[3] / tot)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ownship", default="AIP_DCS.dll")
    ap.add_argument("--targets", default="")
    ap.add_argument("--preset", choices=sorted(PRESETS), default="core")
    ap.add_argument("--num-seeds", type=int, default=30)
    args = ap.parse_args()

    targets = ([t.strip() for t in args.targets.split(",") if t.strip()]
               if args.targets else PRESETS[args.preset])

    rows = []
    for t in targets:
        if not (ROOT / t).exists():
            print(f"  (건너뜀, 파일 없음) {t}", flush=True)
            continue
        r = run_one(args.ownship, t, args.num_seeds)
        rows.append(r)
        print(f"  {t:<16} 완료  순이득 {r['net']:+.4f}  {r['w']}승{r['l']}패", flush=True)

    print("\n" + "=" * 96)
    print(f"[전수 평가] {args.num_seeds}시드/상대   나={args.ownship}   기하=어빔(3-9라인)+반대방향")
    print(f"  {'상대':<16} {'승':>3} {'패':>3} | {'내득점':>8} {'피격':>8} {'순이득':>9} | "
          f"{'<1도s':>6} {'추락':>4} {'교전s':>6} | {'P1%':>5} {'P2%':>5} {'P3%':>5}")
    for r in rows:
        print(f"  {r['target']:<16} {r['w']:>3} {r['l']:>3} | {r['my']:>8.4f} {r['th']:>8.4f} "
              f"{r['net']:>+9.4f} | {r['cone1']:>6.2f} {r['crash']:>4} {r['secs']:>6.1f} | "
              f"{r['p1']:>5.1f} {r['p2']:>5.1f} {r['p3']:>5.1f}")
    if rows:
        nets = np.array([r["net"] for r in rows])
        print(f"\n  순이득 평균 {nets.mean():+.4f}   최저 {nets.min():+.4f}"
              f"({rows[int(nets.argmin())]['target']})   "
              f"총 추락 {sum(r['crash'] for r in rows)}판")
        print("  ※ 채택 판정은 **최저값**을 본다 — 평균만 보면 한 상대에 과적합된 변경을 통과시킨다.")
    print("=" * 96)


if __name__ == "__main__":
    main()
