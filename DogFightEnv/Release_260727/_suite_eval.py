"""상대 전수 평가 — 한 번에 여러 상대를 돌려 요약표를 낸다.

08-08 문제의식: 파라미터 스캔을 v32 단독으로 판정해왔다(기각 8건). 채택은 4상대
전수 검증을 했지만 기각은 안 했다. 기각도 과적합이다 — 다른 상대에겐 도움이 될
변경을 버렸을 수 있다. **상대마다 따로 실행해야 하는 불편이 그 관행을 만들었으므로
도구로 없앤다.**

⚠️ 시작 기하는 08-08에 정정됐다: 2000/2500/3000ft는 **기체 간 거리**이고 두 기체는
서로의 3-9 라인(날개 축)에 나란히 놓여 **반대 방향**을 본다. 헤드온이 아니다.
이전 헤드온 구현은 시작 즉시 서로 WEZ 안이라 Phase1 득점이 과대계상됐다.

🔴 08-10 이 도구의 사각지대 — **반드시 같이 읽을 것.**
여기 상대는 전부 "제대로 기동하는 BT"다(ryujan v29/v32, 자작 arcA/arcE, 이전 세대).
그래서 **퇴화 상대에 대한 실패를 원리상 못 잡는다.** 실제로 승점 144.5/150(96.3%)인
상태에서 `_degenerate_probe.py`로 재보니 0.70 스로틀로 직진 수평비행하는 표적을
**6시드 중 4판에서 200초 내내 못 잡았다**(시작거리에서 1m도 못 좁힌 판 다수).
전추력 수평비행 최고속도는 505m/s인데 추격 실속도가 421m/s로, 조준 진동에
에어프레임 성능의 17%를 버리고 있었다. 이 손실은 실전 교전에도 계속 작용한다.

→ 채택 판정 때는 **`_degenerate_probe.py --kinds duck`도 같이 돌릴 것.**
   (runner는 스로틀 1.0 동등기체 정미익추격이라 못 잡는 게 물리적으로 정상 — 무시)

사용:
    python _suite_eval.py --targets AIP_v32.dll,AIP_v29.dll,AIP_arcA.dll,AIP_arcE.dll
    python _suite_eval.py --ownship AIP_cand.dll --preset all --num-seeds 30
    python _degenerate_probe.py --ownship AIP_cand.dll --kinds duck --num-seeds 6
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
    # 08-10: 상대 추락도 센다. prev 상대 4승14패를 "한쪽만 안전장치를 건 비대칭"으로
    # 해석했으나 우리 추락만 세고 있어 **검증되지 않은 상태**였다. 규정상 추락은 즉시
    # 패배이므로 상대가 자멸하는 판은 실제로는 우리 승리다.
    tgt_crash = 0
    secs_total = 0.0
    # 08-10: 규정 제6조2항 — 200초 만료 시 **잔여 HP가 높기만 하면 승리**다.
    # 즉 규칙이 보상하는 것은 마진의 **크기**가 아니라 **부호**다. 평균 순이득만 보면
    # "이미 압도하는 매치업을 더 압도하는" 변경이 통과하고, 정작 점수를 좌우하는
    # "마진이 0 근처인 판"은 안 보인다. 판별 시드와 무득점 판을 따로 센다.
    zero_scored = 0
    margins = []

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
        tgt_min_alt = 1e9
        seed_my = seed_th = 0.0
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
            tgt_min_alt = min(tgt_min_alt, -float(env._target_state[2]))
            if 152.4 <= d <= 914.4 and ma <= 1.0:
                cone1 += 1
            r, ph = score_rate(d, ma, t_s)
            my_hp += r * DT
            seed_my += r * DT
            if ph:
                my_phase[ph] += r * DT
            r2, _ = score_rate(d, ta, t_s)
            th_hp += r2 * DT
            seed_th += r2 * DT

        secs_total += step * DT
        margins.append(seed_my - seed_th)
        if seed_my <= 0.0:
            zero_scored += 1
        if my_min_alt < 300.0:
            crash += 1
        if tgt_min_alt < 300.0:
            tgt_crash += 1
        # 08-10: 규정 제6조1항 — **추락(고도 300m 이하)은 즉시 패배**다. HP 비교보다 우선한다.
        # 기존엔 우리 추락만 세고 승패엔 반영하지 않아, 상대가 자멸한 판을 무승부/패배로
        # 집계하고 있었다(leadout 실험에서 arcA가 11판 자멸하는 것이 드러남).
        oh = float(info.get("ownship_health", 1.0))
        thh = float(info.get("target_health", 1.0))
        my_crashed = my_min_alt < 300.0
        tgt_crashed = tgt_min_alt < 300.0
        if my_crashed and not tgt_crashed:
            losses += 1
        elif tgt_crashed and not my_crashed:
            wins += 1
        elif my_crashed and tgt_crashed:
            pass                      # 양쪽 추락 -> 무승부 취급
        elif thh < oh:
            wins += 1
        elif oh < thh:
            losses += 1
        env.close()

    tot = max(my_hp, 1e-9)
    return dict(target=target, w=wins, l=losses, n=n, draws=n - wins - losses,
                zero=zero_scored, margins=margins,
                my=my_hp / n, th=th_hp / n, net=(my_hp - th_hp) / n,
                cone1=cone1 / 60 / n, crash=crash, tgt_crash=tgt_crash,
                secs=secs_total / n,
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
          f"{'<1도s':>6} {'추락내/상':>5} {'교전s':>6} | {'P1%':>5} {'P2%':>5} {'P3%':>5}")
    for r in rows:
        print(f"  {r['target']:<16} {r['w']:>3} {r['l']:>3} | {r['my']:>8.4f} {r['th']:>8.4f} "
              f"{r['net']:>+9.4f} | {r['cone1']:>6.2f} {r['crash']:>2}/{r['tgt_crash']:<2} {r['secs']:>6.1f} | "
              f"{r['p1']:>5.1f} {r['p2']:>5.1f} {r['p3']:>5.1f}")
    if rows:
        nets = np.array([r["net"] for r in rows])
        print(f"\n  순이득 평균 {nets.mean():+.4f}   최저 {nets.min():+.4f}"
              f"({rows[int(nets.argmin())]['target']})   "
              f"총 추락 내 {sum(r['crash'] for r in rows)}판 / 상대 {sum(r['tgt_crash'] for r in rows)}판")
        print("  ※ 채택 판정은 **최저값**을 본다 — 평균만 보면 한 상대에 과적합된 변경을 통과시킨다.")
        print()
        print("  [규정 제6조2항] 200초 만료 시 **잔여 HP가 높기만 하면 승리** — 마진의 크기가")
        print("  아니라 **부호**가 점수를 만든다. 예선 단판제 기준 승1.0 / 무0.5 / 패0.")
        print(f"  {'상대':<16} {'승점':>7} {'/만점':>6} | {'무':>3} {'무득점판':>8} | "
              f"{'마진 최소':>10} {'0.01미만':>9}")
        for r in rows:
            pts = r["w"] + 0.5 * r["draws"]
            m = np.array(r["margins"])
            near = int((m < 0.01).sum())
            print(f"  {r['target']:<16} {pts:>7.1f} {r['n']:>6} | {r['draws']:>3} {r['zero']:>8} | "
                  f"{m.min():>+10.4f} {near:>9}")
        allpts = sum(r["w"] + 0.5 * r["draws"] for r in rows)
        alln = sum(r["n"] for r in rows)
        print(f"  → 총 승점 {allpts:.1f} / {alln}  ({100*allpts/alln:.1f}%)")
    print("=" * 96)


if __name__ == "__main__":
    main()
