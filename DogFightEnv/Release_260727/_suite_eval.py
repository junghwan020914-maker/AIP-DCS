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

🔴 08-10 두 번째 주의 — **트리 변경은 기준선도 같은 C++로 재측정해서 비교할 것.**
`AIP_prev.dll`이 자기 XML 없이 `./Rule_mine.xml`을 읽고 있어서, 지금까지 "직전 세대"
대조가 사실은 **현재 트리 + 옛 C++** 였다(수정 완료 — 이제 `Rule_prev.xml`을 읽는다).
그래서 옛 기준선 수치에는 C++ 드리프트가 섞여 있었다. 같은 트리를 현재 C++로 다시
재니 v32 28.5 -> 27.5, v29 26.5 -> 26.0 이었다 — **1.5점쯤이 트리와 무관한 차이**다.
`EnergyTurn` 채택도 이 방식으로 재확인했다(같은 C++, 같은 4상대: 113.5 vs 115.5).

🔴 세 번째 주의 — **헤드투헤드와 다상대 성적은 비이행적이다.**
`EnergyTurn` 트리는 직전 트리와 직접 붙으면 3승 5패로 지는데, 공통 상대군에서는
+2.0으로 앞선다. 대회는 미지의 40상대와 싸우므로 **다상대 지표로 판정**한다.
`prev` 상대 성적이 나쁘다는 이유만으로 기각하지 말 것(좋다고 채택하지도 말 것).

🔴🔴 08-10 **가장 중요한 주의 — 인샘플 이득의 약 1/6만 전이된다.**
지금까지 모든 튜닝이 시드 0~29에서 이루어졌다. `--seed-offset 30`으로 처음 재보니:

    구성              시드 0~29   시드 30~59
    오늘 채택본         116.5       111.0
    오늘 이전 트리      113.5       110.5
    **채택 이득**      **+3.0**    **+0.5**

시드 30~59는 **실제로 더 어렵다**(이전 트리도 -3.0). 그러니 하락 자체는 과적합이 아니다.
그러나 **채택으로 얻은 이득이 +3.0 -> +0.5로 줄어든다.** 나머지는 시드 적합이었다.
또 인샘플에서 0이던 패배가 아웃오브샘플에서는 4~5판 나온다 — **"패배 0"은 인샘플 성질이다.**

➡️ **채택 기준을 이에 맞춰 쓸 것:**
   - 30시드에서 ±1~2점 차이는 **실력 차이가 아니라 시드 적합일 가능성이 높다.**
   - 채택 전 `--seed-offset 30`으로 **홀드아웃 검증**을 할 것.
   - 구조적 수정(버그·결함)은 전이되지만, **게이트값 튜닝은 잘 전이되지 않는다.**

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

from _aim_time_probe import FEET, PHASES, DT, make_state, score_rate  # noqa: E402
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
    # 08-10 신설 원형 2종. `core`에 넣지 않고 별도 프리셋으로 둔다 — core 수치는 오랜
    # 비교 이력(144.5 등)이 걸려 있어 상대를 추가하면 과거와 대조가 끊긴다.
    #   arcD 순수 방어형 : 절대 공격하지 않으므로 **질 수 없다.** 점수가 오직
    #                      "움직이는 표적을 잡을 수 있는가"만 측정한다(기준선 10.0/10).
    #   arcV 수직 파이터 : **우리 최악의 매치업**(기준선 6.0/10, e3 6.5/10).
    #                      10판 전부 중립 98%로 교전 자체가 성립하지 않는다.
    #                      트리에 수직 추격 분기가 없다는 구조적 공백 때문이다.
    "arc": ["AIP_arcA.dll", "AIP_arcE.dll", "AIP_arcD.dll", "AIP_arcV.dll"],
    # 전체: 위 + 구버전 계열 + 팀원 yuno
    "all": ["AIP_v32.dll", "AIP_v29.dll", "AIP_arcA.dll", "AIP_arcE.dll",
            "AIP_v7.dll", "AIP_v6.dll", "AIP_v5.dll", "AIP_v1.dll",
            "AIP_v0.dll", "AIP_yuno.dll", "AIP_prev.dll"],
}


# 🔴🔴 08-13 실서버 실측 — **뷰어는 Phase 2/3을 주지 않는다. Phase 1 전용이다.**
#
# 앞서 이 자리에 "서버가 데미지를 프레임당 누산한다(약 60배)"고 적었다. **그건 틀렸다.**
# 원인은 단위 혼동이었다 — 로컬은 체력을 0~1로 정규화해 쓰고(_init_health=1) 실서버는
# 같은 값을 HP 0~100으로 표시한다. 즉 **로컬 피해량 1.0 = HP 100 = 격추**이고 배율은 없다.
#
# 실서버 4판을 Phase 1(ATA<1도, 152~914m)만으로 재계산하니 전부 맞는다:
#     판     Phase1 피해(경기시계)  조준시간   실제 결과
#     arcE        1.0231           4.34s     격추
#     v32         0.8020           1.75s     격추
#     v29         0.0000           0.00s     100/100 무승부
#     v42         0.0000           0.00s     100/100 무승부
# v29 판이 결정적이다 — 우리 ATA가 한 번도 1도 안에 못 들어갔고(최소 1.5도) Phase 1이
# 정확히 0인데, Phase 2/3을 포함한 우리 채점기는 0.2667을 계산했다. **화면은 100/100이었다.**
#
# ⚠️ 규정 Q&A/PPT에는 Phase 2/3이 분명히 있다. 우리가 쓰는 뷰어(V1.1_VeryLow, 08-06 배포)가
#    구현하지 않았을 뿐이고, **대회 본서버가 어느 쪽인지는 모른다.** 그래서 버리지 않고
#    **둘 다 계산해 나란히 보여준다.** 갈리는 상대가 나오면 그게 곧 위험 신호다.
#
# 이게 왜 중요한가 — 우리 득점의 상당 부분이 Phase 2/3에서 나온다:
#     v29  P1 59.2% / P2 31.3% / P3 9.4%   <- 40%가 뷰어에선 0점
#     v42  P1 81.6% / P2 10.7% / P3 7.7%
# 그 위에서 하루 종일 ±1~2점으로 채택/기각을 판정했다.
#
# 📊 08-13 두 모델 첫 대조 (6상대 x 20시드, AIP_DCS.dll = md5 99c1a200)
#     상대    규정(P1+P2+P3)  뷰어(P1만)  차이   격추   **피격추**
#     arcE        20.0          20.0       0    20/20      0
#     arcA        20.0          20.0       0    14/20      0
#     v29         19.5          18.5     -1.0     2        0
#     v32         18.5          17.5     -1.0     2        0
#     v42          8.0           9.5    **+1.5**  0        0
#     prev         7.0           6.5     -0.5     4      **7**
#     합계        93.0        **92.0/120 (76.7%)**       42        7
#   총합은 거의 안 변하지만 **6상대 중 4상대의 판정이 갈린다**(최대 ±1.5).
#
#   여기서 **오늘 처음 보이게 된 것 둘**:
#     · 우리는 실제로 격추한다 — 120판에서 42번(arcE 20/20 평균70초, arcA 14/20 평균82초).
#       실서버 arcE 격추(67초)와 로컬 예측(70초)이 2초 차이로 맞물린다.
#     · **우리는 prev에게 20판 중 7번 격추당한다.** 이전 척도로는 피격이 늘 0.0000
#       근처라 방어를 볼 이유가 없어 보였다. prev는 우리 혈통이므로 이전 어떤 변경이
#       특정 약점을 만들었다는 신호다. **최저값이 prev 6.5이므로 여기가 1순위다.**
#     · v42는 뷰어 기준으로 오히려 덜 나쁘다(8.0->9.5, 무 12->17). 죽지도 죽이지도
#       않는 순수 교착이라 급하지 않다.



# 🔴 08-15 실서버 실측 — **뷰어는 초기조건을 고정한다.** 5판이 소수점까지 같았다:
#     고도 4572.0m(15000ft)  이격 608.6m(2000ft)  속도 200.0m/s  yaw 90/-90
# 반면 `make_state`는 고도 2000~30000ft, 속도 200~300m/s를 흔든다. 특히 **속도 200은
# 우리 로컬 범위의 최하단**이고, 선회율이 g*sqrt(Nz^2-1)/V라 저속일수록 잘 돈다.
# 즉 실서버는 로컬 평균보다 훨씬 조이는 선회전이다 — 로컬이 낙관적이었던 유력한 원인.
# heading만 흔든다: 평평한 지구·무풍이라 물리적으로 무관해야 하고, 그래서 남는 차이는
# 곧 좌우/방위 비대칭이다(제어기 롤 비대칭 버그 전력이 있어 감시 가치가 있다).
SERVER_ALT_M = 15000.0 * FEET
SERVER_SEP_M = 2000.0 * FEET
SERVER_SPD = 200.0


def server_state(rng):
    hdg = float(rng.uniform(0.0, 360.0))
    half = SERVER_SEP_M / 2.0
    prad = np.deg2rad(hdg + 90.0)
    pn, pe = np.cos(prad) * half, np.sin(prad) * half
    return ([-pn, -pe, -SERVER_ALT_M, 0.0, 0.0, hdg, SERVER_SPD],
            [pn, pe, -SERVER_ALT_M, 0.0, 0.0, (hdg + 180.0) % 360.0, SERVER_SPD])


INIT_MODE = "random"        # "random" = make_state(기존), "server" = 실서버 고정조건


def _make(rng):
    return server_state(rng) if INIT_MODE == "server" else make_state(rng)


def run_one(ownship: str, target: str, n: int, off: int = 0) -> dict:
    my_hp = th_hp = 0.0
    my_phase = {1: 0.0, 2: 0.0, 3: 0.0}
    cone1 = 0
    wins = losses = 0
    crash = 0
    # 실서버 모델 집계
    s_wins = s_losses = 0
    s_kills = s_deaths = 0          # 200초 안에 HP를 다 깎은/깎인 판
    s_kill_times = []               # 우리가 격추한 시각(초)
    s_death_times = []
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
    # 🔴 08-15 실서버 3파전으로 확정된 진짜 병목 — **상대가 우리 뒤에 눌러앉은 시간**.
    # yuno vs v44는 양쪽 다 0.0초인데(서로 자리를 안 내준다) 둘 다 우리에게는 40~50초씩
    # 끊김 없이 눌러앉았다. 셋 중 자리를 못 지키는 건 우리뿐이었다.
    # 지금까지 채점기는 **우리 득점만** 보고 이걸 안 봤다. 공세 = 밴드 안에서
    # 내ATA<50 & 적ATA>130 (`_symmetry_probe.py`와 동일 정의).
    pin_them = 0          # 상대가 우리 뒤에 있은 틱
    pin_me = 0            # 우리가 상대 뒤에 있은 틱
    pin_them_max = 0.0    # 그 중 최장 연속(초) — 총량보다 이게 승패를 가른다
    pin_me_max = 0.0

    for seed in range(off, off + n):
        rng = np.random.default_rng(seed)
        own, tgt = _make(rng)
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
        run_them = run_me = 0
        srv_my = srv_th = 0.0            # Phase1만 누산 (뷰어 실측 모델). 1.0 = HP 100 = 격추
        srv_kill_t = srv_death_t = None  # 먼저 체력을 소진시킨 시각
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
            inb = 152.4 <= d <= 914.4
            if inb and ta < 50.0 and ma > 130.0:       # 상대가 우리 뒤
                pin_them += 1
                run_them += 1
                if run_them * DT > pin_them_max:
                    pin_them_max = run_them * DT
            else:
                run_them = 0
            if inb and ma < 50.0 and ta > 130.0:       # 우리가 상대 뒤
                pin_me += 1
                run_me += 1
                if run_me * DT > pin_me_max:
                    pin_me_max = run_me * DT
            else:
                run_me = 0
            r, ph = score_rate(d, ma, t_s)
            my_hp += r * DT
            seed_my += r * DT
            if ph:
                my_phase[ph] += r * DT
            r2, ph2 = score_rate(d, ta, t_s)
            th_hp += r2 * DT
            seed_th += r2 * DT

            # 뷰어 실측 모델: **Phase 1만** 인정하고 정규화 체력 1.0 소진 시 격추.
            # 배율은 없다 — 로컬 1.0 = 실서버 HP 100이다(위 헤더 참고).
            if ph == 1:
                srv_my += r * DT
            if ph2 == 1:
                srv_th += r2 * DT
            if srv_kill_t is None and srv_my >= 1.0:
                srv_kill_t = t_s
            if srv_death_t is None and srv_th >= 1.0:
                srv_death_t = t_s

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

        # --- 실서버 모델 판정 ---
        # rules 모델과 다른 점은 **격추가 판을 끝낸다**는 것 하나다(부호 판정은 60배를
        # 곱해도 같으므로 동일). 그래서 "먼저 100을 채운 쪽"이 갈리는 판만 결과가 바뀐다.
        # ⚠️ 격추 이후 궤적은 실제로는 존재하지 않는다(시뮬은 계속 돈다). 승패 판정에만
        #    쓰고, 격추 후 구간의 피해량은 무시한다.
        if my_crashed and not tgt_crashed:
            s_losses += 1
        elif tgt_crashed and not my_crashed:
            s_wins += 1
        elif my_crashed and tgt_crashed:
            pass
        elif srv_kill_t is not None or srv_death_t is not None:
            k = srv_kill_t if srv_kill_t is not None else 1e18
            dth = srv_death_t if srv_death_t is not None else 1e18
            if k < dth:
                s_wins += 1
                s_kills += 1
                s_kill_times.append(k)
            elif dth < k:
                s_losses += 1
                s_deaths += 1
                s_death_times.append(dth)
        elif srv_th < srv_my:
            s_wins += 1
        elif srv_my < srv_th:
            s_losses += 1
        env.close()

    tot = max(my_hp, 1e-9)
    return dict(target=target, w=wins, l=losses, n=n, draws=n - wins - losses,
                zero=zero_scored, margins=margins,
                my=my_hp / n, th=th_hp / n, net=(my_hp - th_hp) / n,
                cone1=cone1 / 60 / n, crash=crash, tgt_crash=tgt_crash,
                secs=secs_total / n,
                p1=100 * my_phase[1] / tot, p2=100 * my_phase[2] / tot,
                p3=100 * my_phase[3] / tot,
                s_w=s_wins, s_l=s_losses, s_d=n - s_wins - s_losses,
                s_kills=s_kills, s_deaths=s_deaths,
                s_kt=(float(np.mean(s_kill_times)) if s_kill_times else 0.0),
                s_dt=(float(np.mean(s_death_times)) if s_death_times else 0.0),
                pin_them=pin_them * DT / n, pin_me=pin_me * DT / n,
                pin_them_max=pin_them_max, pin_me_max=pin_me_max)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ownship", default="AIP_DCS.dll")
    ap.add_argument("--targets", default="")
    ap.add_argument("--preset", choices=sorted(PRESETS), default="core")
    ap.add_argument("--num-seeds", type=int, default=30)
    ap.add_argument("--init", choices=["random", "server"], default="random",
                    help="random=기존 make_state, server=실서버 고정조건(15000ft/2000ft/200m/s)")
    # 08-10: 아웃오브샘플 검증용. 지금까지 모든 튜닝이 시드 0~29에서 이루어졌는데
    # 대회는 40시드이고 우리가 본 적 없는 시드가 섞인다. 시드 집합 과적합 여부를
    # 한 번도 확인한 적이 없어 오프셋을 넣는다.
    ap.add_argument("--seed-offset", type=int, default=0)
    args = ap.parse_args()

    global INIT_MODE
    INIT_MODE = args.init

    targets = ([t.strip() for t in args.targets.split(",") if t.strip()]
               if args.targets else PRESETS[args.preset])

    rows = []
    for t in targets:
        if not (ROOT / t).exists():
            print(f"  (건너뜀, 파일 없음) {t}", flush=True)
            continue
        r = run_one(args.ownship, t, args.num_seeds, args.seed_offset)
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

        print()
        print("  [자리 지키기] 밴드 안에서 뒤를 잡힌/잡은 시간 s/판  — 08-15 실서버로 확정된 병목.")
        print("  yuno vs v44는 양쪽 다 0.0초였는데(서로 자리를 안 내준다) 둘 다 우리에겐 40~50초씩 눌러앉았다.")
        print(f"  {'상대':<16} {'뒤잡힘 총':>9} {'최장연속':>9} | {'뒤잡음 총':>9} {'최장연속':>9} | {'비(잡힘/잡음)':>13}")
        for r in rows:
            rt = (r['pin_them'] / r['pin_me']) if r['pin_me'] > 0.05 else float('inf')
            rs = "  inf" if rt == float('inf') else f"{rt:6.2f}"
            print(f"  {r['target']:<16} {r['pin_them']:>9.1f} {r['pin_them_max']:>9.1f} | "
                  f"{r['pin_me']:>9.1f} {r['pin_me_max']:>9.1f} | {rs:>13}")
        tp = sum(r['pin_them'] for r in rows) / len(rows)
        mp = sum(r['pin_me'] for r in rows) / len(rows)
        print(f"  -> 평균 뒤잡힘 {tp:.1f}s / 뒤잡음 {mp:.1f}s   "
              f"최장연속 최악 {max(r['pin_them_max'] for r in rows):.1f}s")

        print()
        print("  [보수 모델] Phase 1만(ATA<1도, 152~914m) 인정한 민감도 확인용.")
        print("  🔴 08-15 철회: 한때 이것을 '뷰어 실측 모델'이라 부르며 실서버 모델로 삼았으나")
        print("     **근거가 없었다.** `_viewer_score.py`의 결함 3건(위상 합산, 시간축 2.07배")
        print("     미환산, 1초 도장 위 보간)이 만든 유령이었다. 채점기를 고쳐 실서버 5판을")
        print("     다시 재니 **규정 모델(P1+P2+P3)이 5판 모두와 일치**한다. 무승부 판이 0인 건")
        print("     뷰어가 Phase를 빼먹어서가 아니라 우리 ATA 최소가 1.5/3.4/3.2도로 **원뿔")
        print("     안에 든 적이 없어서**다. 위 [규정 모델]이 유일한 판정 기준이다.")
        print("  로컬 피해량 1.0 = 실서버 HP 100 = 격추. 배율은 없다(단위 차이였다).")
        print(f"  {'상대':<16} {'승':>3} {'패':>3} {'무':>3} | {'격추':>4} {'피격추':>6} | "
              f"{'격추시각':>8} {'피격시각':>8} | {'승점':>7}")
        for r in rows:
            spts = r["s_w"] + 0.5 * r["s_d"]
            kt = f"{r['s_kt']:.0f}s" if r["s_kills"] else "-"
            dt_ = f"{r['s_dt']:.0f}s" if r["s_deaths"] else "-"
            print(f"  {r['target']:<16} {r['s_w']:>3} {r['s_l']:>3} {r['s_d']:>3} | "
                  f"{r['s_kills']:>4} {r['s_deaths']:>6} | {kt:>8} {dt_:>8} | {spts:>7.1f}")
        spts_all = sum(r["s_w"] + 0.5 * r["s_d"] for r in rows)
        print(f"  → 실서버 승점 {spts_all:.1f} / {alln}  ({100*spts_all/alln:.1f}%)"
              f"   [rules {allpts:.1f} 대비 {spts_all-allpts:+.1f}]")
        flips = [r["target"] for r in rows
                 if (r["w"], r["l"]) != (r["s_w"], r["s_l"])]
        print(f"  판정이 갈린 상대: {', '.join(flips) if flips else '없음'}")
    print("=" * 96)


if __name__ == "__main__":
    main()
