"""감속 권한을 직접 잰다 — 아이들 스로틀과 고G 선회로 실제 몇 m/s^2까지 줄일 수 있나.

08-13 배경. "뒤를 잡히면 못 벗어난다"에 대해 감속으로 오버슛을 강제하는 안을 검토 중인데,
계측 결과 우리는 **받음각 제한이 아니라 G 제한**이고(받음각 p99 16.7도, 30도 한계에 미달)
이미 **코너속도**(이 고도에서 260~280 m/s)에 있다. 즉 "받음각을 세워 감속"도
"느려져서 선회율을 얻기"도 닫혔다.

남은 질문은 순수한 양의 문제다: **스로틀을 0까지 내리고 최대 G로 당기면 몇 초에 얼마나
느려지는가.** 오버슛 강제는 상대와의 **감속 차이**로 성립하므로, 이 절대값이 작으면
그 전술 자체가 성립하지 않는다.

조종 규약(FighterSim.step):
    action[0] 롤  -1~1 (-1 좌, +1 우)
    action[1] 피치 -1~1 (**-1 당김/기수올림**, +1 밀기)
    action[2] 방향타 -1~1
    action[3] 스로틀 0~1 (0 = 아이들). clip 하한이 0.0이라 0까지 쓸 수 있다.

BT를 우회해 고정 명령을 넣어야 하므로 최소 액션 제공자를 직접 만든다.

사용:
    python _decel_probe.py
Not part of the student template; safe to delete after use.
"""
from __future__ import annotations

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
from dogfight.ai.action_provider import ActionResult, clip_action  # noqa: E402
from dogfight.ai.bt_action_provider import BTActionProvider  # noqa: E402

I_KTAS, I_NZ, I_AOA, I_THR = 27, 31, 13, 21


class FixedAction:
    """고정 조종 입력. env의 action_provider 자리에 끼운다."""

    def __init__(self, roll=0.0, pitch=0.0, rudder=0.0, throttle=1.0):
        self.cmd = np.array([roll, pitch, rudder, throttle], dtype=np.float32)

    def set(self, roll, pitch, rudder, throttle):
        self.cmd[:] = (roll, pitch, rudder, throttle)

    def compute_action(self, context):
        return ActionResult(action=clip_action(self.cmd), source="fixed",
                            confidence=1.0, info={})

    def reset(self, context=None):
        return None

    def close(self):
        return None


def run_case(label, pitch, throttle, secs=20.0, seed=0):
    rng = np.random.default_rng(seed)
    own, tgt = make_state(rng)
    ctl = FixedAction(roll=0.0, pitch=pitch, rudder=0.0, throttle=throttle)
    env = DogFightWrapper(
        env_config={
            "observation_mode": "tactical16", "ownship_control_mode": "rl",
            "target_mode": "rl", "max_engage_time": 200.0, "min_altitude": 300.0,
            "ownship": own, "target": tgt,
            "initial_scenario": {"mode": "default"},
        },
        ownship_action_provider=ctl,
        target_action_provider=BTActionProvider(dll_name="AIP_v29.dll"),
    )
    env.reset(seed=seed)
    # 1초 안정화
    for _ in range(60):
        env.step(np.zeros(4, dtype=np.float32))
    o = env._ownship_state
    v0 = float(o[I_KTAS])
    h0 = -float(o[2])
    trace = []
    n = int(secs / DT)
    for i in range(n):
        _, _, term, trunc, _ = env.step(np.zeros(4, dtype=np.float32))
        o = env._ownship_state
        if (i + 1) % 60 == 0:
            trace.append(((i + 1) * DT, float(o[I_KTAS]), abs(float(o[I_NZ])),
                          abs(float(o[I_AOA])), -float(o[2])))
        if term or trunc:
            break
    env.close()
    if not trace:
        print(f"  {label:<34} 표본 없음")
        return
    t_end, v_end, nz_end, aoa_end, h_end = trace[-1]
    dv = v_end - v0
    print(f"  {label:<34} {v0:5.1f} -> {v_end:5.1f} m/s  "
          f"({dv:+6.1f} in {t_end:4.1f}s = {dv/t_end:+5.2f} m/s^2)  "
          f"Nz {nz_end:4.1f}  AOA {aoa_end:4.1f}  고도 {h0:5.0f}->{h_end:5.0f}m")
    return [(0.0, v0)] + [(t, v) for t, v, *_ in trace]


def run_pulse(hold_s, seed=0, recover_s=25.0):
    """hold_s초 동안 최대당김+아이들, 그 뒤 중립+풀스로틀로 복귀시켜 회복 여부를 본다.

    복귀 실패(딥스톨 고착)는 규정 제6조1항 추락 = 즉시 패배이므로 타협 불가 검증이다.
    """
    rng = np.random.default_rng(seed)
    own, tgt = make_state(rng)
    ctl = FixedAction(0.0, 0.0, 0.0, 1.0)
    env = DogFightWrapper(
        env_config={
            "observation_mode": "tactical16", "ownship_control_mode": "rl",
            "target_mode": "rl", "max_engage_time": 200.0, "min_altitude": 300.0,
            "ownship": own, "target": tgt,
            "initial_scenario": {"mode": "default"},
        },
        ownship_action_provider=ctl,
        target_action_provider=BTActionProvider(dll_name="AIP_v29.dll"),
    )
    env.reset(seed=seed)
    for _ in range(60):
        env.step(np.zeros(4, dtype=np.float32))
    o = env._ownship_state
    v0, h0 = float(o[I_KTAS]), -float(o[2])

    ctl.set(0.0, -1.0, 0.0, 0.0)                     # 최대당김 + 아이들
    aoa_max = 0.0
    for _ in range(int(hold_s / DT)):
        env.step(np.zeros(4, dtype=np.float32))
        aoa_max = max(aoa_max, abs(float(env._ownship_state[I_AOA])))
    o = env._ownship_state
    v1, h1 = float(o[I_KTAS]), -float(o[2])

    ctl.set(0.0, 0.0, 0.0, 1.0)                      # 중립 + 풀스로틀로 복귀
    h_min = 1e9
    crashed = False
    for _ in range(int(recover_s / DT)):
        _, _, term, trunc, _ = env.step(np.zeros(4, dtype=np.float32))
        o = env._ownship_state
        h_min = min(h_min, -float(o[2]))
        aoa_max = max(aoa_max, abs(float(o[I_AOA])))
        if -float(o[2]) <= 300.0:
            crashed = True
        if term or trunc:
            break
    o = env._ownship_state
    v2, h2 = float(o[I_KTAS]), -float(o[2])
    env.close()
    ok = (v2 > 120.0) and not crashed
    print(f"  {hold_s:4.1f}초 유지 | 속도 {v0:5.1f} -> {v1:5.1f} ({v1-v0:+6.1f}) "
          f"| 복귀 {recover_s:.0f}초 후 {v2:5.1f} m/s  최저고도 {h_min:5.0f}m "
          f"(시작 {h0:5.0f}) | 최대AOA {aoa_max:4.1f}도 | "
          f"{'✅ 회복' if ok else '🔴 회복실패'}{'  추락!' if crashed else ''}")


def main():
    print("=" * 104)
    print("[감속 권한 계측]  1초 안정화 후 고정 입력 20초.  피치 -1 = 최대 당김")
    print(f"  {'조건':<34} {'속도 변화':<52}")
    cases = [
        ("① 수평·풀스로틀 (기준)",            0.0, 1.0),
        ("② 수평·아이들",                      0.0, 0.0),
        ("③ 최대당김·풀스로틀",               -1.0, 1.0),
        ("④ 최대당김·아이들  <- 최대 감속",   -1.0, 0.0),
        ("⑤ 반당김·아이들",                   -0.5, 0.0),
    ]
    curves = {}
    for label, p, th in cases:
        curves[label] = run_case(label, p, th)

    print()
    print("  [속도 시계열]  1초 간격")
    for label, c in curves.items():
        if not c:
            continue
        s = "  ".join(f"{v:5.1f}" for _, v in c[:16])
        print(f"  {label:<34} {s}")
    print()
    print("  ➜ ④와 ①의 차이가 곧 '상대 대비 벌 수 있는 감속 차이'다.")
    print("     오버슛 강제가 성립하려면 이 차이로 몇 초 안에 유의미한 속도차가 나야 한다.")

    print()
    print("  [펄스 + 복귀]  N초만 최대당김·아이들 후 중립·풀스로틀로 되돌린다.")
    print("  복귀 실패 = 딥스톨 고착 = 추락 = 규정상 즉시 패배. 안전 상한을 찾는다.")
    for hold in (1.0, 2.0, 3.0, 4.0, 6.0, 8.0):
        run_pulse(hold)
    print("=" * 104)


if __name__ == "__main__":
    main()
