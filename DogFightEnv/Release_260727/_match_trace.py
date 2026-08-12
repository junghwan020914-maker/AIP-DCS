"""임의의 두 DLL 대전을 시계열로 찍는다 — 설계 전 계측용.

08-10 `_stalemate_diag.py`는 집계(국면 점유율, 공세밴드 시간)만 준다. arcV 매치업이
"10판 전부 중립 98%, 공세밴드 0.0s, 200초 만료"라는 것까지는 알아냈지만, **누가 언제
올라가고 우리가 따라가는지, 에너지가 어디서 새는지**는 시계열이 있어야 보인다.

오늘 처방 9건이 전부 실패한 이유가 계측 없이 기전을 추측한 것이었으므로,
수직 추격 분기를 설계하기 전에 먼저 이걸로 실제 역학을 본다.

사용:
    python _match_trace.py --ownship AIP_DCS.dll --target AIP_arcV.dll --seed 0
    python _match_trace.py --ownship AIP_DCS.dll --target AIP_arcV.dll --seed 0 --every 2

상태 규약: state = [N, E, D, roll, pitch, heading, speed]
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
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--every", type=float, default=5.0, help="샘플 간격(초)")
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
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
    env.reset(seed=args.seed)

    print(f"[TR] {args.ownship} vs {args.target}  seed={args.seed}  "
          f"시작 고도 {-own[2]:.0f}m 속도 {own[6]:.0f} 분리 "
          f"{np.hypot(tgt[0]-own[0], tgt[1]-own[1]):.0f}m", flush=True)
    print("[TR]     t   거리 |  내고도 내속도 내피치 |  적고도 적속도 적피치 | "
          "내ATA 적ATA | dH    dV   | 내득점", flush=True)

    stride = max(1, int(round(args.every / DT)))
    terminated = truncated = False
    step = 0
    my_hp = 0.0
    while not (terminated or truncated):
        _, _, terminated, truncated, _ = env.step(np.zeros(4, dtype=np.float32))
        step += 1
        t_s = step * DT
        g, o, t = env._geo_info, env._ownship_state, env._target_state
        d = float(g._get_distance(o, t))
        ma = abs(float(g._get_antenna_train_angle(o, t, False)))
        my_hp += score_rate(d, ma, t_s)[0] * DT
        if step % stride:
            continue
        ta = abs(float(g._get_antenna_train_angle(t, o, False)))
        ma_h, ta_h = -float(o[2]), -float(t[2])
        print(f"[TR] {t_s:5.0f} {d:6.0f} | {ma_h:7.0f} {o[6]:6.1f} {o[4]:6.1f} | "
              f"{ta_h:7.0f} {t[6]:6.1f} {t[4]:6.1f} | {ma:5.1f} {ta:5.1f} | "
              f"{ma_h-ta_h:+6.0f} {o[6]-t[6]:+5.1f} | {my_hp:.4f}", flush=True)
    env.close()
    print(f"[TR] 종료 t={step*DT:.1f}s  내득점={my_hp:.4f}", flush=True)


if __name__ == "__main__":
    main()
