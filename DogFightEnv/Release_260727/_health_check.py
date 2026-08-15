"""시뮬레이터가 들고 있는 체력과 **내가 파이썬에서 재계산한 피해량**을 대조한다.

08-15 배경. 사용자 지적: "궤적에서 역산하는 식과 채점기는 아주 위험해. 정직하게
데이터를 받아오고 분석해야해." 실서버 채점기(`_viewer_score.py`)에서 그 위험이
실제로 터졌고(결함 3건, 판정 철회), 같은 패턴이 **로컬 루프에도 있다**:

    `single_agent_env.py`는 `StateIndex.HEALTH`를 직접 들고 `update_damage()`로
    갱신하며 `info["ownship_health"]` / `info["target_health"]`로 내보낸다.
    그런데 `_suite_eval.py`와 모든 프로브는 그걸 **안 읽고** `score_rate`로 재계산한다.

구조는 같아 보인다 — Phase1 우선, 안 맞으면 P2/P3 폴백, 시간 게이트(100/150s).
**그러나 같아 보이는 것과 같은 것은 다르다.** 특히 눈여겨볼 차이:

  · env는 Phase1 반각을 `self._wez["angle_deg"] / 2.0`으로 **설정에서** 읽는다.
    내 `PHASES`는 1.0도를 하드코딩한다. 설정이 2.0이 아니면 즉시 갈린다.
  · env의 경과시간은 `self._ep_step_count * self._delta_t`, 내 것은 `step * DT`.
  · env는 `deduct_health`로 누산하고 0에서 잘릴 수 있다(격추 후 추가 피해 무시).

**이 프로브는 처방이 아니라 감사(audit)다.** 결과에 따라:
  · 일치하면 -> 과거 수치는 유효하다. 그래도 **앞으로는 env 값을 읽는다**(같은 값이면
    재계산할 이유가 없고, 환경이 바뀌면 조용히 갈라지기 때문).
  · 어긋나면 -> 어긋난 양과 방향이 곧 지금까지 쌓은 모든 수치의 오차다.

⚠️ 판정 기준을 미리 박는다. 체력은 0~1 정규화이고 격추가 1.0이므로:
    · 최대 절대차 < 1e-6  -> 동일. 재계산이 우연히 맞은 게 아니라 같은 식이다.
    · 1e-6 ~ 1e-3        -> 부동소수 누산 차이. 실무상 무해하나 env 값으로 갈아탄다.
    · > 1e-3             -> **모델이 다르다.** 원인을 찾기 전에는 어떤 수치도 못 믿는다.

사용:
    python _health_check.py --targets AIP_arcD.dll,AIP_ryujan_v42.dll --num-seeds 4
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
    ap.add_argument("--targets", default="AIP_arcD.dll,AIP_ryujan_v42.dll")
    ap.add_argument("--num-seeds", type=int, default=4)
    ap.add_argument("--seed-offset", type=int, default=0)
    args = ap.parse_args()

    print("=" * 104)
    print(f"[체력 감사] 시뮬레이터 info 체력  대  내 score_rate 재계산   나={args.ownship}")
    print(f"  {'상대':<20} {'시드':>4} | {'env 상대피해':>12} {'내 계산':>10} {'차':>11} | "
          f"{'env 내피해':>11} {'내 계산':>10} {'차':>11}")
    worst = 0.0
    missing = False
    for tname in [x.strip() for x in args.targets.split(",") if x.strip()]:
        if not (ROOT / tname).exists():
            print(f"  (없음) {tname}")
            continue
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
                target_action_provider=BTActionProvider(dll_name=tname),
            )
            env.reset(seed=seed)
            mine = them = 0.0          # 내 재계산 (내가 상대에게 / 상대가 나에게)
            info = {}
            terminated = truncated = False
            step = 0
            while not (terminated or truncated):
                _, _, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
                step += 1
                t_s = step * DT
                g, o, t = env._geo_info, env._ownship_state, env._target_state
                d = float(g._get_distance(o, t))
                ma = abs(float(g._get_antenna_train_angle(o, t, False)))
                ta = abs(float(g._get_antenna_train_angle(t, o, False)))
                mine += score_rate(d, ma, t_s)[0] * DT
                them += score_rate(d, ta, t_s)[0] * DT
            env.close()

            if "target_health" not in info or "ownship_health" not in info:
                print(f"  {tname:<20} {seed:>4} | info에 체력 없음 — 키: "
                      f"{sorted(info.keys())[:8]}")
                missing = True
                continue
            env_t = 1.0 - float(info["target_health"])      # 내가 상대에게 준 피해
            env_o = 1.0 - float(info["ownship_health"])
            d1, d2 = env_t - mine, env_o - them
            worst = max(worst, abs(d1), abs(d2))
            print(f"  {tname:<20} {seed:>4} | {env_t:>12.6f} {mine:>10.6f} {d1:>+11.2e} | "
                  f"{env_o:>11.6f} {them:>10.6f} {d2:>+11.2e}", flush=True)

    print(f"\n  최대 절대차 {worst:.3e}")
    if missing:
        print("  -> info에 체력이 없는 경우가 있었다. 위 키 목록을 보고 경로를 고칠 것.")
    elif worst < 1e-6:
        print("  -> **동일하다.** 과거 수치는 유효하다. 그래도 앞으로는 env 값을 읽는다")
        print("     (같은 값이면 재계산할 이유가 없고, 환경이 바뀌면 조용히 갈라진다).")
    elif worst < 1e-3:
        print("  -> 부동소수 누산 차이 수준. 실무상 무해하나 env 값으로 갈아탄다.")
    else:
        print("  -> 🔴 **모델이 다르다.** 원인을 찾기 전에는 어떤 로컬 수치도 믿을 수 없다.")
        print("     의심 1순위: env는 Phase1 반각을 _wez['angle_deg']/2 로 설정에서 읽는데")
        print("     내 PHASES는 1.0도를 하드코딩한다.")
    print("=" * 104)


if __name__ == "__main__":
    main()
