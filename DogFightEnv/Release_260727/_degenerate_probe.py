"""퇴화 상대 위생검사 — 현재 상대 풀에 **없는** 실패 모드를 잡는다.

현재 전수 평가의 상대는 전부 "제대로 싸우는" BT다(ryujan v29/v32, 자작 arcA/arcE,
이전 세대). 그래서 아래 유형은 **한 번도 검증된 적이 없다**:

  duck   직진 수평비행    — 가만히 있는 표적을 확실히 잡는가 (가장 기본)
  runner 최대추력 직진    — 도주하는 표적을 따라잡는가 (에너지/추격)
  circler 정상선회 45도뱅크 — 예측 가능한 선회 표적을 컷할 수 있는가

본선 40시드에는 미완성 AI나 사실상 정지한 기체가 섞일 수 있다. 그런 판에서
득점을 못 하면 0-0 무승부(승점 0.5)가 되어 **이길 판을 그냥 버린다**.
또 어느 하나라도 실패하면 그건 튜닝 문제가 아니라 트리의 구조적 결함이다.

🔴 08-10 반드시 읽을 것 — **이 시험은 "얻는 것"만 보여주고 "치르는 대가"는 못 보여준다.**
여기 상대는 **반격하지 않는다.** 그래서 "장거리에서 당기지 말고 가속" 같은 변경은
이득만 계상된다. 실제로 `Sprint`(사거리 밖 코스팅)는 duck 무득점 4/6 -> **0/6**으로
완전히 고쳤는데, 코어 전수평가에서는 **처음으로 패배를 만들며 기각**됐다
(144.0 vs 144.5, arcA 30.0->29.0 / prev 29.5->29.0, 피격 0.0000 -> 0.0335/0.0250).
장거리에서 안 당기고 직진하면 상대가 전환해 올 때 기동하고 있지 않아 맞는다.

-> **여기서 좋아졌다고 채택하지 말 것.** 반드시 `_suite_eval.py --preset core`로
   반격하는 상대에게 치르는 대가를 확인한 뒤에 판정한다.
   반대 방향도 성립한다 — 코어만 보면 이 시험이 잡는 구조적 결함을 못 본다.
   **둘 다 봐야 한다.**

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
from dogfight.ai.action_provider import ActionProvider, ActionResult, clip_action  # noqa: E402
from dogfight.ai.bt_action_provider import BTActionProvider  # noqa: E402


class ScriptedProvider(ActionProvider):
    """자세 유지 스크립트 조종사. 원시 조종면 명령이므로 간단한 비례 자세유지를 건다."""

    def __init__(self, kind: str):
        self.kind = kind
        if kind == "duck":
            self.bank, self.pitch_t, self.thr = 0.0, 0.0, 0.70
        elif kind == "runner":
            self.bank, self.pitch_t, self.thr = 0.0, 0.0, 1.00
        elif kind == "circler":
            self.bank, self.pitch_t, self.thr = 45.0, 4.0, 0.90
        else:
            raise ValueError(kind)

    def compute_action(self, context) -> ActionResult:
        s = context.ownship_state
        roll = float(s[3]) if s is not None else 0.0
        pitch = float(s[4]) if s is not None else 0.0
        # 비례 자세유지: 명령 = -k * (현재 - 목표)
        roll_cmd = np.clip(-(roll - self.bank) * 0.05, -1.0, 1.0)
        pitch_cmd = np.clip((pitch - self.pitch_t) * 0.08, -1.0, 1.0)
        act = clip_action([roll_cmd, pitch_cmd, 0.0, self.thr])
        return ActionResult(action=np.asarray(act, dtype=np.float32), source=self.kind)


def run(ownship, kind, seeds):
    out = []
    for seed in range(seeds):
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
            target_action_provider=ScriptedProvider(kind),
        )
        env.reset(seed=seed)
        terminated = truncated = False
        step = 0
        my_hp = th_hp = 0.0
        my_alt_min = tgt_alt_min = 1e9
        first_score = None
        dmin = 1e9
        while not (terminated or truncated):
            _, _, terminated, truncated, _ = env.step(np.zeros(4, dtype=np.float32))
            step += 1
            t_s = step * DT
            g = env._geo_info
            os_, ts_ = env._ownship_state, env._target_state
            d = float(g._get_distance(os_, ts_))
            ma = abs(float(g._get_antenna_train_angle(os_, ts_, False)))
            ta = abs(float(g._get_antenna_train_angle(ts_, os_, False)))
            dmin = min(dmin, d)
            my_alt_min = min(my_alt_min, -float(os_[2]))
            tgt_alt_min = min(tgt_alt_min, -float(ts_[2]))
            r = score_rate(d, ma, t_s)[0]
            if r > 0.0 and first_score is None:
                first_score = t_s
            my_hp += r * DT
            th_hp += score_rate(d, ta, t_s)[0] * DT
        env.close()
        out.append(dict(seed=seed, my=my_hp, th=th_hp, secs=step * DT, dmin=dmin,
                        first=first_score, my_alt=my_alt_min, tgt_alt=tgt_alt_min))
        o = out[-1]
        fs = f"{o['first']:5.1f}s" if o["first"] is not None else "  없음"
        print(f"  [{kind:<7}] seed {seed:>2}  내{o['my']:7.4f} 상대{o['th']:6.4f} | "
              f"첫득점 {fs} | 최소거리 {o['dmin']:6.0f}m | "
              f"최저고도 내{o['my_alt']:5.0f} 상{o['tgt_alt']:5.0f} | {o['secs']:5.1f}s",
              flush=True)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ownship", default="AIP_DCS.dll")
    ap.add_argument("--num-seeds", type=int, default=8)
    ap.add_argument("--kinds", default="duck,runner,circler")
    args = ap.parse_args()

    print("=" * 96)
    print(f"[퇴화 상대 위생검사] {args.num_seeds}시드/유형   나={args.ownship}")
    summary = []
    for kind in args.kinds.split(","):
        rs = run(args.ownship, kind.strip(), args.num_seeds)
        zero = sum(1 for r in rs if r["my"] <= 0.0)
        crash = sum(1 for r in rs if r["my_alt"] < 300.0)
        med = np.median([r["my"] for r in rs])
        firsts = [r["first"] for r in rs if r["first"] is not None]
        summary.append((kind, zero, crash, med, np.median(firsts) if firsts else None,
                        np.median([r["dmin"] for r in rs]), len(rs)))
        print()

    print("=" * 96)
    print(f"  {'유형':<9} {'무득점':>7} {'내추락':>7} {'내득점중앙':>11} {'첫득점':>8} {'최소거리':>9}")
    for kind, zero, crash, med, fst, dmin, n in summary:
        fs = f"{fst:.1f}s" if fst is not None else "없음"
        flag = ""
        if zero > 0:
            flag += f"  ⚠ 무득점 {zero}/{n}"
        if crash > 0:
            flag += f"  🔴 추락 {crash}/{n}"
        print(f"  {kind:<9} {zero:>7} {crash:>7} {med:>11.4f} {fs:>8} {dmin:>8.0f}m{flag}")
    print("  ※ duck(정지표적)에서 무득점이 하나라도 나오면 구조적 결함이다.")
    print("=" * 96)


if __name__ == "__main__":
    main()
