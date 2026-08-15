"""`_engagement_recorder.py`가 받아적은 실서버 교전을 규정대로 채점한다.

08-15. 뷰어 CSV(1초 정수 도장 + 경기시계 2.07배)를 버리고, 서버가 프레임마다 보내는
값을 그대로 기록한 JSONL을 읽는다. 시간축은 **서버 자신의 프레임 번호**를 쓰므로
추측이 없다(60Hz = 경기 시계, `_project_damage_model_confirmed` 참고).

## ⚠️ 좌표·자세 규약을 가정하지 않는다 — 데이터가 고르게 한다

오늘 궤적 역산으로 하루를 잃은 원인이 "그럴듯한 가정"이었다. 그래서 여기서는
규약을 **검증 가능한 항등식**으로 결정한다.

관측: 첫 프레임에서 `vel = [200.0, 0.0, 0.1]`이고 기체는 정지 상태가 아니다.
즉 `vel`은 world 속도가 아니라 **동체 좌표계(u,v,w)**로 보인다. 그렇다면 항등식

    R(roll,pitch,yaw) @ vel_body  ==  d(pos)/dt

가 성립해야 한다. 우변은 위치 차분으로 독립적으로 구할 수 있으므로, 여러 회전 규약
후보를 넣어보고 **이 항등식을 만족하는 것 하나**를 고르면 된다. 이건 가정이 아니라
측정이다. 그리고 각도 오차가 크면 **채점을 거부한다** — 틀린 규약으로 낸 숫자는
오늘 아침의 0.2667과 같은 유령이다.

규약이 정해지면 기수 벡터는 `R @ [1,0,0]`이고, 거기서 ATA를 낸다.
(속도 벡터가 아니라 기수 방향이어야 한다 — 받음각만큼 다르고 실측 p99가 16.7도다.)

## 데미지 모델

`_aim_time_probe.PHASES`를 그대로 쓴다. 이 식은 08-15에 로컬 시뮬레이터의 자체
`update_damage()`와 **15,220틱 전수 대조에서 불일치 0**으로 검증됐다(`_health_check.py`).
Phase1 우선, 안 맞으면 P2(t>=100s)·P3(t>=150s) 폴백, 합산 아님.

사용:
    python _engagement_score.py engagements/0815_mine_vs_v42_1.jsonl
Not part of the student template; safe to delete after use.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
for p in (ROOT, ROOT / "src"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from _aim_time_probe import score_rate  # noqa: E402

SERVER_HZ = 60.0
BAND_MIN, BAND_MAX = 152.4, 914.4


def _R(conv, roll, pitch, yaw):
    """후보 회전행렬. 열 [fwd, right, up] 꼴로, 동체벡터를 world로 옮긴다."""
    r, p, y = np.radians([roll, pitch, yaw])
    cr, sr, cp, sp, cy, sy = (np.cos(r), np.sin(r), np.cos(p),
                              np.sin(p), np.cos(y), np.sin(y))
    if conv == "compass_zup":          # yaw는 +y(북)에서 시계방향, z는 위
        f = np.array([cp * sy, cp * cy, sp])
        rt = np.array([cr * cy + sr * sp * sy, -cr * sy + sr * sp * cy, -sr * cp])
    elif conv == "math_zup":           # yaw는 +x에서 반시계, z는 위
        f = np.array([cp * cy, cp * sy, sp])
        rt = np.array([sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, -sr * cp])
    elif conv == "compass_zup_negp":   # 위와 같으나 pitch 부호 반대
        f = np.array([cp * sy, cp * cy, -sp])
        rt = np.array([cr * cy - sr * sp * sy, -cr * sy - sr * sp * cy, -sr * cp])
    elif conv == "math_zup_negp":
        f = np.array([cp * cy, cp * sy, -sp])
        rt = np.array([-sr * sp * cy - cr * sy, -sr * sp * sy + cr * cy, -sr * cp])
    else:
        raise ValueError(conv)
    u = np.cross(rt, f)
    return np.stack([f, rt, u], axis=1)      # world = R @ body


CONVS = ("compass_zup", "math_zup", "compass_zup_negp", "math_zup_negp")


def load(path: Path):
    fr, out = [], None
    for line in path.open(encoding="utf-8"):
        try:
            o = json.loads(line)
        except json.JSONDecodeError:
            continue
        if o["kind"] == "frame" and o.get("own") and o.get("enemy"):
            fr.append(o)
        elif o["kind"] == "outcome":
            out = o
    return fr, out


def fit_convention(fr, stride=6):
    """R @ vel_body == d(pos)/dt 를 가장 잘 만족하는 규약을 고른다."""
    idx = list(range(stride, len(fr) - stride, 13))
    res = {}
    for c in CONVS:
        ang, mag = [], []
        for i in idx:
            dt = (fr[i + stride]["frame_index"] - fr[i - stride]["frame_index"]) / SERVER_HZ
            if dt <= 0:
                continue
            dp = (np.array(fr[i + stride]["own"]["pos"], float)
                  - np.array(fr[i - stride]["own"]["pos"], float)) / dt
            vb = np.array(fr[i]["own"]["vel"], float)
            vw = _R(c, *fr[i]["own"]["rot"]) @ vb
            n1, n2 = np.linalg.norm(dp), np.linalg.norm(vw)
            if n1 < 1.0 or n2 < 1.0:
                continue
            ang.append(np.degrees(np.arccos(np.clip(np.dot(dp, vw) / (n1 * n2), -1, 1))))
            mag.append(n2 / n1)
        res[c] = (float(np.median(ang)) if ang else 1e9,
                  float(np.median(mag)) if mag else 0.0, len(ang))
    return res


def ata(pos_a, fwd_a, pos_b):
    los = pos_b - pos_a
    r = float(np.linalg.norm(los))
    if r < 1e-9:
        return 0.0, r
    c = float(np.clip(np.dot(fwd_a, los) / r, -1, 1))
    return float(np.degrees(np.arccos(c))), r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path")
    ap.add_argument("--max-fit-error-deg", type=float, default=25.0,
                    help="규약 적합 각도 오차 한계. 넘으면 채점을 거부한다")
    args = ap.parse_args()

    path = Path(args.path)
    fr, out = load(path)
    print("=" * 96)
    print(f"[실서버 채점] {path.name}   프레임 {len(fr)}")
    if len(fr) < 100:
        print("  프레임 부족")
        return
    dur = (fr[-1]["frame_index"] - fr[0]["frame_index"]) / SERVER_HZ
    print(f"  서버 프레임 {fr[0]['frame_index']} ~ {fr[-1]['frame_index']}  "
          f"= 경기시계 {dur:.1f}s")

    fit = fit_convention(fr)
    print(f"\n  [규약 적합] R @ vel_body 대 d(pos)/dt  — 가정하지 않고 데이터로 고른다")
    print(f"  {'후보':<20} {'각도오차중앙':>12} {'크기비':>8} {'표본':>7}")
    for c, (a, m, n) in sorted(fit.items(), key=lambda kv: kv[1][0]):
        print(f"  {c:<20} {a:>11.2f}도 {m:>8.3f} {n:>7}")
    best = min(fit, key=lambda c: fit[c][0])
    err, mag, _ = fit[best]
    print(f"  -> 채택: **{best}** (오차 {err:.2f}도, 크기비 {mag:.3f})")
    if err > args.max_fit_error_deg:
        print(f"\n  🔴 적합 오차 {err:.2f}도 > 한계 {args.max_fit_error_deg}도 — **채점을 거부한다.**")
        print("     규약을 못 정한 채로 낸 각도는 믿을 수 없다. 후보를 늘리거나")
        print("     vel이 동체 좌표계라는 전제부터 다시 볼 것.")
        print("=" * 96)
        return

    mine = them = 0.0
    mp = {1: 0.0, 2: 0.0, 3: 0.0}
    tp = {1: 0.0, 2: 0.0, 3: 0.0}
    MA, TA, D = [], [], []
    prev_fi = fr[0]["frame_index"]
    for o in fr:
        fi = o["frame_index"]
        dt = max(0.0, (fi - prev_fi) / SERVER_HZ)
        prev_fi = fi
        t_s = fi / SERVER_HZ
        pa = np.array(o["own"]["pos"], float)
        pb = np.array(o["enemy"]["pos"], float)
        fa = _R(best, *o["own"]["rot"])[:, 0]
        fb = _R(best, *o["enemy"]["rot"])[:, 0]
        a1, r = ata(pa, fa, pb)
        a2, _ = ata(pb, fb, pa)
        MA.append(a1); TA.append(a2); D.append(r)
        rate, ph = score_rate(r, a1, t_s)
        if ph:
            mine += rate * dt; mp[ph] += rate * dt
        rate, ph = score_rate(r, a2, t_s)
        if ph:
            them += rate * dt; tp[ph] += rate * dt

    MA, TA, D = np.array(MA), np.array(TA), np.array(D)
    band = (D >= BAND_MIN) & (D <= BAND_MAX)
    print(f"\n  거리   중앙 {np.median(D):6.0f}m  최소 {D.min():5.0f}m  "
          f"밴드(152~914m) 체류 {100*band.mean():.1f}%")
    print(f"  내 ATA  최소 {MA.min():5.2f}도  중앙 {np.median(MA):6.1f}도  "
          f"밴드내 <1도 {100*np.mean(MA[band] < 1) if band.any() else 0:.2f}%")
    print(f"  적 ATA  최소 {TA.min():5.2f}도  중앙 {np.median(TA):6.1f}도  "
          f"밴드내 <1도 {100*np.mean(TA[band] < 1) if band.any() else 0:.2f}%")
    print(f"\n  피해(규정 P1+P2+P3, 안쪽 우선)")
    print(f"  {'':<6} {'P1':>10} {'P2':>10} {'P3':>10} {'합':>10} {'환산HP':>9}")
    print(f"  {'나->적':<6} {mp[1]:>10.5f} {mp[2]:>10.5f} {mp[3]:>10.5f} "
          f"{mine:>10.5f} {100*(1-mine):>9.2f}")
    print(f"  {'적->나':<6} {tp[1]:>10.5f} {tp[2]:>10.5f} {tp[3]:>10.5f} "
          f"{them:>10.5f} {100*(1-them):>9.2f}")
    pred = "무승부" if abs(mine - them) < 1e-12 else ("승리" if mine > them else "패배")
    print(f"  -> 예측 결과: **{pred}**")

    if out:
        print(f"\n  [화면 판독]  내HP {out['my_hp']}  상대HP {out['opp_hp']}  {out['result']}")
        eh, em = 100 * (1 - them), 100 * (1 - mine)
        print(f"  [채점 예측]  내HP {eh:.2f}  상대HP {em:.2f}")
        d1, d2 = abs(eh - float(out["my_hp"])), abs(em - float(out["opp_hp"]))
        print(f"  차이         {d1:.2f} / {d2:.2f}")
        print(f"  -> {'일치한다' if max(d1, d2) < 0.5 else '**어긋난다 — 원인을 찾기 전에는 이 채점기를 믿지 말 것**'}")
    else:
        print("\n  화면 결과 미기록 — `--annotate`로 남길 것")
    print("=" * 96)


if __name__ == "__main__":
    main()
