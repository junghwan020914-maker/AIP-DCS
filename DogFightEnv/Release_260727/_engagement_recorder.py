"""배틀뷰어 교전에서 **서버가 보낸 것을 그대로 기록**한다. 역산도 가공도 하지 않는다.

08-15 배경. 사용자 지시: "앞으로의 교전은 배틀뷰어에서 해야해. 대회 예선과 본선이 그
환경에서 이루어지기 때문이야. 그래서 정직하게 데이터를 받아오는게 필요하고."

지금까지 실서버 분석은 **뷰어가 자기 용도로 남긴 CSV**(`DogFightViewer/Binaries/Win64/Log/`)를
읽었는데 그게 쓸 수 없는 자료였다:

  · `Time` 열이 **1초 정수 도장**이라 20Hz로 기록해도 실질 해상도가 1Hz
  · 그 열이 **경기 시계의 2.07배**로 흘러 phase 게이트가 절반 시각에 열림
  · 보간 오차 중앙 0.42~0.73도, p99 5.2~8.0도 — **1도 판정 자체가 불가능**

그 자료로 "뷰어는 Phase 2/3을 안 준다"는 판정을 내렸다가 08-15에 철회했다.

**그런데 우리 클라이언트는 애초에 더 좋은 것을 받고 있다.** `MT_PlaneInfo`는 매 프레임
양 기체의 위치·자세·속도와 **서버 자신의 프레임 번호(`index`)**를 준다. 프레임 번호가
오므로 시간축을 추측할 필요가 없다. 그걸 우리 시계와 함께 그대로 적으면 된다.

## 설계 원칙 — 이 파일은 오직 받아적기만 한다

  · `client.py` / `policies.py` / 프로토콜을 **건드리지 않는다.** 정책을 감싸기만 한다.
    (`ProviderCommandPolicy.compute_command(context)`가 완성된 프레임마다 한 번 불린다.)
  · 기록하는 값은 **수신한 그대로**다. 좌표 변환·각도 계산·데미지 추정을 하지 않는다.
    분석은 전부 사후에 이 로그를 읽어서 한다.
  · 우리가 보낸 CMD도 같이 남긴다 — 우리 기체가 왜 그렇게 움직였는지 볼 때 필요하고,
    이것도 "우리가 실제로 보낸 값"이지 추정이 아니다.
  · 한 행 = 한 프레임. JSONL이라 도중에 끊겨도 앞부분이 살아남는다.
    (진행 중 파일을 읽어 잘린 CSV로 오보고한 전례가 있어 각 행을 즉시 flush 한다.)

⚠️ **HP는 여전히 못 받는다.** 서버는 `MT_Damage`(타입 3)를 보내지 않는다(미처리 패킷
   덤프 0건, 5판 확인). 프로토콜에는 정의돼 있으므로 **대회 서버가 보내는지는 주최측에
   물어볼 가치가 있다.** 그때까지 HP는 화면에서 읽은 값을 `--note`로 같이 남긴다.

사용:
    # 1) 뷰어에서 시나리오 선택 -> OpenServer -> (아직 Start 누르지 말 것)
    # 2) 클라이언트를 기록 모드로 붙인다
    python run_unreal_inference.py --mode bt --team-name FDSA \
        --server-ip <ip> --bt-dll AIP_DCS.dll --bt-rule-xml Rule_mine.xml \
        --record engagements/2026-08-15_v42_1.jsonl --record-note "vs v42, 2000ft"
    # 3) 뷰어에서 Start
    # 4) 끝나면 화면 HP를 기록에 남긴다
    python _engagement_recorder.py --annotate engagements/2026-08-15_v42_1.jsonl \
        --my-hp 100 --opp-hp 100 --result draw

Not part of the student template; safe to delete after use.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path


def _vec(v):
    return None if v is None else [v.x, v.y, v.z]


def _rot(r):
    return None if r is None else [r.roll, r.pitch, r.yaw]


def _plane(snap):
    """PlaneSnapshot -> 순수 dict. 수신값 그대로, 변환 없음."""
    if snap is None or snap.plane_info is None:
        return None
    p = snap.plane_info
    return {
        "plane_id": p.plane_id,
        "index": p.index,
        "pos": _vec(p.position),
        "rot": _rot(p.rotation),
        "vel": _vec(p.velocity),
    }


class RecordingPolicy:
    """임의의 command policy를 감싸 프레임마다 수신값과 송신 CMD를 적는다.

    정책 자체의 동작은 **전혀 바꾸지 않는다** — 그대로 호출하고 결과를 그대로 돌려준다.
    """

    def __init__(self, inner, path: str | Path, note: str = ""):
        self.inner = inner
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._f = self.path.open("a", encoding="utf-8")
        self._t0 = time.perf_counter()
        self._n = 0
        self._write({
            "kind": "header",
            "wall_utc": time.time(),
            "note": note,
            "schema": "frame: t_mono(s, 클라 수신 시각) / frame_index(서버 프레임 번호) "
                      "/ own,enemy: {plane_id,index,pos[x,y,z],rot[roll,pitch,yaw],vel[x,y,z]} "
                      "/ cmd[roll,pitch,yaw,throttle] (우리가 보낸 값). 전부 원본, 가공 없음.",
        })

    def _write(self, obj) -> None:
        self._f.write(json.dumps(obj, ensure_ascii=False) + "\n")
        self._f.flush()          # 도중에 죽어도 앞부분은 살린다

    def reset(self, context) -> None:
        self._write({"kind": "reset", "t_mono": time.perf_counter() - self._t0,
                     "plane_id": context.plane_id})
        self.inner.reset(context)

    def compute_command(self, context):
        cmd = self.inner.compute_command(context)
        self._n += 1
        self._write({
            "kind": "frame",
            "t_mono": time.perf_counter() - self._t0,
            "frame_index": context.frame_index,
            "own": _plane(context.own_plane),
            "enemy": _plane(context.enemy_plane),
            "cmd": None if cmd is None else
                   [cmd.roll_cmd, cmd.pitch_cmd, cmd.yaw_cmd, cmd.throttle_cmd],
        })
        return cmd

    def close(self) -> None:
        self._write({"kind": "end", "t_mono": time.perf_counter() - self._t0,
                     "frames": self._n})
        self._f.close()

    # 감싼 정책의 나머지 속성은 그대로 통과시킨다
    def __getattr__(self, name):
        return getattr(self.inner, name)


def annotate(path: Path, my_hp, opp_hp, result, note) -> None:
    """교전이 끝난 뒤 **화면에서 읽은** HP와 결과를 같은 파일에 덧붙인다.

    이것만이 유일한 HP 출처다(서버는 MT_Damage를 보내지 않는다). 추정이 아니라
    사람이 화면에서 읽은 값임을 `source: screen`으로 명시해 둔다.
    """
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps({
            "kind": "outcome", "source": "screen", "wall_utc": time.time(),
            "my_hp": my_hp, "opp_hp": opp_hp, "result": result, "note": note,
        }, ensure_ascii=False) + "\n")
    print(f"기록 완료: {path}  내HP {my_hp} 상대HP {opp_hp} 결과 {result}")


def summarize(path: Path) -> None:
    """받아적은 것만 보고한다 — 기하도 데미지도 계산하지 않는다."""
    hdr = out = None
    n = 0
    fi = []
    tm = []
    for line in path.open(encoding="utf-8"):
        try:
            o = json.loads(line)
        except json.JSONDecodeError:
            continue                      # 기록 중 끊긴 마지막 행
        k = o.get("kind")
        if k == "header":
            hdr = o
        elif k == "outcome":
            out = o
        elif k == "frame":
            n += 1
            fi.append(o["frame_index"])
            tm.append(o["t_mono"])
    print("=" * 88)
    print(f"[교전 기록] {path.name}")
    if hdr:
        print(f"  메모: {hdr.get('note','')}")
    print(f"  프레임 {n}")
    if n >= 2:
        span = tm[-1] - tm[0]
        print(f"  클라 시계 {tm[0]:.2f} ~ {tm[-1]:.2f}s  (폭 {span:.2f}s)")
        print(f"  서버 프레임 번호 {fi[0]} ~ {fi[-1]}  (증가 {fi[-1]-fi[0]})")
        if span > 0:
            print(f"  수신율 {n/span:.1f} 프레임/s   프레임번호 증가율 "
                  f"{(fi[-1]-fi[0])/span:.1f} /s")
        gaps = sum(1 for a, b in zip(fi, fi[1:]) if b - a != 1)
        print(f"  프레임 번호 불연속 {gaps}회  (0이면 손실 없음)")
    if out:
        print(f"  결과(화면 판독): 내HP {out['my_hp']}  상대HP {out['opp_hp']}  "
              f"{out['result']}")
    else:
        print("  결과 미기록 — --annotate 로 화면 HP를 남길 것")
    print("=" * 88)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--annotate", help="교전 기록 파일에 화면 HP/결과를 덧붙인다")
    ap.add_argument("--summary", help="교전 기록 파일 요약")
    ap.add_argument("--my-hp", type=float)
    ap.add_argument("--opp-hp", type=float)
    ap.add_argument("--result", choices=["win", "loss", "draw"])
    ap.add_argument("--note", default="")
    args = ap.parse_args()
    if args.annotate:
        annotate(Path(args.annotate), args.my_hp, args.opp_hp, args.result, args.note)
    elif args.summary:
        summarize(Path(args.summary))
    else:
        ap.error("--annotate 또는 --summary 중 하나가 필요하다")


if __name__ == "__main__":
    main()
