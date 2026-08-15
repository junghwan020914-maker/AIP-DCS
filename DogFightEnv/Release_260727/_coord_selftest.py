"""DLL이 **대회 경로에서 좌표를 올바로 읽는지** 판정한다. PC·하네스 무관, 30초.

08-15 배경. 실서버 교전에서 ryujan v42가 내부 거리 `1.05657e+08`(1억 미터)을 찍으며
200초 내내 우리 쪽으로 기수를 못 돌렸다(ATA 최소 57.4도). 우리와 arcE는 같은 경로에서
정상이었다. 그런데 팀에서는 "머신에 따라 다르다, yuno PC에서는 우리 것도 터진다"는
보고가 나왔다. **말로 다툴 문제가 아니라 각자 재서 숫자를 대면 끝나는 문제다.**

## 원인 (주최측 26-05-25 답변으로 확정)

배포 템플릿 `CPPBehaviorTree::Step()`에 이 줄이 있었다:

    Vector3 Mylocation_Cartesian = LLAtoCartesian(MyInfo.Location, Vector3(OriLAT, OriLOn, 0));

주최측: **"이 부분은 지워야 했는데 까먹고 안지운 부분입니다."**
       **"송신은 Cartesian 좌표계로만 이뤄지며 LLA 좌표계는 무시하시면 됩니다."**

즉 서버는 항상 Cartesian 미터를 보낸다. 그걸 위경도로 오해해 다시 변환하면
`dlat = 608.5 - 37.91 = 570.6`을 "도"로 읽어 `(M+h) * 570.6 * pi/180` = 약 6.3e7 m가 된다.
**기체가 눈이 먼 채로 경기한다. 그리고 로컬 배치 테스트로는 안 잡힌다** —
로컬은 `ChangeData`가 원시 위경도를 변환해 넣어주므로 항상 올바른 값이 들어간다.

## 재는 방법

제출물(DLL)을 **대회와 같은 경로**로 부른다 — `oPlaneData`에 Cartesian 미터를 담아
`Step()`에 직접 넣고(= `StepWithPlaneData`), 나온 추적점(VP)이 기체 근처인지 본다.

  · 정상   : VP가 기체에서 수 km 이내
  · 변환됨 : VP가 1e5 m 이상 떨어짐 (실측 사례는 1e8)

판정 기준을 미리 박는다: **|VP - 내위치| > 1e5 m 이면 FAIL.** 정상 교전에서 추적점이
100 km 밖에 찍힐 이유가 없다(WEZ 최대 사거리가 914 m다).

⚠️ 이 시험은 **DLL 파일 자체의 성질**을 잰다. 같은 DLL이 PC마다 다른 결과를 낸다면
   그건 DLL이 아니라 **하네스 배선**(Cartesian을 `ChangeData`에 넣는 등) 문제다.
   그 경우도 이 시험이 갈라준다 — DLL은 PASS인데 실교전만 터지면 배선을 보면 된다.

사용:
    python _coord_selftest.py                                  # 있는 DLL 전부
    python _coord_selftest.py --dlls AIP_DCS.dll,AIP_v42.dll
Not part of the student template; safe to delete after use.
"""
from __future__ import annotations

import argparse
import ctypes as ct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
for p in (ROOT, ROOT / "src"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from dogfight.ai.native_bt import AIPilot, OPlaneData  # noqa: E402

FAIL_M = 1.0e5          # 사전 등록 기준
CASES = [
    ("원점 근처 1km",      (0.0, 0.0, 5000.0),        (1000.0, 0.0, 5000.0)),
    ("원점에서 25km 떨어짐", (20000.0, -15000.0, 4500.0), (20850.0, -14150.0, 4600.0)),
    ("대회 시작기하 2000ft", (608.5, 6.1, 4572.0),      (0.1, -7.3, 4572.0)),
]


def plane(pos, yaw, pid, team, hp=100.0, speed=200.0):
    d = OPlaneData()
    d.LocationX, d.LocationY, d.LocationZ = pos
    d.Roll, d.Pitch, d.Yaw = 0.0, 0.0, yaw
    d.Speed = speed
    d.Team = team
    d.Resv0, d.Resv1, d.Resv2 = pid, hp, 0.0
    return d


def test_dll(name: str) -> bool:
    path = ROOT / name
    if not path.exists():
        print(f"  {name:<26} (없음)")
        return True
    try:
        bt = AIPilot(name)
        bt.CreateBehaviorTree(1, 1)
    except Exception as exc:                      # noqa: BLE001
        print(f"  {name:<26} 🔴 로드/트리생성 실패: {type(exc).__name__}: {exc}")
        return False

    worst = 0.0
    rows = []
    for label, mypos, foepos in CASES:
        me = plane(mypos, 90.0, 1, 1)
        foe = plane(foepos, 270.0, 2, 2)
        try:
            for _ in range(3):                    # 트리가 상태를 잡도록 몇 틱
                bt.StepWithPlaneData(me, foe)
            vp = bt.GetVPWithPlaneData(me)
        except Exception as exc:                  # noqa: BLE001
            print(f"  {name:<26} 🔴 {label}: 호출 실패 {type(exc).__name__}: {exc}")
            return False
        dx = float(vp.X) - mypos[0]
        dy = float(vp.Y) - mypos[1]
        dz = float(vp.Z) - mypos[2]
        dist = (dx * dx + dy * dy + dz * dz) ** 0.5
        worst = max(worst, dist)
        rows.append((label, vp, dist))

    ok = worst <= FAIL_M
    print(f"  {name:<26} {'PASS' if ok else '🔴 FAIL'}   최대 |VP-내위치| = {worst:.3e} m")
    for label, vp, dist in rows:
        mark = " " if dist <= FAIL_M else "*"
        print(f"     {mark} {label:<22} VP=({vp.X:12.1f},{vp.Y:12.1f},{vp.Z:10.1f})  "
              f"거리 {dist:.3e} m")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dlls", help="쉼표 구분. 없으면 폴더의 AIP_*.dll 전부")
    args = ap.parse_args()
    names = ([x.strip() for x in args.dlls.split(",") if x.strip()] if args.dlls
             else sorted(p.name for p in ROOT.glob("AIP_*.dll")))

    print("=" * 96)
    print("[좌표 규약 자체시험]  대회 경로(Cartesian -> Step)로 DLL을 직접 호출한다")
    print(f"  판정: |VP - 내위치| > {FAIL_M:.0e} m 이면 FAIL "
          f"(WEZ 최대 사거리가 914 m다. 100 km 밖에 추적점이 찍힐 이유가 없다)")
    print(f"  주최측 26-05-25: \"송신은 Cartesian 좌표계로만 이뤄지며 LLA는 무시하시면 됩니다\"")
    print()
    bad = [n for n in names if not test_dll(n)]
    print()
    if bad:
        print(f"  🔴 FAIL {len(bad)}개: {', '.join(bad)}")
        print("     -> Step()에서 LLAtoCartesian을 지울 것. 주최측이 '지웠어야 할 줄'이라고 한 그 줄이다.")
        print("     -> 이 상태로 제출하면 대회 경로에서 기체가 눈이 먼다.")
    else:
        print(f"  ✅ 전부 PASS ({len(names)}개)")
    print("  같은 DLL이 PC마다 다르게 나오면 DLL이 아니라 하네스 배선 문제다")
    print("  (Cartesian을 ChangeData에 넣으면 원시 위경도로 오해해 같은 폭발이 난다).")
    print("=" * 96)
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
