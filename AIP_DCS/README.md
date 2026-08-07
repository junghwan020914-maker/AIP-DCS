# AIP Dogfight Behavior Tree

전투기 1대1 근접 공중전(WVR dogfight)을 위한 **규칙기반 AI**를 BehaviorTree.CPP v3로 구현한 프로젝트입니다.
"2026 AI Pilot Top Gun Challenge" 교전 환경(JSBSim 6-DOF)에 DLL로 연결되어 동작합니다.

관련 논문:
- 이민석 외, *공대공 전투 모의를 위한 규칙기반 AI 교전 모델 개발*, KIMST, 2022.
- Yang et al., *Manual-Based Automated Maneuvering Decisions for Air-to-Air Combat*, JAIS, 2024. (BEM 기반 1v1 dogfight flow)

## 아키텍처

```
시뮬(JSBSim) → LibMain.cpp Step() → UCPPBehaviorTree(BT tick)
   → VP(추적점) 생성 → StickController(조종값 Roll/Pitch/Rudder/Throttle) → 6-DOF
```

- 상황인식(Service 노드) → 국면 판정(BFMDecision) → 국면별 기동 선택 → VP 생성 → 지면충돌 보정.
- 트리 구조는 `Rule_forTraining.xml`에 정의(런타임 로드).

## BFM 국면과 기동 (논문 Fig.3 기반)

| 국면 | 판정 | 기동 노드 |
|---|---|---|
| DETECTING | 원거리(>9260m) | Pure |
| OBFM (공격/우세) | 조준 우위 or 에너지 우세 | Lag / Lead / Pure (+ WEZ 진입 시 사격유지) |
| HABFM (정면/대등) | 위치·에너지 대등 | One Circle / Two Circle / Scissors / Lead |
| DBFM (방어/열세) | 적이 내 후미 | Break Turn / Jinking |

- **WEZ 인식**: `WEZUpdate`가 채점 WEZ(기수 앞 2°, 500~3000ft) 진입 여부 판정.
- **지면충돌 방지**: `PreventLandCrash`가 저고도 시 상승 보정.

## 폴더 구조

```
BehaviorTree/
├── behaviortree_cpp_v3/     # BehaviorTree.CPP v3 (외부 라이브러리)
├── BT_Content/
│   ├── BlackBoard/          # CPPBlackBoard (공유 데이터)
│   ├── Service/             # 상황인식/판정: SelectTarget, *Update, BFMDecision, WEZUpdate
│   ├── Decorator/           # 조건: DECO_DistanceCheck, DECO_LOSCheck, DECO_WEZCheck, ...
│   └── Task/                # 기동: Pure/Lead/Lag, One/TwoCircle, Scissors, BreakTurn, Jinking, PreventLandCrash
│   └── CPPBehaviorTree.*    # 트리 로드/틱, 노드 등록
Geometry/                    # 벡터/행렬/쿼터니언 + StickController(제어기)
LibMain.cpp                  # DLL export (Step/CreateBehaviorTree/ChangeData/GetVP/...)
Rule_forTraining.xml         # 행동트리 규칙(XML)
```

## 빌드

Visual Studio(MSVC, x64)로 DLL 빌드:

```
MSBuild AIP_DCS.vcxproj -t:Build -p:Configuration=Debug -p:Platform=x64 -maxcpucount:1 -p:MultiProcessorCompilation=false
```

- 산출물: `../bin/debug.x64/AIP_DCS.dll`
- `..\PropertySheets\Essential_debug.props`(출력경로/런타임 설정)가 필요합니다.

## 주의사항 (개발 메모)

- **XML의 `<Sequence>`/`<Fallback>`에 `name=` 속성을 쓰지 말 것.** 본 프로젝트가 사용하는 파서는 부모가 컨트롤 노드인지를 이름 문자열로 판정하므로, `name=` 별칭을 붙이면 자식 노드가 연결되지 않아 트리가 동작하지 않습니다.
- 신규 소스 파일은 **UTF-8 BOM**으로 저장(MSVC의 CP949 오인식 방지).
- 새 노드는 `CPPBehaviorTree::init()`의 `registerNodeType`에 등록 + `.vcxproj`에 추가해야 합니다.

## 현재 상태

- 전체 BFM 기동 + WEZ 인식까지 동작. 미러 매치(자가 대전) 300초 완주 확인.
- 미해결 과제: 정면 교차(merge) 후 재교전(추격 수렴) 개선.
