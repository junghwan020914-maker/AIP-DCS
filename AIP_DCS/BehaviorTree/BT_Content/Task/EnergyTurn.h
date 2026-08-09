#pragma once
/*
	EnergyTurn (하강 선회 = 슬라이스) 기동 Task 노드

	높고 느린 상태에서는 선회 성능이 최악이다. 고도를 속도로 바꾸면서 표적 쪽으로
	선회한다(descending turn / slice). 표적을 향한 선회는 유지하므로 각을 버리지 않는다.

	배경(08-10 `_stalemate_diag.py` + 초기조건 상관, 30시드 vs v29):
	  무득점 6판과 득점 24판이 **이분적으로** 갈린다 —
	      무득점: 공세 0.0% / 수세 42.2% / 공세밴드 0.0s
	      득점  : 공세53.2% / 수세  0.0% / 공세밴드11.8s
	  중간 지대가 없다. 즉 조준·접근 문제가 아니라 **위치 싸움 결과가 초반에 갈린다.**
	  (동선회 비율은 양쪽 다 56%로 판별력이 없어 러프베리 가설은 기각)

	  초기조건도 갈리는 것처럼 보였다:
	      무득점: 시작고도 중앙 6408m, 시작속도 중앙 220.5m/s
	      득점  : 시작고도 중앙 4306m, 시작속도 중앙 251.7m/s
	  ⚠️ **단, 이 상관은 통계적으로 확립되지 않았다.** 30개 중 6개를 고르는 모든 조합
	  (593,775개)에 대한 정확 순열검정에서 고도 p=0.177, 속도 p=0.213이다. n=6에서
	  이 정도 차이는 우연으로 흔하다. **가설로만 다루고 근거로 삼지 말 것.**
	  (반면 공세 0.0% vs 53.2%는 6판 전부 대 24판 전부의 범주적 분리라 통계 이전의 사실이다)

	  그래서 이 노드의 근거는 상관이 아니라 **BFM 원칙 자체**다: 코너속도 미만에서는
	  가속이 선회율과 선회반경을 동시에 개선하므로, 높고 느리면 고도를 속도로 바꾸는
	  것이 맞다. 6400m에서 TAS 220m/s는 IAS로 약 300kt로 F-16 코너속도(약 330~400kt)
	  미만이다. 현재 트리에는 에너지 관리 노드가 **하나도 없다**(ClimbOut은 arcE 전용).
	  효과 여부는 전수평가가 판정한다.

	❌ 08-10 1차 검증 **기각** (SpeedBelow=235 / AltAbove=3000 / DistBeyond=1000 /
	   DiveSlope=0.5 / FloorAlt=2000, 시간 제한 없음). 전수평가 승점 130.0 vs 기준선 144.5:
	       v32  28.5 -> 28.5    v29  26.5 -> **27.5**   arcA 30.0 -> 30.0
	       prev 29.5 -> 29.0    arcE 30.0 -> **15.0**
	   arcE 30판 **전부** 무승부·무득점·200초 만료다. `<1도` 0.00s, P1/P2/P3 전부 0.0% —
	   **밴드에 단 한 번도 못 들어갔다.** 우리가 슬라이스로 내려가는 동안 arcE는
	   ClimbOut으로 올라가 교전 자체가 성립하지 않았다. 수직으로 서로 멀어진 것이다.
	   v29에서 얻은 +1.0(패배 1 -> 0)은 진짜였지만 -15.0에 완전히 묻힌다.

	⚠️ **교훈: 에너지 기동은 상대의 에너지 기동과 상호작용한다.** 고도를 속도로 바꾸는
	   것이 단독으로는 옳아도, 상대가 반대 방향(상승)으로 가면 둘 다 교전을 잃는다.
	   BFM에서 수직 기동은 **상대를 따라가는 수단**이지 혼자 하는 것이 아니다.

	✅ **arcE(자작 에너지 파이터 원형)가 이 결함을 잡았다.** ryujan v29/v32와 arcA는
	   전부 동률이라 이 변경을 통과시켰을 것이다. 상대 풀에 **행동 원형**을 넣어둔 것이
	   실제로 값을 했다 — 실전 40시드에 에너지 파이터가 하나라도 있으면 그 판을 통째로
	   버릴 뻔했다. (같은 이유로 `_degenerate_probe.py`도 유지할 것)

	자체 게이트(적용 조건):
	  - 사거리 근처면 발동 안 함 — 거기선 싸워야지 내려가면 안 된다
	  - 이미 충분히 빠르면 발동 안 함
	  - 바닥 고도 여유가 없으면 발동 안 함 (규정 300m + PreventLandCrash와 충돌 방지)
*/
#include "../../behaviortree_cpp_v3\action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../../../Geometry/Vector3.h"
#include "../Functions.h"
#include "../BlackBoard/CPPBlackBoard.h"

using namespace BT;

namespace Action
{
	class EnergyTurn : public SyncActionNode
	{
	private:


	public:

		EnergyTurn(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
		{
		}

		~EnergyTurn()
		{
		}

		static PortsList providedPorts();

		NodeStatus tick() override;
	};
}
