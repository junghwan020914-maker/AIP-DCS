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

	  그리고 초기조건이 갈린다:
	      무득점: 시작고도 중앙 6408m, 시작속도 중앙 220.5m/s
	      득점  : 시작고도 중앙 4306m, 시작속도 중앙 251.7m/s
	  높고(+2100m) 느린(-31m/s) 판을 진다. 6400m에서 TAS 220m/s는 IAS로 약 300kt로
	  **코너속도 미만**이고, 4300m의 252m/s는 약 390kt로 코너속도 근처다.
	  선회율이 가장 나쁜 에너지 상태로 시작하는 판이 그대로 진다는 뜻이다.

	  현재 트리에는 에너지 관리 노드가 **하나도 없다**(ClimbOut은 arcE 원형 전용).

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
