#pragma once
/*
	Sprint (조준 데드밴드 가속) 기동 Task 노드

	사거리 밖에서 이미 조준이 맞아 있으면 G를 당길 이유가 없다 — 그건 에너지 낭비다.
	VP를 **내 기수 정면**에 찍어 제어기가 보는 LOS를 0으로 만들고, 전추력으로 가속한다.

	배경(08-10 계측):
	  제어기 피치 명령은 ERROR_Effect = LOS/6 + clamp(LOSErrorSUM/7.5, 0, 0.25) + D 인데
	  가운데 항이 적분기가 아니라 **최근 60샘플 평균**이라 정상 추격에서 0.25로 영구 포화한다.
	  즉 LOS 1도에서도 0.42를 당긴다 — **명령이 0으로 내려갈 방법이 없어** 조준점을 반드시
	  지나치고, 그 상시 선회가 에너지를 갉아먹는다.
	  실측: 전추력 수평비행 505m/s가 가능한데(`_thrust_probe.py`) 추격 실속도는 421m/s.
	        0.70스로틀 직진 표적(407m/s)조차 못 따라잡아 duck 6시드 중 4판 무득점.

	이 노드는 제어기를 고치는 대신 **트리에서 데드밴드를 만들어** 우회한다.
	제어기 인터페이스가 VP뿐이라 "당기지 마라"를 표현하는 유일한 방법이 VP를 기수에
	얹는 것이다.

	적용 조건(XML에서 게이트): 사거리 밖 + LOS 작음. 사거리 안에서는 조준이 곧 득점이므로
	절대 걸면 안 된다.
*/
#include "../../behaviortree_cpp_v3\action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../../../Geometry/Vector3.h"
#include "../Functions.h"
#include "../BlackBoard/CPPBlackBoard.h"

using namespace BT;

namespace Action
{
	class Sprint : public SyncActionNode
	{
	private:


	public:

		Sprint(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
		{
		}

		~Sprint()
		{
		}

		static PortsList providedPorts();

		NodeStatus tick() override;
	};
}
