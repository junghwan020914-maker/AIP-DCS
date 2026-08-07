#pragma once
/*
	Lag Pursuit (후방 추적) 기동 Task 노드

	적기의 "꼬리 뒤쪽"을 조준한다. 적기 진행 방향의 반대쪽(후미)을 겨냥하여
	오버슛을 방지하고 에너지(속도)를 보존하면서 적기 6시 방향으로 진입할 때 사용.

	VP = 적기현재위치 - 적기진행방향 * LagDistance

	- LagDistance : 적기 후미에서 얼마나 뒤를 겨냥할지(m). 포트로 조절.
	                기본값 0 이면 적기와의 거리의 절반(Distance * 0.5)을 자동 사용.
	- 장점 : 오버슛 억제, 선회반경 안쪽 유지, 에너지 보존
	- 단점 : 조준은 풀리므로 사격 직전에는 Pure/Lead로 전환 필요
*/
#include "../../behaviortree_cpp_v3\action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../../../Geometry/Vector3.h"
#include "../Functions.h"
#include "../BlackBoard/CPPBlackBoard.h"

using namespace BT;

namespace Action
{
	class Lag : public SyncActionNode
	{
	private:


	public:

		Lag(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
		{
		}

		~Lag()
		{
		}

		static PortsList providedPorts();

		NodeStatus tick() override;
	};
}
