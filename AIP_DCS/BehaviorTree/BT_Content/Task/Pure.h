#pragma once
/*
	Pure Pursuit (순수 추적) 기동 Task 노드

	내 기수를 적기의 "현재 위치"로 직접 향하게 하는 가장 기본적인 추적.
	VP(추적점) = 적기의 현재 위치(Cartesian)

	- 장점 : 구현이 단순하고 적기를 바로 조준
	- 단점 : 적기가 기동하면 항상 뒤를 쫓게 되어(개꼬리 추적) 오버슛/에너지 손실 발생 가능
*/
#include "../../behaviortree_cpp_v3\action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../../../Geometry/Vector3.h"
#include "../Functions.h"
#include "../BlackBoard/CPPBlackBoard.h"

using namespace BT;

namespace Action
{
	class Pure : public SyncActionNode
	{
	private:


	public:

		Pure(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
		{
		}

		~Pure()
		{
		}

		static PortsList providedPorts();

		NodeStatus tick() override;
	};
}
