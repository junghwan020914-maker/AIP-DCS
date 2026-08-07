#pragma once
/*
	TwoCircle - HABFM 정면교전. 적기와 같은 회전 방향(nose-to-tail)으로 선회하여 두 개의 원을
	만드는 선회율(rate) 싸움. 적기 기수가 향하는 쪽으로 지속 최대선회하여 각속도로 우위를 노린다.
	(에너지 우세/높은 선회율 상황에서 유리)
*/
#include "../../behaviortree_cpp_v3\action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../../../Geometry/Vector3.h"
#include "../Functions.h"
#include "../BlackBoard/CPPBlackBoard.h"
using namespace BT;
namespace Action
{
	class TwoCircle : public SyncActionNode
	{
	public:
		TwoCircle(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config) {}
		~TwoCircle() {}
		static PortsList providedPorts();
		NodeStatus tick() override;
	};
}
