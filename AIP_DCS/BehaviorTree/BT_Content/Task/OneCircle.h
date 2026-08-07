#pragma once
/*
	OneCircle - HABFM 정면교전. 적기와 반대 방향(nose-to-nose)으로 선회하여 하나의 원을
	만드는 반경(radius) 싸움. 선회반경을 최소화하기 위해 수평으로 적기 쪽으로 강하게 당긴다.
	(작은 선회반경/낮은 에너지 상황에서 유리)
*/
#include "../../behaviortree_cpp_v3\action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../../../Geometry/Vector3.h"
#include "../Functions.h"
#include "../BlackBoard/CPPBlackBoard.h"
using namespace BT;
namespace Action
{
	class OneCircle : public SyncActionNode
	{
	public:
		OneCircle(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config) {}
		~OneCircle() {}
		static PortsList providedPorts();
		NodeStatus tick() override;
	};
}
