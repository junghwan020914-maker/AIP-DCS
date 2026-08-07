#pragma once
/*
	Scissors - HABFM 근접 저속 엇갈림. 적기를 향해 반복적으로 되말아(reverse) 오버슛을
	유도하고 속도를 죽여 상대를 앞세운다. 항상 적기 쪽으로 되돌리면서 상하 진동을 더한다.
*/
#include "../../behaviortree_cpp_v3\action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../../../Geometry/Vector3.h"
#include "../Functions.h"
#include "../BlackBoard/CPPBlackBoard.h"
using namespace BT;
namespace Action
{
	class Scissors : public SyncActionNode
	{
	public:
		Scissors(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config) {}
		~Scissors() {}
		static PortsList providedPorts();
		NodeStatus tick() override;
	};
}
