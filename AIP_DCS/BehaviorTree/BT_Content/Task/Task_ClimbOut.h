#pragma once
/*
	Task_ClimbOut - 최저고도 미만일 때 즉시 상승해 고도(=에너지)를 회복하는 기동.
	DECO_AltitudeCheck 게이트와 함께 트리 최우선에 두어, BFM 판정보다 먼저
	"일단 고도부터 확보"하게 한다. 기수 방향(수평)은 유지하고 크게 상승.
*/
#include "../../behaviortree_cpp_v3\action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../../../Geometry/Vector3.h"
#include "../BlackBoard/CPPBlackBoard.h"
using namespace BT;
namespace Action
{
	class Task_ClimbOut : public SyncActionNode
	{
	public:
		Task_ClimbOut(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config) {}
		~Task_ClimbOut() {}
		static PortsList providedPorts();
		NodeStatus tick() override;
	};
}
