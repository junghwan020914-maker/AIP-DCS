#pragma once
/*
	Jinking - DBFM 최후방어. 예측 불가한 롤/피치 진동으로 적의 건 조준을 흐트러뜨린다.
	시간(RunningTime) 기반 사인파를 좌우/상하 이위상으로 섞어 추적점을 흔든다.
*/
#include "../../behaviortree_cpp_v3\action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../../../Geometry/Vector3.h"
#include "../Functions.h"
#include "../BlackBoard/CPPBlackBoard.h"
using namespace BT;
namespace Action
{
	class Jinking : public SyncActionNode
	{
	public:
		Jinking(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config) {}
		~Jinking() {}
		static PortsList providedPorts();
		NodeStatus tick() override;
	};
}
