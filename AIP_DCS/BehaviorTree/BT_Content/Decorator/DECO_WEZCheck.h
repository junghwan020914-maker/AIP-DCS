#pragma once
/*
	DECO_WEZCheck - WEZ 진입 여부 조건 노드. 논문2 block 7 "Hostile in WEZ?" 에 해당.
	포트 Side:
	  "Target" (기본) : 적이 내 WEZ 안이면 SUCCESS (내가 사격 가능)
	  "Me"            : 내가 적 WEZ 안이면 SUCCESS (내가 피격 위험)
*/
#include "../../behaviortree_cpp_v3\action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../BlackBoard/CPPBlackBoard.h"
using namespace BT;
namespace Action
{
	class DECO_WEZCheck : public SyncActionNode
	{
	public:
		DECO_WEZCheck(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config) {}
		~DECO_WEZCheck() {}
		static PortsList providedPorts();
		NodeStatus tick() override;
	};
}
