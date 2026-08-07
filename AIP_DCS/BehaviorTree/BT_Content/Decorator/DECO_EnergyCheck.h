#pragma once
/*
	DECO_EnergyCheck - 내 비에너지가 타겟보다 우세/열세인지 판정.
	Fig.3의 "High Energy?" 블록에 해당.
	포트: UpDown("Greater"/"Less"), Margin(m, 기본 0)
	  Greater : MySpecificEnergy >= TargetSpecificEnergy + Margin 이면 SUCCESS
	  Less    : MySpecificEnergy <= TargetSpecificEnergy - Margin 이면 SUCCESS
*/
#include "../../behaviortree_cpp_v3\action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../BlackBoard/CPPBlackBoard.h"
using namespace BT;
namespace Action
{
	class DECO_EnergyCheck : public SyncActionNode
	{
	public:
		DECO_EnergyCheck(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config) {}
		~DECO_EnergyCheck() {}
		static PortsList providedPorts();
		NodeStatus tick() override;
	};
}
