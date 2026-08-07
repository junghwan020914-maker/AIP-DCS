#pragma once
/*
	DECO_AltitudeCheck - 내 고도가 최저고도(MinAlt) 미만인지 판정.
	미만이면 SUCCESS -> 상위 Fallback에서 ClimbOut을 최우선 발동시키는 게이트로 사용.
	(에너지=고도. 사후 보정(PreventLandCrash)보다 먼저 개입해 고도를 지키는 것이 목적)

	포트: MinAlt (m, 기본 3000)
*/
#include "../../behaviortree_cpp_v3\action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../BlackBoard/CPPBlackBoard.h"
using namespace BT;
namespace Action
{
	class DECO_AltitudeCheck : public SyncActionNode
	{
	public:
		DECO_AltitudeCheck(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config) {}
		~DECO_AltitudeCheck() {}
		static PortsList providedPorts();
		NodeStatus tick() override;
	};
}
