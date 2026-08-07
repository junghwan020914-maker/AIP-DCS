#pragma once
/*
	WEZUpdate - WEZ(Weapon Engagement Zone) 진입 여부를 계산하는 서비스 노드.
	논문2 Fig.10 채점 기준: 기수 앞 2° 원뿔, 거리 500~3000ft.
	  TargetInMyWEZ  : 적이 내 WEZ 안 (내 LOS<반각 AND 거리 범위) -> 내가 사격/득점 가능
	  IamInTargetWEZ : 내가 적 WEZ 안 (적 LOS<반각 AND 거리 범위) -> 내가 피격 위험

	포트: ConeDeg(원뿔 반각, 기본 1.0=2°총각), MinM(하한, 기본 152m=500ft), MaxM(상한, 기본 914m=3000ft)
*/
#include "../../behaviortree_cpp_v3\action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../BlackBoard/CPPBlackBoard.h"
using namespace BT;
namespace Action
{
	class WEZUpdate : public SyncActionNode
	{
	public:
		WEZUpdate(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config) {}
		~WEZUpdate() {}
		static PortsList providedPorts();
		NodeStatus tick() override;
	};
}
