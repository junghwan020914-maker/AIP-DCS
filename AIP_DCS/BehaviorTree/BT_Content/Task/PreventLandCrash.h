#pragma once
/*
	PreventLandCrash - 지면 충돌 방지(고도 보호) 노드.
	기동 노드가 VP(추적점)를 정한 뒤 트리 맨 끝에서 실행되어, 고도가 낮으면
	VP를 상승 방향으로 덮어써서 death spiral/지면 충돌을 막는다.
	논문(Fig.5 조종제어 모델)의 PreventLandCrash 기능에 해당.

	FloorHard : 이 고도(m) 미만이면 무조건 강한 상승 VP로 덮어씀
	FloorSoft : 이 고도(m) 미만이면 기존 VP를 상승 VP 쪽으로 비례 블렌딩
	(그 이상 고도에서는 아무 것도 하지 않음)
*/
#include "../../behaviortree_cpp_v3\action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../../../Geometry/Vector3.h"
#include "../Functions.h"
#include "../BlackBoard/CPPBlackBoard.h"
using namespace BT;
namespace Action
{
	class PreventLandCrash : public SyncActionNode
	{
	public:
		PreventLandCrash(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config) {}
		~PreventLandCrash() {}
		static PortsList providedPorts();
		NodeStatus tick() override;
	};
}
