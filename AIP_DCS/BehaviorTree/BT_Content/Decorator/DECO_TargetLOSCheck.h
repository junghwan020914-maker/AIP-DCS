#pragma once
/*
	DECO_TargetLOSCheck - DECO_LOSCheck과 동일하되 내 조준각(Los_Degree) 대신
	상대가 나에 대해 가진 조준각(Los_Degree_Target)을 검사한다.
	08-04: 불리한 시작(시나리오4번 등)에서 DBFM에 갇혀 못 벗어나는 문제 진단 후 추가 —
	상대 조준이 일시적으로 풀리는 순간(their LOS 큼)을 감지해 반격 전환에 사용.
*/
#include "../../behaviortree_cpp_v3/action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../../../Geometry/Vector3.h"
#include "../Functions.h"
#include "../BlackBoard/CPPBlackBoard.h"

using namespace BT;

namespace Action
{
	class DECO_TargetLOSCheck : public SyncActionNode
	{
	public:
		DECO_TargetLOSCheck(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config) {}
		~DECO_TargetLOSCheck() {}

		static PortsList providedPorts();

		NodeStatus tick() override;
	};
}
