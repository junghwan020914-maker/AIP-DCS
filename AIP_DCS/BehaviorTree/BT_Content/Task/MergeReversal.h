#pragma once
/*
	MergeReversal - ryujan0208/Topgun 팀(Task_MergeReversal.cpp, 2026-07-20 신설)에서 이식.
	정면 엇갈림(merge) 직후 상대가 내 뒤로 넘어가는 순간(LOS 100~150도), 큰 원으로 늘어지지
	말고 상대 쪽으로 급선회 + 감속(선회반경 축소)해서 빨리 꼬리를 무는 반전 기동.
	VP를 상대 위치에 그대로 찍으면 LOS>90이라 CPPBehaviorTree.cpp의 boresight 75도 클램프가
	자동으로 최대선회 명령으로 바꿔준다(별도 롤 로직 불필요).

	07-31: 우리와 ryujan, dhks1573(Trinity) 세 팀 모두 독립적으로 "근접 각도전은 폐쇄율
	제어(throttle)로 못 푼다, 기하학적 해법(reversal/lag)이 필요하다"는 결론에 도달함.
	이 노드가 그 방향의 시도 — 단, ryujan의 실전배치(v22c/Rule_v2.xml)에도 아직 안 쓰이는
	미검증 상태였음(가설 단계). A/B로 직접 검증 필요.
*/
#include "../../behaviortree_cpp_v3\action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../../../Geometry/Vector3.h"
#include "../Functions.h"
#include "../BlackBoard/CPPBlackBoard.h"

using namespace BT;

namespace Action
{
	class MergeReversal : public SyncActionNode
	{
	public:
		MergeReversal(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config) {}
		~MergeReversal() {}

		static PortsList providedPorts();
		NodeStatus tick() override;
	};
}
