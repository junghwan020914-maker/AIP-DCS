#pragma once
/*
	DECO_StuckCheck - 상대 조준각이 순간적으로 풀리는 wobble뿐 아니라, 잘 훈련된 상대
	(v5/v29 08-05 진단)가 한 번 뒤를 잡으면 their_ata를 임계값 밑으로 수십초씩 계속
	유지해서 wobble 자체가 안 생기는 경우까지 함께 잡는다.

	08-05 버그수정: 원래 DECO_TargetLOSCheck(즉시 wobble 감지)의 Fallback 형제로 뒤에
	배치했었는데, Fallback은 실패해야 다음 자식을 틱하므로 이 노드는 "CurrentLOS<ExitLOS"
	일 때만 틱됨 -> 노드 내부의 "CurrentLOS>=ExitLOS면 리셋" 분기가 죽은 코드가 되어
	LastFreeTime이 절대 갱신 안 됨(선행 DBFM/HABFM 중첩판정과 완전히 같은 버그 패턴).
	그 결과 "3초 이상 지속 조준"이 아니라 "에피소드 시작 3초 이후부터 거의 항상 발동"으로
	퇴화해 전체 트리를 사실상 상시 Lead로 덮어써버림 - v29전 악화(무승부->패배)의 원인.
	수정: wobble 즉시감지까지 이 노드 하나로 흡수해서 매틱 무조건 평가되게 함(형제 없음).
*/
#include "../../behaviortree_cpp_v3\action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../../../Geometry/Vector3.h"
#include "../Functions.h"
#include "../BlackBoard/CPPBlackBoard.h"

using namespace BT;

namespace Action
{
	class DECO_StuckCheck : public SyncActionNode
	{
	private:
		double LastFreeTime = -1e9;
		double LastTickRunningTime = -1e9;
		bool Initialized = false;

	public:
		DECO_StuckCheck(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config) {}
		~DECO_StuckCheck() {}

		static PortsList providedPorts();

		NodeStatus tick() override;
	};
}
