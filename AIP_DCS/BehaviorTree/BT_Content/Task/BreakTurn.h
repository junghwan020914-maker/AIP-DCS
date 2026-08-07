#pragma once
/*
	BreakTurn - DBFM(방어) 기동. 위협을 향해 최대선회로 당겨 3-9 라인(beam)으로 밀어내고
	오버슛을 유도한다. 코너속도 유지를 위해 약간 기수 아래로 당긴다.
	VP = 내위치 + 전방*a + (적방향 좌우)*b - 아래*c

	진입 시점(월드프레임)에 방향을 한 번 래치(latch)한다. 매틱 현재 자세로 방향을
	재계산하면, 기체가 돌아서 조준각을 좁히는 순간 VP도 같이 돌아버려 절대 수렴하지
	않는 자기추적(self-chasing) 문제가 생기기 때문 — 실측(2026-07-17)으로 off-boresight
	각이 롤이 진행돼도 전혀 안 줄어드는 현상으로 확인됨.
*/
#include "../../behaviortree_cpp_v3\action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../../../Geometry/Vector3.h"
#include "../Functions.h"
#include "../BlackBoard/CPPBlackBoard.h"
using namespace BT;
namespace Action
{
	class BreakTurn : public SyncActionNode
	{
	private:
		// 08-05 실험: 순수 회피(dir)에 목표 방향(래치 시점 기준)을 소량 섞어서, 회피는
		// 유지하되 시간이 지날수록 조준각이 조금씩이라도 좁혀지도록 함. 0=기존과 동일.
		BT_Geometry::Vector3 LatchedDir;
		bool HasLatch = false;
		double LastTickTime = -1e9;
		double NextRelatchTime = -1e9;
		BT_Geometry::Vector3 LastKnownLocation;
		bool HasLastLocation = false;

		// 08-06 실험: 재래치할 때마다 진짜로 조준각(Los_Degree)이 좋아지고 있는지 확인해서,
		// 여러 번 연속으로 개선이 없으면 그 다음 래치부터 블렌드 비율을 점점 올린다.
		// StuckCheck/RelatchSeconds와 달리 기동을 바꾸거나 주기를 통째로 당기는 게 아니라
		// "같은 BreakTurn 안에서, 정체가 확인될 때만" 강도를 조절 — v29/v32에서 역효과였던
		// "상시 더 공격적으로"와는 다른 접근.
		double LastRelatchLosMe = -1.0;
		int StuckStreak = 0;

	public:
		BreakTurn(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config) {}
		~BreakTurn() {}
		static PortsList providedPorts();
		NodeStatus tick() override;
	};
}
