#include "Pure.h"

namespace Action
{
	PortsList Pure::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB")
		};
	}

	NodeStatus Pure::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");

		// 추적점(VP) = 적기의 현재 위치. 내 기수를 적기에 그대로 조준한다.
		(*BB)->VP_Cartesian = (*BB)->TargetLocaion_Cartesian;

		// 폐쇄율(dV) 관리 — 조준은 되는데 사거리를 관통해버리는 문제(사격전환 벽) 방지.
		BTFunc::ApplyClosureThrottle(*BB);

		return NodeStatus::SUCCESS;
	}
}
