#include "DECO_StuckCheck.h"

namespace Action
{
	PortsList DECO_StuckCheck::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
			InputPort<std::string>("ExitLOS"),
			InputPort<std::string>("StuckSeconds")
		};
	}

	NodeStatus DECO_StuckCheck::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");
		Optional<std::string> ExitLOSPort = getInput<std::string>("ExitLOS");
		Optional<std::string> StuckSecondsPort = getInput<std::string>("StuckSeconds");

		float ExitLOS = std::stof(ExitLOSPort.value());
		double StuckSeconds = std::stod(StuckSecondsPort.value());

		double RunningTime = (*BB)->RunningTime;
		float CurrentLOS = (*BB)->Los_Degree_Target;

		// 최초 틱(노드 생성 직후, sentinel -1e9 오염 방지) 또는 에피소드 리셋(RunningTime이
		// 이전 틱보다 뒤로 감) 감지 시 타이머를 "지금"으로 리셋.
		if (!Initialized || RunningTime < LastTickRunningTime - 1e-6)
		{
			LastFreeTime = RunningTime;
			Initialized = true;
		}
		LastTickRunningTime = RunningTime;

		if (CurrentLOS >= ExitLOS)
		{
			// 상대가 조준을 놓친 순간 - 즉시 반격(구 DECO_TargetLOSCheck의 wobble감지 흡수) +
			// 스턱 타이머도 함께 리셋. 이 노드가 매틱 무조건 평가되므로 이 분기가 실제로 실행됨.
			std::fprintf(stdout, "[WOBBLE_FIRE] team=%d losTgt=%.2f\n", (int)(*BB)->Team, CurrentLOS);
			std::fflush(stdout);
			LastFreeTime = RunningTime;
			return NodeStatus::SUCCESS;
		}

		if ((RunningTime - LastFreeTime) >= StuckSeconds)
		{
			std::fprintf(stdout,
				"[STUCK_FIRE] team=%d t=%.2f stuckFor=%.2f losTgt=%.2f losMe=%.2f dist=%.1f bfm=%d\n",
				(int)(*BB)->Team, RunningTime, RunningTime - LastFreeTime, CurrentLOS,
				(*BB)->Los_Degree, (*BB)->Distance, (int)(*BB)->BFM);
			std::fflush(stdout);
			return NodeStatus::SUCCESS;
		}

		return NodeStatus::FAILURE;
	}
}
