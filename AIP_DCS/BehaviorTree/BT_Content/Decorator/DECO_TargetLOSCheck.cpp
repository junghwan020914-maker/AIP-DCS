#include "DECO_TargetLOSCheck.h"

// #define WOBBLE_DBG_TRACE	// uncomment for one-off [WOBBLE_FIRE] per-tick trace

namespace Action
{
	PortsList DECO_TargetLOSCheck::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
			InputPort<std::string>("UpDown"),
			InputPort<std::string>("InputLOS")
		};
	}

	NodeStatus DECO_TargetLOSCheck::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");
		Optional<std::string> UpOrDown = getInput<std::string>("UpDown");
		Optional<std::string> Dist = getInput<std::string>("InputLOS");

		float CurrentLOS = (*BB)->Los_Degree_Target;
		std::string UD = UpOrDown.value();
		float InputLOS = std::stof(Dist.value());

		if (UD == "Greater")
		{
			if (CurrentLOS >= InputLOS)
			{
				// 08-07: 이 노드는 최상위 반격분기 조건(상대LOS>=45)이라 교전 대부분의 틱에서
				// SUCCESS가 난다. 여기 있던 무조건 fprintf+fflush는 매 틱 stdout을 동기
				// 플러시해서, 60Hz·CMD 16ms 응답 제약과 "AI 순수연산 0.1667초 초과시 패널티"
				// 규정에 직접 걸릴 수 있는 프로덕션 결함이었다. 매크로로 가둔다.
#ifdef WOBBLE_DBG_TRACE
				std::fprintf(stdout, "[WOBBLE_FIRE] team=%d losTgt=%.2f\n", (int)(*BB)->Team, CurrentLOS);
				std::fflush(stdout);
#endif
				return NodeStatus::SUCCESS;
			}
			else
			{
				return NodeStatus::FAILURE;
			}
		}
		else if (UD == "Less")
		{
			if (CurrentLOS <= InputLOS)
			{
				return NodeStatus::SUCCESS;
			}
			else
				return NodeStatus::FAILURE;
		}
		else
		{
			return NodeStatus::FAILURE;
		}
	}
}
