#include "DECO_TimeCheck.h"

namespace Action
{
	PortsList DECO_TimeCheck::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
			InputPort<std::string>("UpDown"),
			InputPort<std::string>("Seconds")
		};
	}

	NodeStatus DECO_TimeCheck::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");
		Optional<std::string> UpOrDown = getInput<std::string>("UpDown");
		Optional<std::string> Sec = getInput<std::string>("Seconds");

		// RunningTime은 CPPBehaviorTree::RunCPPBT 에서 매틱 누산되며, 같은 함수에서
		// 리스폰(위치 100m 이상 점프)을 감지해 0으로 리셋한다. 주최측 reset()이 no-op이라
		// BT 인스턴스가 라운드 경계에서 재생성되지 않기 때문에 이 리셋이 없으면
		// RunningTime이 배치/다라운드 전체에 누적돼 항상 Phase3으로 오판한다.
		double Elapsed = (*BB)->RunningTime;
		std::string UD = UpOrDown.value();
		double InputSeconds = std::stod(Sec.value());

		if (UD == "Greater")
		{
			return (Elapsed >= InputSeconds) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
		}
		else if (UD == "Less")
		{
			return (Elapsed < InputSeconds) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
		}
		else
		{
			//UpDown 입력 문자열이 오타난건 아닌지 확인 필요!!!! Greater 나 Less 가 아님
			return NodeStatus::FAILURE;
		}
	}
}
