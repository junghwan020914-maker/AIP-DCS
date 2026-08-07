#include "Scissors.h"
#include <cmath>

namespace Action
{
	PortsList Scissors::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
			InputPort<double>("Freq", 2.0, "vertical oscillation frequency")
		};
	}

	NodeStatus Scissors::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");
		double w = getInput<double>("Freq").value();

		Vector3 my  = (*BB)->MyLocation_Cartesian;
		Vector3 tgt = (*BB)->TargetLocaion_Cartesian;
		Vector3 F = (*BB)->MyForwardVector; F.normalize();
		Vector3 R = (*BB)->MyRightVector;   R.normalize();
		Vector3 U = (*BB)->MyUpVector;      U.normalize();

		// 항상 적기 쪽으로 되말기(reverse) -> nose-to-nose 반복
		Vector3 toT = tgt - my;
		double side = (toT.dot(R) >= 0.0) ? 1.0 : -1.0;

		double t = (*BB)->RunningTime;
		double vert = std::sin(t * w);

		// 전방은 짧게(속도 죽이기), 적 방향으로 강하게, 상하 진동
		(*BB)->VP_Cartesian = my + F * 800.0 + R * (side * 5000.0) + U * (1500.0 * vert);

		return NodeStatus::SUCCESS;
	}
}
