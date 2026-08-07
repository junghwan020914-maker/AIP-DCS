#include "Task_ClimbOut.h"

namespace Action
{
	PortsList Task_ClimbOut::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
			InputPort<double>("ClimbGain", 8000.0, "vertical reach of the climb VP (m)")
		};
	}

	NodeStatus Task_ClimbOut::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");
		double ClimbGain = getInput<double>("ClimbGain").value();

		Vector3 my = (*BB)->MyLocation_Cartesian;

		// 수평 기수 방향 유지(고도만 확보, 방향은 안 흔듦)
		Vector3 F = (*BB)->MyForwardVector;
		Vector3 FH(F.X, F.Y, 0.0);
		if (FH.length() < 1e-3) FH = Vector3(1, 0, 0);
		FH.normalize();

		Vector3 WorldUp(0.0, 0.0, 1.0);

		// 전방은 짧게, 위로 크게 -> 가파른 상승
		(*BB)->VP_Cartesian = my + FH * 1500.0 + WorldUp * ClimbGain;

		// 상승엔 추력이 필요하므로 최대 스로틀
		(*BB)->Throttle = 1.0f;

		return NodeStatus::SUCCESS;
	}
}
