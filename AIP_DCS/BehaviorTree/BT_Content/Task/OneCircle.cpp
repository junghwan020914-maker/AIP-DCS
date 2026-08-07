#include "OneCircle.h"

namespace Action
{
	PortsList OneCircle::providedPorts()
	{
		return { InputPort<CPPBlackBoard*>("BB") };
	}

	NodeStatus OneCircle::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");

		Vector3 my  = (*BB)->MyLocation_Cartesian;
		Vector3 tgt = (*BB)->TargetLocaion_Cartesian;
		Vector3 F = (*BB)->MyForwardVector; F.normalize();
		Vector3 R = (*BB)->MyRightVector;   R.normalize();

		// 적기 쪽(nose-to-nose)으로 수평 하드턴 -> 반경 최소화
		Vector3 toT = tgt - my;
		double side = (toT.dot(R) >= 0.0) ? 1.0 : -1.0;

		// 전방 성분은 작게, 옆(적 방향) 성분을 크게 -> 아주 타이트한 수평선회
		(*BB)->VP_Cartesian = my + F * 800.0 + R * (side * 5000.0);

		return NodeStatus::SUCCESS;
	}
}
