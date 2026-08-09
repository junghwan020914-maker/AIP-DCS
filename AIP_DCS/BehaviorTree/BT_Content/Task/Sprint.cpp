#include "Sprint.h"

namespace Action
{
	PortsList Sprint::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
			// 표적 쪽으로 섞는 비율(0=완전 직진, 1=Pure와 동일). 0이면 조준을 아예 놓으므로
			// 아주 약하게만 섞어 표류를 막는다.
			InputPort<double>("AimBlend", 0.0, "0 = fly straight, 1 = pure pursuit")
		};
	}

	NodeStatus Sprint::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");
		double blend = getInput<double>("AimBlend").value();
		if (blend < 0.0) blend = 0.0;
		if (blend > 1.0) blend = 1.0;

		Vector3 my = (*BB)->MyLocation_Cartesian;
		Vector3 fwd = (*BB)->MyForwardVector;
		fwd.normalize();

		double dist = (*BB)->Distance;
		if (dist < 1.0) dist = 1.0;

		// 기수 정면의 같은 거리 지점 = 제어기가 보는 LOS 0 (당김 명령 0)
		Vector3 straight = my + fwd * dist;
		Vector3 tgt = (*BB)->TargetLocaion_Cartesian;

		(*BB)->VP_Cartesian = straight * (1.0 - blend) + tgt * blend;

		// 목적이 가속이므로 폐쇄율 관리를 쓰지 않고 전추력을 그대로 준다.
		// (사거리 밖에서만 걸리는 노드라 관통 위험이 없다)
		(*BB)->Throttle = 1.0f;

		return NodeStatus::SUCCESS;
	}
}
