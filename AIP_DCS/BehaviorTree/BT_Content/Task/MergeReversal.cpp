#include "MergeReversal.h"

namespace Action
{
	PortsList MergeReversal::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB")
		};
	}

	NodeStatus MergeReversal::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");

		Vector3 MyLocation = (*BB)->MyLocation_Cartesian;
		Vector3 TargetLocation = (*BB)->TargetLocaion_Cartesian;
		Vector3 MyForward = (*BB)->MyForwardVector;

		double Distance = MyLocation.distance(TargetLocation);

		Vector3 MyToTarget = TargetLocation - MyLocation;
		float ata = (Distance > 1e-3)
			? (float)(MyToTarget.angleBetween(MyForward) * 57.2958)
			: 0.0f;

		// 자체 게이트(ryujan 원본과 동일): 엇갈림 반전 구간만. XML에서도 DECO로 같은
		// 범위를 걸어두지만, 이 노드 단독으로도 안전하게 동작하도록 원본 그대로 유지.
		if (Distance > 2500.0 || Distance < 80.0)  return NodeStatus::FAILURE;
		if (ata < 100.0f || ata > 150.0f)          return NodeStatus::FAILURE;

		// turn toward target; LOS>90 lets the boresight clamp convert this into max-rate turn
		Vector3 VP = TargetLocation;
		double climbSlope = Distance * 0.5;
		double diveSlope = Distance * 0.2;
		double minZ = MyLocation.Z - diveSlope;
		double maxZ = MyLocation.Z + climbSlope;
		if (VP.Z < minZ) VP.Z = minZ;
		if (VP.Z > maxZ) VP.Z = maxZ;
		if (VP.Z < 3500.0) VP.Z = 3500.0;
		(*BB)->VP_Cartesian = VP;

		// 감속으로 선회반경 축소
		(*BB)->Throttle = 0.65f;

		return NodeStatus::SUCCESS;
	}
}
