#include "FlatPursuit.h"

namespace Action
{
	PortsList FlatPursuit::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
			// 이 거리(m) 밖에서만 발동. 사거리 안에서는 표적을 그대로 겨눈다.
			InputPort<double>("DistBeyond", 914.0, "engage only beyond this range (m)"),
			// 표적이 나보다 이만큼(m) 넘게 위에 있을 때만 발동.
			InputPort<double>("TargetAboveMin", 300.0, "engage only if target is this much higher (m)"),
			// 조준점 상승 성분 **상한** = 거리 * 이 값 (수직 자제 모드). 0 이하면 미적용.
			InputPort<double>("MaxClimbSlope", 0.3, "aim point rise CAP = Distance * this; <=0 disables"),
			// 조준점 상승 성분 **하한** = 거리 * 이 값 (수직 추격 모드). 0 이하면 미적용.
			// 상대가 위에 있을 때 최소 이 각도로는 올라가 고도(perch)를 잡으러 간다.
			InputPort<double>("MinClimbSlope", 0.0, "aim point rise FLOOR = Distance * this; <=0 disables"),
			// 비에너지(v^2/2 + g*h)가 상대보다 이만큼(J/kg) 이상일 때만 발동. 0이면 무조건.
			// 에너지가 없는데 올라가면 높고 느려져 오히려 먹잇감이 된다.
			InputPort<double>("MinEnergyMargin", 0.0, "engage only if my specific energy exceeds target's by this")
		};
	}

	NodeStatus FlatPursuit::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");

		const double distBeyond    = getInput<double>("DistBeyond").value();
		const double tgtAboveMin   = getInput<double>("TargetAboveMin").value();
		const double maxClimbSlope = getInput<double>("MaxClimbSlope").value();
		const double minClimbSlope = getInput<double>("MinClimbSlope").value();
		const double minEnergyMargin = getInput<double>("MinEnergyMargin").value();

		Vector3 my  = (*BB)->MyLocation_Cartesian;
		Vector3 tgt = (*BB)->TargetLocaion_Cartesian;
		const double dist = (*BB)->Distance;

		// 자체 게이트 — 하나라도 어긋나면 아래 분기에 넘긴다.
		if (dist <= distBeyond)              return NodeStatus::FAILURE;	// 사거리 안이면 겨눈다
		if ((tgt.Z - my.Z) <= tgtAboveMin)   return NodeStatus::FAILURE;	// 위에 있지 않다

		// 비에너지 비교 — 올라가려면 에너지가 있어야 한다.
		// E = v^2/2 + g*h  (단위질량당). 없는데 올라가면 높고 느려져 먹잇감이 된다.
		if (minEnergyMargin > 0.0)
		{
			const double g = 9.80665;
			const double vMy = (double)(*BB)->MySpeed_MS;
			const double vTg = (double)(*BB)->TargetSpeed_MS;
			const double eMy = 0.5 * vMy * vMy + g * my.Z;
			const double eTg = 0.5 * vTg * vTg + g * tgt.Z;
			if ((eMy - eTg) < minEnergyMargin) return NodeStatus::FAILURE;
		}

		// 방위는 표적 그대로 두고 상승 성분만 조정한다.
		Vector3 VP = tgt;
		if (minClimbSlope > 0.0)					// 수직 추격: 최소 이만큼은 올라간다
		{
			const double floorZ = my.Z + dist * minClimbSlope;
			if (VP.Z < floorZ) VP.Z = floorZ;
		}
		if (maxClimbSlope > 0.0)					// 수직 자제: 이 이상은 안 올라간다
		{
			const double capZ = my.Z + dist * maxClimbSlope;
			if (VP.Z > capZ) VP.Z = capZ;
		}

		(*BB)->VP_Cartesian = VP;

		// 폐쇄율 관리는 평소대로 — 여기서 전추력을 강제하면 오히려 관통한다.
		BTFunc::ApplyClosureThrottle(*BB);

		return NodeStatus::SUCCESS;
	}
}
