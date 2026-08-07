#include "PreventLandCrash.h"

namespace Action
{
	PortsList PreventLandCrash::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
			InputPort<double>("FloorHard", 1000.0, "hard pull-up altitude (m)"),
			InputPort<double>("FloorSoft", 2000.0, "soft blend altitude (m)")
		};
	}

	NodeStatus PreventLandCrash::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");
		double FloorHard = getInput<double>("FloorHard").value();
		double FloorSoft = getInput<double>("FloorSoft").value();

		Vector3 my  = (*BB)->MyLocation_Cartesian;
		double alt = my.Z;						// Z = 고도(up, m)

		if (alt >= FloorSoft)
			return NodeStatus::SUCCESS;			// 충분히 높음 -> 개입 안 함

		Vector3 WorldUp(0.0, 0.0, 1.0);

		// 수평 추격 방향(기동 VP 기준)은 살리되, 상승을 크게 실어 확실히 pull-up
		Vector3 cur = (*BB)->VP_Cartesian;
		Vector3 horiz(cur.X - my.X, cur.Y - my.Y, 0.0);
		if (horiz.length() < 1e-3)
		{
			Vector3 F = (*BB)->MyForwardVector;
			horiz = Vector3(F.X, F.Y, 0.0);
		}
		if (horiz.length() < 1e-3) horiz = Vector3(1, 0, 0);
		horiz.normalize();
		horiz = horiz * 1500.0;								// 수평은 작게(상승 우선)

		// 강한 상승 목표점 (원래 검증된 값: Up*8000)
		Vector3 climbVP = my + horiz + WorldUp * 8000.0;

		if (alt < FloorHard)
		{
			(*BB)->VP_Cartesian = climbVP;					// 무조건 강한 pull-up
		}
		else
		{
			double t = (FloorSoft - alt) / (FloorSoft - FloorHard);
			if (t < 0.0) t = 0.0; if (t > 1.0) t = 1.0;
			(*BB)->VP_Cartesian = cur * (1.0 - t) + climbVP * t;
		}

		return NodeStatus::SUCCESS;
	}
}
