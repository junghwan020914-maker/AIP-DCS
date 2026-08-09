#include "EnergyTurn.h"

namespace Action
{
	PortsList EnergyTurn::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
			// 이 속도(m/s) 미만일 때만 발동. 코너속도 근처를 목표로 한다.
			InputPort<double>("SpeedBelow", 235.0, "engage only when my speed is below this (m/s)"),
			// 이 고도(m) 위에서만 발동. 아래로 내려갈 여유가 있어야 한다.
			InputPort<double>("AltAbove", 3000.0, "engage only above this altitude (m)"),
			// 이 거리(m) 밖에서만 발동. 사거리 근처면 내려가지 말고 싸운다.
			InputPort<double>("DistBeyond", 1000.0, "engage only beyond this range (m)"),
			// 하강 기울기: 조준점을 거리 * 이 값만큼 내 고도 아래에 찍는다.
			InputPort<double>("DiveSlope", 0.5, "aim point drop = Distance * DiveSlope"),
			// 이 고도(m) 아래로는 조준점을 내리지 않는다.
			InputPort<double>("FloorAlt", 2000.0, "never aim below this altitude (m)"),
			// 이 시각(s) 이전에만 발동. 0이면 제한 없음.
			// 목적이 **머지 전 에너지 확보**라면 초반에만 걸어야 한다 — 교전이 붙은 뒤에
			// 내려가면 접촉을 잃는다(특히 상승하는 에너지 파이터 상대).
			InputPort<double>("BeforeSeconds", 0.0, "engage only before this time (s); 0 = no limit")
		};
	}

	NodeStatus EnergyTurn::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");

		const double speedBelow = getInput<double>("SpeedBelow").value();
		const double altAbove   = getInput<double>("AltAbove").value();
		const double distBeyond = getInput<double>("DistBeyond").value();
		const double diveSlope  = getInput<double>("DiveSlope").value();
		const double floorAlt   = getInput<double>("FloorAlt").value();
		const double beforeSec  = getInput<double>("BeforeSeconds").value();

		Vector3 my = (*BB)->MyLocation_Cartesian;
		const double dist = (*BB)->Distance;
		const double spd  = (*BB)->MySpeed_MS;
		const double alt  = my.Z;

		// 자체 게이트 — 하나라도 어긋나면 아래 분기에 넘긴다.
		if (beforeSec > 0.0 && (*BB)->RunningTime > beforeSec)
			return NodeStatus::FAILURE;						// 초반 한정 모드
		if (dist <= distBeyond) return NodeStatus::FAILURE;	// 사거리 근처면 싸운다
		if (spd  >= speedBelow) return NodeStatus::FAILURE;	// 이미 충분히 빠르다
		if (alt  <= altAbove)   return NodeStatus::FAILURE;	// 내려갈 여유가 없다

		// 표적 쪽으로 선회하되 조준점을 내 고도 아래로 내린다(슬라이스).
		// 방위는 표적 그대로이므로 각을 버리지 않고, 수직 성분만 에너지로 바꾼다.
		Vector3 VP = (*BB)->TargetLocaion_Cartesian;
		double dropTo = alt - dist * diveSlope;
		if (dropTo < floorAlt) dropTo = floorAlt;
		if (VP.Z > dropTo) VP.Z = dropTo;	// 표적이 더 아래면 표적을 따른다

		(*BB)->VP_Cartesian = VP;
		(*BB)->Throttle = 1.0f;

		return NodeStatus::SUCCESS;
	}
}
