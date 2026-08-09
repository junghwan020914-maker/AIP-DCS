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
			// ❌ 08-10 2차 기각: BeforeSeconds=30으로 "머지 전 에너지 확보"만 노렸으나
			//    승점 142.0(기준선 144.5). arcE는 30.0으로 복구됐지만 v29가 26.5->25.0,
			//    v32가 28.5->27.0으로 **둘 다 기준선 아래**가 됐다. 두 가지를 배웠다:
			//    (a) v29의 이득은 머지가 아니라 **지속적** 에너지 관리에서 나온다
			//        (무제한판 27.5 vs 30초판 25.0). 내 "머지 전" 해석이 틀렸다.
			//    (b) 더 약한 개입이 더 나빴다 — t=30의 **하드 전환**이 하강 중·저속·자세가
			//        어긋난 순간에 일반 BFM으로 튀는 불연속을 만든다.
			//    -> 시간으로 자르는 접근 자체를 버린다. 기본값 0(비활성) 유지.
			InputPort<double>("BeforeSeconds", 0.0, "engage only before this time (s); 0 = no limit"),
			// 상대가 나보다 이만큼(m) 넘게 **위에** 있으면 발동하지 않는다.
			// 1차 기각의 기전이 정확히 이것이다 — 우리가 내려가는 동안 상대(arcE)가
			// ClimbOut으로 올라가 수직으로 이별했고, 30판 전부 밴드 진입 0회였다.
			// BFM 원칙으로도 **올라가는 상대에게서 내려가면 안 된다**. 따라가야 한다.
			// 0 이하면 이 게이트를 쓰지 않는다.
			InputPort<double>("TargetAboveMax", 0.0, "skip if target is higher than me by more than this (m); <=0 disables")
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
		const double tgtAboveMax= getInput<double>("TargetAboveMax").value();

		Vector3 my = (*BB)->MyLocation_Cartesian;
		const double dist = (*BB)->Distance;
		const double spd  = (*BB)->MySpeed_MS;
		const double alt  = my.Z;

		// 자체 게이트 — 하나라도 어긋나면 아래 분기에 넘긴다.
		if (beforeSec > 0.0 && (*BB)->RunningTime > beforeSec)
			return NodeStatus::FAILURE;						// 초반 한정 모드
		if (dist <= distBeyond) return NodeStatus::FAILURE;	// 사거리 근처면 싸운다
		if (tgtAboveMax > 0.0 &&
		    ((*BB)->TargetLocaion_Cartesian.Z - alt) > tgtAboveMax)
			return NodeStatus::FAILURE;						// 상대가 위에 있으면 내려가지 않는다
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
