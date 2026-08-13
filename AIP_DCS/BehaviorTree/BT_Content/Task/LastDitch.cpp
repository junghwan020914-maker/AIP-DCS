#include "LastDitch.h"

namespace Action
{
	PortsList LastDitch::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
			InputPort<double>("TargetAtaBelow", 30.0, "fire only if target ATA < this (deg)"),
			InputPort<double>("MyAtaAbove", 120.0, "fire only if my ATA > this (deg)"),
			InputPort<double>("DistBelow", 600.0, "fire only within this range (m)"),
			InputPort<double>("ArmSeconds", 2.0, "conditions must hold this long before firing"),
			InputPort<double>("HoldSeconds", 3.0, "how long to hold the brake (measured safe max 4)"),
			InputPort<double>("CooldownSeconds", 25.0, "lockout after release (measured recovery time)"),
			InputPort<double>("FloorAlt", 1500.0, "do not fire below this altitude (m)"),
			InputPort<double>("Throttle", 0.0, "throttle while braking (0 = idle)")
		};
	}

	NodeStatus LastDitch::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");

		const double ataBelow = getInput<double>("TargetAtaBelow").value();
		const double myAtaAbove = getInput<double>("MyAtaAbove").value();
		const double distBelow = getInput<double>("DistBelow").value();
		const double armSec = getInput<double>("ArmSeconds").value();
		const double holdSec = getInput<double>("HoldSeconds").value();
		const double coolSec = getInput<double>("CooldownSeconds").value();
		const double floorAlt = getInput<double>("FloorAlt").value();
		const double thr = getInput<double>("Throttle").value();

		Vector3 my = (*BB)->MyLocation_Cartesian;
		Vector3 tgt = (*BB)->TargetLocaion_Cartesian;
		const double now = (*BB)->RunningTime;

		// --- 라운드 경계 리셋 ---
		// `BreakTurn`과 같은 이유다. 주최측 reset()이 no-op이라 네이티브 노드 인스턴스가
		// 배치/다라운드에 걸쳐 재사용된다. 직전 라운드의 쿨다운 타임스탬프를 물려받으면
		// 새 라운드 내내 발동이 잠긴다. 위치 점프와 시간 역행을 둘 다 본다.
		const bool timeWentBack = (now < LastTickTime - 1e-6);
		const bool episodeReset = !HasLastLocation
			|| my.distance(LastKnownLocation) > 3000.0
			|| timeWentBack;
		LastKnownLocation = my;
		HasLastLocation = true;
		LastTickTime = now;
		if (episodeReset)
		{
			ArmedSince = -1.0;
			FireStart = -1.0;
			LastFireEnd = -1e9;
		}

		// --- 발동 중이면 지속시간만 본다 ---
		// 일단 걸리면 조건이 잠깐 흔들려도 유지한다. 감속은 연속이어야 의미가 있고,
		// 매틱 켜졌다 꺼지면 채터링이 된다(`Lag` 조준점 채터링에서 배운 것).
		if (FireStart >= 0.0)
		{
			if ((now - FireStart) < holdSec)
			{
				const double myAlt = my.Z;
				if (myAlt >= floorAlt)          // 고도 안전선은 발동 중에도 계속 본다
				{
					// 조준점: 상대 쪽으로 계속 당긴다. **등을 돌리지 않는다.**
					// ryujan이 `Task_Evade`(횡 브레이크) 제거로 이득을 본 이유가 그거다.
					// 상대 방향 성분 + 강한 상승 성분 -> 최대 받음각 당김 + 줌.
					Vector3 toT = tgt - my;
					double len = toT.length();
					if (len < 1e-6) len = 1e-6;
					toT = toT / len;

					Vector3 U = (*BB)->MyUpVector; U.normalize();

					// 상대 쪽 방위는 유지하되 크게 위로 — 당김을 최대화한다.
					Vector3 dir = toT * 1000.0 + U * 3000.0;
					dir.normalize();
					(*BB)->VP_Cartesian = my + dir * 3000.0;
					(*BB)->Throttle = (float)thr;
					return NodeStatus::SUCCESS;
				}
			}
			// 지속시간 만료 또는 고도 하한 침범 -> 해제하고 쿨다운 시작
			FireStart = -1.0;
			LastFireEnd = now;
			ArmedSince = -1.0;
			return NodeStatus::FAILURE;
		}

		// --- 쿨다운 ---
		// 실측 회복 시간이 25초다. 그 전에 다시 걸면 두 번째는 회복 불가 영역으로 간다
		// (8초 유지 = 114 m/s에서 회복 실패). 여기가 이 노드의 가장 중요한 안전장치다.
		if ((now - LastFireEnd) < coolSec) return NodeStatus::FAILURE;

		// --- 발동 조건 ---
		const double myAta = (*BB)->Los_Degree;           // 내 기수 -> 상대
		const double tgtAta = (*BB)->Los_Degree_Target;   // 상대 기수 -> 나
		const double dist = (*BB)->Distance;

		const bool cond =
			(tgtAta < ataBelow)        // 상대가 나를 겨누고 있다
			&& (myAta > myAtaAbove)    // 나는 상대를 못 겨눈다 = 확실한 수세
			&& (dist < distBelow)      // 가깝다 = 오버슛이 실제로 일어날 수 있다
			&& (my.Z >= floorAlt);

		if (!cond)
		{
			ArmedSince = -1.0;
			return NodeStatus::FAILURE;
		}

		// 순간 스침으로 발동하지 않도록 연속 유지를 요구한다.
		if (ArmedSince < 0.0)
		{
			ArmedSince = now;
			return NodeStatus::FAILURE;
		}
		if ((now - ArmedSince) < armSec) return NodeStatus::FAILURE;

		FireStart = now;
		return NodeStatus::SUCCESS;
	}
}
