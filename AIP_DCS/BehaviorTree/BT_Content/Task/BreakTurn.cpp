#include "BreakTurn.h"

namespace Action
{
	PortsList BreakTurn::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
			InputPort<std::string>("AimBlend"),
			InputPort<std::string>("RelatchSeconds"),
			InputPort<std::string>("EscalationStep"),
			InputPort<std::string>("MaxAimBlend"),
			InputPort<std::string>("StuckImproveDeg")
		};
	}

	NodeStatus BreakTurn::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");
		Optional<std::string> AimBlendPort = getInput<std::string>("AimBlend");
		double AimBlend = AimBlendPort ? std::stod(AimBlendPort.value()) : 0.0;
		Optional<std::string> RelatchPort = getInput<std::string>("RelatchSeconds");
		double RelatchSeconds = RelatchPort ? std::stod(RelatchPort.value()) : 3.0;
		Optional<std::string> EscalationPort = getInput<std::string>("EscalationStep");
		double EscalationStep = EscalationPort ? std::stod(EscalationPort.value()) : 0.0;
		Optional<std::string> MaxBlendPort = getInput<std::string>("MaxAimBlend");
		double MaxAimBlend = MaxBlendPort ? std::stod(MaxBlendPort.value()) : AimBlend;
		Optional<std::string> StuckDegPort = getInput<std::string>("StuckImproveDeg");
		double StuckImproveDeg = StuckDegPort ? std::stod(StuckDegPort.value()) : 5.0;

		Vector3 my  = (*BB)->MyLocation_Cartesian;
		Vector3 tgt = (*BB)->TargetLocaion_Cartesian;

		double now = (*BB)->RunningTime;

		// 07-31 실측: 배치 테스트에서 네이티브 BT 인스턴스가 에피소드마다 재생성되지 않고
		// 재사용됨(주최측 reset()이 no-op) -- RunningTime도 에피소드 경계에서 리셋 안 되고
		// 배치 전체에 걸쳐 계속 누적되므로, 매틱 연속 호출되는 한 "갭" 감지(now-LastTickTime>0.5)가
		// 에피소드가 바뀌어도 절대 발동하지 않아 이전 판(전혀 다른 기하)의 LatchedDir이 새 판
		// 시작에도 그대로 들고 갈 수 있음이 확인됨(같은 시드가 배치안/단독실행에서 결과가
		// 달랐음). 위치가 큰 폭으로 점프하면(=새 판 리스폰) 강제로 재래치.
		bool episodeReset = !HasLastLocation || my.distance(LastKnownLocation) > 3000.0;
		LastKnownLocation = my;
		HasLastLocation = true;
		if (episodeReset)
		{
			StuckStreak = 0;
			LastRelatchLosMe = -1.0;
		}

		// 이 노드가 최근에 계속 틱되고 있었는지(연속 DBFM 유지 중인지) 확인. 갭이 크면
		// 새 조우로 보고 다시 래치한다.
		// 07-30 실험: 원래는 연속 DBFM 동안 한번 래치하면 끝까지 고정이라(갭이 안 생기니
		// justEntered가 다시 True될 일이 없음), 방어기동이 처음부터 끝까지 완전히 예측
		// 가능한 단일 곡선이 되어 상대가 조준을 계속 물고 있을 수 있었음(실측: 패배 시드들의
		// 상대 ATA가 10~20초 내내 0~10도로 유지됨). 3초마다 주기적으로 재래치해서 방향을
		// 바꿔주는 실험.
		bool justEntered = !HasLatch || episodeReset || (now - LastTickTime > 0.5) || (now >= NextRelatchTime);
		LastTickTime = now;

		if (justEntered)
		{
			// 08-06: 이번 래치 구간이 끝나면서 실제로 조준각(Los_Degree)이 좋아졌는지 확인.
			// StuckImproveDeg 이상 개선 안 됐으면 정체로 보고 스트릭 누적, 개선됐으면 리셋.
			double CurrentLosMe = (*BB)->Los_Degree;
			if (EscalationStep > 0.0 && LastRelatchLosMe >= 0.0)
			{
				if (CurrentLosMe > LastRelatchLosMe - StuckImproveDeg)
					StuckStreak++;
				else
					StuckStreak = 0;
			}
			LastRelatchLosMe = CurrentLosMe;

			double EffectiveAimBlend = AimBlend;
			if (EscalationStep > 0.0)
			{
				EffectiveAimBlend = AimBlend + StuckStreak * EscalationStep;
				if (EffectiveAimBlend > MaxAimBlend) EffectiveAimBlend = MaxAimBlend;
			}

			Vector3 F = (*BB)->MyForwardVector; F.normalize();
			Vector3 R = (*BB)->MyRightVector;   R.normalize();
			Vector3 U = (*BB)->MyUpVector;      U.normalize();

			// 적기가 내 좌/우 어느 쪽인가 -> 그 방향으로 하드턴
			Vector3 toT = tgt - my;
			double side = (toT.dot(R) >= 0.0) ? 1.0 : -1.0;

			// 전방보다 옆(적 방향)을 크게, 약간 아래로 -> 최대선회 + 코너속도 유지.
			// 이 방향은 진입 시점 자세로 한 번만 계산해 월드프레임에 고정(래치)한다.
			// 매틱 현재 자세로 재계산하면 기체가 도는 만큼 목표도 같이 돌아버려서
			// off-boresight 각이 절대 줄지 않는 자기추적 문제가 생긴다.
			Vector3 dir = F * 1000.0 + R * (side * 4000.0) - U * 1500.0;
			dir.normalize();

			if (EffectiveAimBlend > 0.0)
			{
				// 래치 시점 기준 목표 방향을 소량 섞는다 — 그 후 다음 재래치까지는
				// 여전히 고정(래치)이라 자기추적 문제는 재발하지 않음.
				Vector3 toTUnit = toT;
				toTUnit.normalize();
				dir = dir * (1.0 - EffectiveAimBlend) + toTUnit * EffectiveAimBlend;
				dir.normalize();
			}

			LatchedDir = dir;
			HasLatch = true;
			NextRelatchTime = now + RelatchSeconds;
		}

		(*BB)->VP_Cartesian = my + LatchedDir * 4500.0;

		return NodeStatus::SUCCESS;
	}
}
