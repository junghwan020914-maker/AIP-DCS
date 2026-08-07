#include "Lead.h"

namespace Action
{
	PortsList Lead::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
			// 예측량 보정 계수. XML에서 지정하지 않으면 기본값 1.0 사용
			InputPort<double>("LeadGain", 1.0, "lead amount gain (default 1.0)")
		};
	}

	NodeStatus Lead::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");
		double LeadGain = getInput<double>("LeadGain").value();

		Vector3 TargetLocation = (*BB)->TargetLocaion_Cartesian;	// 적기 현재 위치
		Vector3 TFV = (*BB)->TargetForwardVector;					// 적기 진행 방향
		TFV.normalize();											// 단위 벡터화(안전)

		float TargetSpeed = (*BB)->TargetSpeed_MS;					// 적기 속도 (m/s)
		float MySpeed = (*BB)->MySpeed_MS;							// 내 속도 (m/s)
		float Distance = (*BB)->Distance;							// 적기와의 거리 (m)

		// 0으로 나누기 방지. 정지에 가까울 땐 최소 접근 속도 가정
		double Closing = (MySpeed < 1.0f) ? 1.0 : (double)MySpeed;

		// 적기에 도달하기까지 걸리는 대략적인 시간
		double TimeToReach = Distance / Closing;

		// 그 시간 동안 적기가 진행 방향으로 이동할 거리만큼 앞을 겨냥
		Vector3 PredictedTarget = TargetLocation + TFV * (TargetSpeed * TimeToReach * LeadGain);

		(*BB)->VP_Cartesian = PredictedTarget;

		// 폐쇄율(dV) 관리 — 조준은 되는데 사거리를 관통해버리는 문제(사격전환 벽) 방지.
		BTFunc::ApplyClosureThrottle(*BB);

		return NodeStatus::SUCCESS;
	}
}
