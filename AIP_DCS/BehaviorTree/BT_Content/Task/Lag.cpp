#include "Lag.h"

namespace Action
{
	PortsList Lag::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
			// 적기 후미에서 겨냥할 거리(m). 0이면 거리의 절반을 자동 사용
			InputPort<double>("LagDistance", 0.0, "lag point distance behind target in meters (0 = auto)")
		};
	}

	NodeStatus Lag::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");
		double LagDistance = getInput<double>("LagDistance").value();

		Vector3 my  = (*BB)->MyLocation_Cartesian;
		Vector3 TargetLocation = (*BB)->TargetLocaion_Cartesian;	// 적기 현재 위치
		Vector3 TFV = (*BB)->TargetForwardVector;					// 적기 진행 방향
		TFV.normalize();
		Vector3 TRV = (*BB)->TargetRightVector;						// 적기 오른쪽(선회면 판단용)
		TRV.normalize();

		float Distance = (*BB)->Distance;							// 적기와의 거리 (m)

		// 컨트롤존: 적 6시(꼬리축) 뒤 고정 거리 = extended 6 o'clock (논문 ~3000ft)
		// 단, 현재 거리보다 크면 관통하므로 클램프
		double lagDist = (LagDistance > 0.0) ? LagDistance : 700.0;
		if (lagDist > Distance * 0.9) lagDist = Distance * 0.9;

		// 적 꼬리축 위 목표점
		Vector3 sixPoint = TargetLocation - TFV * lagDist;

		// 래그 추적: 적 선회 '안쪽'(내가 있는 쪽)으로 편향해 각을 딴다(pull to inside)
		Vector3 toMe = my - TargetLocation;
		double insideSign = (toMe.dot(TRV) >= 0.0) ? 1.0 : -1.0;
		Vector3 inside = TRV * (insideSign * lagDist * 0.35);

		Vector3 predicted = sixPoint + inside;

		// TailSlot(선회궤도 추종) 시도했으나 실측 회귀(WEZ 340->170, 패배 5->7)로 되돌림.
		// 기록: [[project-topgun-status]], 재시도 전 원인 규명 필요(블렌드 가중치 w 튜닝 등).

		(*BB)->VP_Cartesian = predicted;

		// 폐쇄율(dV) 관리 — 조준은 되는데 사거리를 관통해버리는 문제(사격전환 벽) 방지.
		BTFunc::ApplyClosureThrottle(*BB);

		return NodeStatus::SUCCESS;
	}
}
