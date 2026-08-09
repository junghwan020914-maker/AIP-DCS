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

		// 🔴 08-10 수정: 원본은 `insideSign = (toMe.dot(TRV) >= 0) ? 1 : -1` 인 **하드 부호
		// 반전**이었다. 히스테리시스도 연속성도 없어서, 내가 적의 꼬리축(중심선) 근처에
		// 있으면 toMe.dot(TRV) ~ 0 이라 **매 틱 부호가 뒤집힌다.** 조준점이 좌우로
		// lagDist*0.35*2 (= 700m 기준 490m) 만큼 60Hz로 튀고, 제어기는 그걸 최대 롤로
		// 쫓다가 리밋사이클에 빠진다.
		// 하필 그 특이점이 **정미익 추격 = 가장 흔한 공세 상황**이다.
		//
		// 실측(퇴화상대 `_degenerate_probe.py`, 직진수평 표적 duck 6시드):
		//   기준선 4/6 무득점. 트레이스에서 ATA는 0.1~5.8도로 기수가 표적에 물려 있는데
		//   롤 각이 -151.9 / +176.7 / -95.5도 — 뒤집힌 채 계속 구른다.
		//   전추력 수평비행 최고속도는 505m/s인데(`_thrust_probe.py`) 추격 중 실제 속도는
		//   421m/s — **에어프레임 성능의 84m/s(17%)를 진동 항력으로 버리고 있었다.**
		//   그래서 0.70 스로틀(407m/s) 직진 표적조차 못 따라잡았다.
		//   거리 의존성도 관측과 맞는다: 횡편향각 = atan(0.35*lagDist / (D - lagDist)) 이라
		//   2600m에서 7.3도, 2000m에서 10.7도로 **접근할수록 커져** 3000~3500m에서 평형에 걸린다.
		//   (`DECO_DistanceCheck Less 3000` 게이트 안쪽에서만 발동하는 것과 일치)
		// -> 부호를 **연속 램프**로 바꾼다. 중심선에서 확실히 벗어나 있으면 원래대로 최대
		//    편향, 중심선 부근(횡성분 20% 이내)에서는 0을 통과하며 매끄럽게 전환한다.
		//    편향의 의도(선회 안쪽에서 각 따기)는 그대로 두고 채터링만 없앤다.
		double toMeLen = toMe.length();
		double lat = (toMeLen > 1e-6) ? (toMe.dot(TRV) / toMeLen) : 0.0;	// -1 ~ +1
		const double LatDead = 0.20;										// 약 11.5도
		double insideScale = lat / LatDead;
		if (insideScale >  1.0) insideScale =  1.0;
		if (insideScale < -1.0) insideScale = -1.0;
		Vector3 inside = TRV * (insideScale * lagDist * 0.35);

		Vector3 predicted = sixPoint + inside;

		// TailSlot(선회궤도 추종) 시도했으나 실측 회귀(WEZ 340->170, 패배 5->7)로 되돌림.
		// 기록: [[project-topgun-status]], 재시도 전 원인 규명 필요(블렌드 가중치 w 튜닝 등).

		(*BB)->VP_Cartesian = predicted;

		// 폐쇄율(dV) 관리 — 조준은 되는데 사거리를 관통해버리는 문제(사격전환 벽) 방지.
		BTFunc::ApplyClosureThrottle(*BB);

		return NodeStatus::SUCCESS;
	}
}
