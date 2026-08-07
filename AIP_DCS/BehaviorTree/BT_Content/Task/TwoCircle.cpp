#include "TwoCircle.h"

namespace Action
{
	PortsList TwoCircle::providedPorts()
	{
		return { InputPort<CPPBlackBoard*>("BB") };
	}

	NodeStatus TwoCircle::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");

		Vector3 my  = (*BB)->MyLocation_Cartesian;
		Vector3 tgt = (*BB)->TargetLocaion_Cartesian;
		Vector3 F  = (*BB)->MyForwardVector; F.normalize();
		Vector3 R  = (*BB)->MyRightVector;   R.normalize();
		Vector3 U  = (*BB)->MyUpVector;      U.normalize();
		Vector3 TF = (*BB)->TargetForwardVector; TF.normalize();

		// 적기 기수가 향하는 쪽(같은 회전방향)으로 선회 -> two-circle rate fight
		// 적 전방벡터를 내 우측축에 투영한 부호로 회전 방향 결정
		double proj = TF.dot(R);
		double side;
		if (proj > 0.05)       side = 1.0;
		else if (proj < -0.05) side = -1.0;
		else
		{
			// 적기가 나와 거의 정면(투영≈0)이면 적 위치 반대쪽으로 -> nose-to-tail 유도
			Vector3 toT = tgt - my;
			side = (toT.dot(R) >= 0.0) ? -1.0 : 1.0;
		}

		// 지속 최대선회(rate). 전방 성분을 조금 더 두고 옆으로 강하게, 약간 아래로 에너지 유지
		(*BB)->VP_Cartesian = my + F * 1500.0 + R * (side * 4500.0) - U * 500.0;

		return NodeStatus::SUCCESS;
	}
}
