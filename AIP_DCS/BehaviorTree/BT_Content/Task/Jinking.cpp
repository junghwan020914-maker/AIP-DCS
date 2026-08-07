#include "Jinking.h"
#include <cmath>

namespace Action
{
	PortsList Jinking::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
			InputPort<double>("Freq", 3.0, "jink frequency (rad/s scale)"),
			InputPort<std::string>("AimBlend")
		};
	}

	NodeStatus Jinking::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");
		double w = getInput<double>("Freq").value();
		Optional<std::string> AimBlendPort = getInput<std::string>("AimBlend");
		double AimBlend = AimBlendPort ? std::stod(AimBlendPort.value()) : 0.0;

		Vector3 my = (*BB)->MyLocation_Cartesian;
		Vector3 tgt = (*BB)->TargetLocaion_Cartesian;
		Vector3 F = (*BB)->MyForwardVector; F.normalize();
		Vector3 R = (*BB)->MyRightVector;   R.normalize();
		Vector3 U = (*BB)->MyUpVector;      U.normalize();

		double t = (*BB)->RunningTime;

		// 좌우/상하 이위상 진동 -> 예측 불가한 저크
		double lat = std::sin(t * w);
		double vert = std::sin(t * w * 1.37 + 1.1);

		Vector3 jinkOffset = F * 3000.0 + R * (4000.0 * lat) + U * (2000.0 * vert);

		if (AimBlend > 0.0)
		{
			// 08-05: BreakTurn AimBlend와 같은 아이디어. toT는 월드프레임 벡터라
			// 내 자세에 안 묶여 있어 매틱 그대로 섞어도 BreakTurn류 자기추적 문제는
			// 없음 — 다만 여기는 근접(<600m) 불규칙 회피라 너무 크게 섞으면 진동의
			// 예측불가성 자체가 훼손될 위험이 있어 작은 값으로 시작.
			Vector3 toT = tgt - my;
			toT.normalize();
			Vector3 jinkDir = jinkOffset; jinkDir.normalize();
			Vector3 blended = jinkDir * (1.0 - AimBlend) + toT * AimBlend;
			blended.normalize();
			jinkOffset = blended * jinkOffset.length();
		}

		(*BB)->VP_Cartesian = my + jinkOffset;

		return NodeStatus::SUCCESS;
	}
}
