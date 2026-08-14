#include "Jinking.h"
#include <cmath>

namespace Action
{
	PortsList Jinking::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
			InputPort<double>("Throttle", -1.0, "방어 중 스로틀. <0이면 미지정(폴백)"),
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

			// 🔴 08-14: 방어 중 스로틀이 **스테이션키핑 폴백**에 넘어가고 있었다.
			// 방어 노드는 스로틀을 안 건드리므로 `CPPBehaviorTree.cpp:421`의 폴백이 걸린다:
			//     thr = 0.5 + (적속도-내속도)/40 + (거리-500)/1500   clamp[0.15,1.0]
			// 이건 "표적 옆 500m를 유지하며 속도를 맞추는" 로직이라 **쫓기는 상황을
			// 상정한 게 아니다**(템플릿 원본 코드).
			// 실측(`_death_probe.py`, prev 20시드): 우리가 맞는 순간의 중앙값이
			//     내ATA 171.4도(등을 보임) / 거리 504m / 속도차 -12.5m/s / **스로틀 0.83**
			// 폴백식에 그 값을 넣으면 0.5+12.5/40+4/1500 = **0.815**로 정확히 일치한다.
			// 즉 등을 보인 채 상대보다 12.5m/s 느린 상태를 스스로 유지하고 있었다.
			// 교과서적으로도 방어 선회는 전추력이 정석이다 — 최대 G로 당기면서 추력을
			// 줄이면 에너지가 급격히 빠진다.
			// 포트 미지정(-1)이면 **기존 동작 그대로**라 같은 바이너리로 A/B가 된다.
			{
				const double thr = getInput<double>("Throttle").value();
				if (thr >= 0.0) (*BB)->Throttle = (float)thr;
			}

		return NodeStatus::SUCCESS;
	}
}
