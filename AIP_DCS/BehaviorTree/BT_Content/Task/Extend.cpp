#include "Extend.h"

namespace Action
{
	PortsList Extend::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
			InputPort<std::string>("ClimbBias")
		};
	}

	NodeStatus Extend::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");

		double ClimbBias = 0.0;
		Optional<std::string> CB = getInput<std::string>("ClimbBias");
		if (CB) ClimbBias = std::stod(CB.value());

		Vector3 my = (*BB)->MyLocation_Cartesian;
		Vector3 tgt = (*BB)->TargetLocaion_Cartesian;

		// 적 반대 방향 단위벡터
		Vector3 away = my - tgt;
		double len = away.length();
		if (len < 1e-6)
		{
			// 완전 동일좌표(정상적으로는 불가) — 안전하게 현재 기수방향 유지
			away = (*BB)->MyForwardVector;
		}
		else
		{
			away = away / len;
		}

		// 이탈점을 충분히 멀리 찍어 제어기가 최대 선회로 등을 돌리게 한다.
		// (VP는 "통과할 점"이 아니라 "바라볼 점"이므로 거리 자체는 방향만 결정)
		Vector3 vp = my + away * 20000.0;

		if (ClimbBias != 0.0)
		{
			// PreventLandCrash와 동일 관례: Z가 위쪽 양수인 평면좌표계
			vp = vp + Vector3(0.0, 0.0, 1.0) * ClimbBias;
		}

		(*BB)->VP_Cartesian = vp;

		// 이탈은 속도가 생명 — 폐쇄율 관리(ApplyClosureThrottle)를 쓰지 않고 최대추력.
		(*BB)->Throttle = 1.0f;

		return NodeStatus::SUCCESS;
	}
}
