#include "DECO_WEZCheck.h"

namespace Action
{
	PortsList DECO_WEZCheck::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
			InputPort<std::string>("Side", "Target",
				"Target = enemy in my WEZ, Me = I am in enemy WEZ, MeOnly = 나만 피격중(일방 열세)")
		};
	}

	NodeStatus DECO_WEZCheck::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");
		std::string side = getInput<std::string>("Side").value();

		bool ok;
		if (side == "Me")
		{
			ok = (*BB)->IamInTargetWEZ;
		}
		else if (side == "MeOnly")
		{
			// 08-08 신규. 데미지는 거리에만 의존하므로(양측 공용) **동시 피격은 순이득이
			// 정확히 0**이다 — 실측으로 확인(동시구간 +0.0000). 즉 "서로 쏘는 중"은 손해가
			// 아니라 무의미하고, 진짜 손실은 **나만 맞는 구간**에서만 난다
			// (arcA 상대 실측: 상대만 득점 38.8%, v32 상대는 2.7%).
			// 그래서 방어 전환은 "적이 나를 조준함"이 아니라 "적이 나를 조준하는데
			// 나는 못 하고 있음"에서만 걸어야 공격을 훼손하지 않는다.
			//
			// ❌ 08-08: 이 조건으로 최상위 방어전환(→BreakTurn)을 걸어봤으나 **실패, 원복**.
			//    arcA  -0.0074(17승13패) → -0.0720(14승16패)
			//    v32   +0.7474(38승2패)  → +0.7087(39승1패)
			//    원인: 트리거가 너무 늦다. IamInTargetWEZ가 참이 되는 시점은 적이 이미
			//    1도 안에 들어와 **맞고 있는 중**이라 브레이크를 걸어도 피격은 이미 났고,
			//    대신 우리가 만들던 공격 기회만 버린다(밴드 체류 52.2%→46.4%, 내 득점
			//    0.7530→0.7347). 반응형이 아니라 예측형 트리거라야 의미가 있다.
			//    포트는 남겨둠(다른 용도/조건으로 재시도 가능).
			ok = (*BB)->IamInTargetWEZ && !(*BB)->TargetInMyWEZ;
		}
		else
		{
			ok = (*BB)->TargetInMyWEZ;
		}
		return ok ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
	}
}
