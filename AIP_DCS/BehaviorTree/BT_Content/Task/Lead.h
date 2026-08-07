#pragma once
/*
	Lead Pursuit (전방 추적) 기동 Task 노드

	적기의 "미래 예측 위치"를 조준한다. 적기의 진행 방향(ForwardVector) 앞쪽을 겨냥하여
	거리를 좁히거나 총/미사일 조준(가늠좌)을 만들 때 사용.

	VP = 적기현재위치 + 적기진행방향 * (적기속도 * 도달예상시간 * LeadGain)

	- 도달예상시간(t) = 적기와의 거리 / 내 속도 (단순 비례 예측)
	- LeadGain : 예측량 보정 계수(포트로 조절, 기본 1.0). 1보다 키우면 더 앞을 겨냥
	- 장점 : 적기를 따라잡고 무기 조준선(lead)을 형성
	- 단점 : 너무 앞을 겨냥하면 오버슛 위험
*/
#include "../../behaviortree_cpp_v3\action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../../../Geometry/Vector3.h"
#include "../Functions.h"
#include "../BlackBoard/CPPBlackBoard.h"

using namespace BT;

namespace Action
{
	class Lead : public SyncActionNode
	{
	private:


	public:

		Lead(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
		{
		}

		~Lead()
		{
		}

		static PortsList providedPorts();

		NodeStatus tick() override;
	};
}
