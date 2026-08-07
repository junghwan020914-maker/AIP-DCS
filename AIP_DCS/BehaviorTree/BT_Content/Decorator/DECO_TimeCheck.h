#pragma once
#include "../../behaviortree_cpp_v3/action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../../../Geometry/Vector3.h"
#include <iostream>
#include "../Functions.h"
#include "../BlackBoard/CPPBlackBoard.h"

using namespace BT;

/*
	DECO_TimeCheck : 라운드 경과시간 조건 검사

	08-07: WEZ Phase가 "공간적 폴백"이 아니라 **시간 게이트**라는 것이 주최측 공식
	답변으로 확정되면서(Phase2 t>=100s, Phase3 t>=150s) 경과시간 자체가 전술 입력이 됐다.

		Phase1  t>=0s    LOS<1도  152~914m   계수 1.0
		Phase2  t>=100s  LOS<2도  152~1067m  계수 0.3
		Phase3  t>=150s  LOS<3도  152~1219m  계수 0.1

	원뿔 입체각비가 1 : 4 : 9라 계수를 곱한 기대득점은 1 : 1.2 : 0.9 —
	즉 100초 이후는 "명중 판정이 훨씬 관대해지는 구간"이다. 게다가 데미지 산식상
	격추(HP 100 소진)는 사실상 불가능해 모든 경기가 200초 만료 후 HP 비교로 결정되므로,
	후반 100초의 관대한 원뿔에서 얼마나 긁어내느냐가 승패를 직접 가른다.

	이 노드는 상대 기체의 성질이 아니라 **규칙**에서 도출되므로 어떤 상대에게도 성립한다.

	Seconds  : 비교 기준 시각(초)
	UpDown   : "Greater" = 경과시간 >= Seconds 일 때 SUCCESS
	           "Less"    = 경과시간 <  Seconds 일 때 SUCCESS
*/
namespace Action
{
	class DECO_TimeCheck : public SyncActionNode
	{
	private:

	public:

		DECO_TimeCheck(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
		{
		}

		~DECO_TimeCheck()
		{

		}

		static PortsList providedPorts();

		NodeStatus tick() override;
	};
}
