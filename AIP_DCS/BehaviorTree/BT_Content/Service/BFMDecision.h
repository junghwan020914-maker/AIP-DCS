#pragma once
/*
	BFMDecision - 현재 교전 국면(BFM 모드)을 판정하여 BB->BFM 에 기록하는 서비스 노드

	판정에 사용하는 블랙보드 지표(모두 앞선 서비스 노드가 매 틱 계산):
		Distance          : 적기와의 거리(m)
		Los_Degree        : 내 기수 -> 적  각도(deg)  (작을수록 내가 적을 조준)
		Los_Degree_Target : 적 기수 -> 나  각도(deg)  (작을수록 적이 나를 조준)
		MyAngleOff_Degree : 두 기수의 교차각(deg)     (0=동일방향, 180=정면반대)
		MySpeed_MS/TargetSpeed_MS : 시저스(저속) 판정용

	분류 규칙 (우선순위)
		1) Distance > DetectRange                      -> DETECTING
		2) 내가조준 & 적은못조준                        -> OBFM (공격)
		3) 적이조준 & 나는못조준                        -> DBFM (방어)
		4) 서로조준                                     -> HABFM (정면)
		5) 뉴트럴(둘다못조준) + 교차각 큼               -> HABFM
		6) 뉴트럴 + 근접 + 저속 + 교차각 중간이상        -> SCISSORS
		7) 그 외                                         -> HABFM
		(타겟이 없으면 NONE)

	임계값은 모두 InputPort 로 노출(미지정 시 기본값 사용) -> 재빌드 없이 XML에서 튜닝 가능
*/
#include "../../behaviortree_cpp_v3\action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../../../Geometry/Vector3.h"
#include "../Functions.h"
#include "../BlackBoard/CPPBlackBoard.h"

using namespace BT;

namespace Action
{
	class BFMDecision : public SyncActionNode
	{
	private:


	public:

		BFMDecision(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
		{
		}

		~BFMDecision()
		{
		}

		static PortsList providedPorts();

		NodeStatus tick() override;
	};
}
