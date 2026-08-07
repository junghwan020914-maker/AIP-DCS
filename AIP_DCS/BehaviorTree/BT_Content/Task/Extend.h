#pragma once
/*
	Extend (거리 이탈) 기동 Task 노드

	08-07 주최측 Q&A로 데미지 산식이 확정되면서 도출된 규칙유래 방어기동.

		d_wez(r) = 0                    (r > 3000ft = 914.4m)
		d_wez(r) = (3000 - r) / 2500    (500ft <= r <= 3000ft)
		d_wez(r) = 0                    (r < 500ft = 152.4m)

	여기서 핵심은 **r이 양측 공용 변수**라는 점이다. 거리가 가까워지면 내가 주는
	데미지와 내가 받는 데미지가 "같은 배율"로 함께 커진다. 따라서 접근은 각 우위일
	때만 이득이고, 각 열세일 때 근접 유지는 순손해다.

	그리고 밴드 밖에서는 데미지가 각도와 무관하게 **정확히 0**이다. 즉 각 열세일 때
	r > 914.4m로 벌리는 것은 "회피 확률을 높이는" 것이 아니라 피격을 수학적으로
	완전히 차단하는 행위다. 기존 방어노드(BreakTurn/Jinking)는 전부 각도 회피만 하고
	거리 개념이 없었기 때문에, 상대가 조준을 유지하는 동안 밴드 안에 계속 머물렀다.
	(실측: 조준정체 케이스에서 134초 내내 밴드 안에 있으면서 Los_Degree 최소 71도 —
	 각도로는 안전했지만 상대 각도가 좋아지는 순간 그대로 실점하는 구조였다.)

	VP = 적기 반대 방향으로 멀리 찍는다. 순수 분리(pure separation)라 방향이 예측
	가능하다는 단점이 있으나, 밴드 밖으로 나가는 순간 예측가능성은 무의미해진다
	(데미지가 0이므로 조준당해도 실점이 없다).

	ClimbBias : 이탈 방향에 섞을 상승 성분(m). 0이면 순수 수평 이탈.
	            약간의 상승은 에너지를 위치에너지로 저장해 이후 재교전에 유리하지만,
	            과하면 속도가 죽어 오히려 거리가 안 벌어진다.
*/
#include "../../behaviortree_cpp_v3\action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../../../Geometry/Vector3.h"
#include "../Functions.h"
#include "../BlackBoard/CPPBlackBoard.h"

using namespace BT;

namespace Action
{
	class Extend : public SyncActionNode
	{
	private:


	public:

		Extend(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
		{
		}

		~Extend()
		{
		}

		static PortsList providedPorts();

		NodeStatus tick() override;
	};
}
