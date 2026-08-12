#pragma once
/*
	Sprint (조준 데드밴드 가속) 기동 Task 노드

	사거리 밖에서 이미 조준이 맞아 있으면 G를 당길 이유가 없다 — 그건 에너지 낭비다.
	VP를 **내 기수 정면**에 찍어 제어기가 보는 LOS를 0으로 만들고, 전추력으로 가속한다.

	배경(08-10 계측):
	  제어기 피치 명령은 ERROR_Effect = LOS/6 + clamp(LOSErrorSUM/7.5, 0, 0.25) + D 인데
	  가운데 항이 적분기가 아니라 **최근 60샘플 평균**이라 정상 추격에서 0.25로 영구 포화한다.
	  즉 LOS 1도에서도 0.42를 당긴다 — **명령이 0으로 내려갈 방법이 없어** 조준점을 반드시
	  지나치고, 그 상시 선회가 에너지를 갉아먹는다.
	  실측: 전추력 수평비행 505m/s가 가능한데(`_thrust_probe.py`) 추격 실속도는 421m/s.
	        0.70스로틀 직진 표적(407m/s)조차 못 따라잡아 duck 6시드 중 4판 무득점.

	이 노드는 제어기를 고치는 대신 **트리에서 데드밴드를 만들어** 우회한다.
	제어기 인터페이스가 VP뿐이라 "당기지 마라"를 표현하는 유일한 방법이 VP를 기수에
	얹는 것이다.

	적용 조건(XML에서 게이트): 사거리 밖 + LOS 작음. 사거리 안에서는 조준이 곧 득점이므로
	절대 걸면 안 된다.

	❌ 08-10 **기각.** 퇴화 상대(duck 직진표적)에서는 완벽했는데 반격하는 상대에게
	   대가가 더 컸다.

	   duck (게이트 1300m / LOS<6도, AimBlend 스캔):
	       0.00 -> 무득점 2/6 (조준을 아예 놓아 득점하던 판을 죽임)
	       0.15 -> 무득점 **0/6**  (기준선 4/6, 승점 4.0 -> 6.0. seed2는 최소거리 865m로
	                                처음 밴드 안에 진입)
	       0.30 -> 무득점 1/6 (당김이 되살아나 가속 손실)

	   그러나 코어 전수평가는 **144.0**(기준선 144.5)이고 **오늘 처음으로 진 판이 나왔다**:
	       arcA 30.0 -> 29.0 (29승 1패, 마진 -0.0926)
	       prev 29.5 -> 29.0 (29승 1패, 마진 -0.2924)
	       피격 0.0000 -> 0.0335 / 0.0250
	   장거리에서 안 당기고 직진하면 **상대가 전환해 올 때 기동하고 있지 않아 맞는다.**

	   ❌ 후속 시도도 실패: `DECO_TargetLOSCheck Greater 45`(상대가 나를 겨누면 해제)를
	   얹었더니 arcA가 오히려 **28.0(2패)**로 더 나빠졌다.
	   **겨누기 시작한 시점에는 이미 늦다** — 코스팅으로 잃은 에너지는 그 순간 회복되지
	   않는다. 이 계열은 접는다.

	   ⚠️ 교훈: **퇴화 상대 시험은 "얻는 것"만 보여주고 "치르는 대가"는 못 보여준다.**
	   반격하지 않는 상대에게서 얻은 개선은 반드시 코어 전수평가로 대가를 확인할 것.
*/
#include "../../behaviortree_cpp_v3\action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../../../Geometry/Vector3.h"
#include "../Functions.h"
#include "../BlackBoard/CPPBlackBoard.h"

using namespace BT;

namespace Action
{
	class Sprint : public SyncActionNode
	{
	private:


	public:

		Sprint(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
		{
		}

		~Sprint()
		{
		}

		static PortsList providedPorts();

		NodeStatus tick() override;
	};
}
