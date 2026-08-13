#include "SelectTarget.h"

namespace Action
{
	PortsList SelectTarget::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB")
		};
	}



	NodeStatus SelectTarget::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");

		//�л����� 1��1�� ������ �׳� ������ Ÿ�� ����
		if((*BB)->Enemy.size() > 0)
		{
			(*BB)->ACM = EF;
			
			(*BB)->TargetLocaion_Cartesian = (*BB)->Enemy.at(0).Location;
			(*BB)->TargetRotation_EDegree = (*BB)->Enemy.at(0).Rotation;
			(*BB)->TargetSpeed_MS = (*BB)->Enemy.at(0).Speed;

		}
		else
		{ 
#ifdef SELTGT_DBG_TRACE
			// 08-10: 무가드였다. 적 리스트가 비면 **매 틱 60Hz로 stdout에 쓴다** —
			// 틱 예산이 16.67ms인데 동기 stdout은 무시할 수 없는 비용이고,
			// 우리가 ryujan 팀에 경고한 바로 그 형태다. 가드로 묶는다.
			std::cout << "Target is not Valid!!!!!!" << std::endl;
#endif
		}
				
		return NodeStatus::SUCCESS;
	}

}