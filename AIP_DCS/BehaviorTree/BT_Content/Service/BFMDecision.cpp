#include "BFMDecision.h"

namespace Action
{
	PortsList BFMDecision::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
			// 이 거리(m) 밖이면 DETECTING (기본 9260m = 5NM)
			InputPort<double>("DetectRange",   9260.0, "detecting range in meters"),
			// 기수 조준 판정 원뿔 반각(deg)
			InputPort<double>("Cone",            30.0, "aiming cone half-angle in deg"),
			// 3-9 라인(내가 적 후미쪽) 판정 AspectAngle 기준(deg). 이 값 이상이면 내가 적 뒤
			InputPort<double>("BehindAA",        90.0, "aspect angle(deg) above which I am behind target 3-9 line"),
			// 우세/열세를 가르는 비에너지 차이(m)
			InputPort<double>("EnergyMargin",  1000.0, "specific-energy advantage margin in meters")
		};
	}

	NodeStatus BFMDecision::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");

		double DetectRange  = getInput<double>("DetectRange").value();
		double Cone         = getInput<double>("Cone").value();
		double BehindAA     = getInput<double>("BehindAA").value();
		double EnergyMargin = getInput<double>("EnergyMargin").value();

		// 타겟이 없으면 판정 불가 -> NONE
		if ((*BB)->Enemy.size() == 0)
		{
			(*BB)->BFM = NONE;
			return NodeStatus::SUCCESS;
		}

		float Distance   = (*BB)->Distance;
		float LOS        = (*BB)->Los_Degree;			// 내 기수 -> 적 (작을수록 내가 조준)
		float LOS_Target = (*BB)->Los_Degree_Target;	// 적 기수 -> 나 (작을수록 적이 조준)

		// --- 비에너지(Specific Energy) 계산 : Es = 고도 + V^2 / 2g ---
		const double g = 9.80665;
		float MyAlt  = (*BB)->MyLocation_Cartesian.Z;
		float TgtAlt = (*BB)->TargetLocaion_Cartesian.Z;
		float MyV    = (*BB)->MySpeed_MS;
		float TgtV   = (*BB)->TargetSpeed_MS;
		float MyEs   = MyAlt  + (MyV  * MyV)  / (2.0 * g);
		float TgtEs  = TgtAlt + (TgtV * TgtV) / (2.0 * g);
		(*BB)->MySpecificEnergy     = MyEs;
		(*BB)->TargetSpecificEnergy = TgtEs;

		// 1) 교전거리 밖 -> 탐지/접근 단계
		if (Distance > DetectRange)
		{
			(*BB)->BFM = DETECTING;
			return NodeStatus::SUCCESS;
		}

		// 2) 위치 우열 판정
		bool iPoint      = (LOS <= Cone);			// 내가 적을 조준 중
		bool enemyPoint  = (LOS_Target <= Cone);	// 적이 나를 조준 중
		float AA         = (*BB)->MyAspectAngle_Degree;	// 적 기수 기준 내 위치각. ~180=내가 적 6시(후미)
		bool iAmBehind   = (AA >= BehindAA);		// 내가 적 3-9 라인 뒤(후미쪽) -> 유리

		// 3) 에너지 우열
		bool energySuperior = (MyEs >= TgtEs + EnergyMargin);
		bool energyInferior = (MyEs <= TgtEs - EnergyMargin);

		BFM_Mode Result;

		// --- Superior? 종합 판정 (위치 우선, 에너지 보조) ---
		if (iAmBehind || (iPoint && !enemyPoint))
		{
			// 내가 적 후미를 물었거나 조준 우위 -> 공격 (기수를 겨눔)
			Result = OBFM;
		}
		else if (enemyPoint && !iPoint)
		{
			// 적이 내 후미를 물음 -> 방어
			Result = DBFM;
		}
		else
		{
			// 위치 대등 -> 에너지로 우열 결정
			if (energySuperior)      Result = OBFM;
			else if (energyInferior) Result = DBFM;
			else                     Result = HABFM;
		}

		(*BB)->BFM = Result;
		return NodeStatus::SUCCESS;
	}
}
