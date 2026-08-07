// Fill out your copyright notice in the Description page of Project Settings.


#include "CPPBehaviorTree.h"

#include <exception>
#include <cstdio>

// #define MINE_DBG_TRACE	// uncomment for one-off [MINE_DBG] per-tick trace


Vector3 UCPPBehaviorTree::LLAtoCartesian(Vector3 LLA, Vector3 BaseLLA)
{
	double eccentricitysquare, N, M;
	eccentricitysquare = 1.0 - pow(6356752.3142, 2) / pow(6378137.0, 2);
	N = 6378137.0 / sqrt(1.0 - eccentricitysquare * pow(sin(BaseLLA.X * PI / 180.0), 2)); // prime vertical radius of curvature
	M = 6378137.0 * (1.0 - eccentricitysquare) / pow(1 - eccentricitysquare * pow(sin(BaseLLA.X * PI / 180.0), 2), 3 / 2);

	double dlat, dlon;
	dlat = LLA.X - BaseLLA.X;
	dlon = LLA.Y - BaseLLA.Y;

	double dN, dE, dD;
	dN = (M + BaseLLA.Z) * dlat * PI / 180.0;
	dE = (N + BaseLLA.Z) * cos(BaseLLA.X * PI / 180.0) * dlon * PI / 180.0;
	dD = (LLA.Z - BaseLLA.Z);
	Vector3 res(dN, dE, dD);
	return res;
}

// Sets default values for this component's properties
UCPPBehaviorTree::UCPPBehaviorTree()
{
	ID = -1;
	ForceID = -1;

	f2m = 3.28084;
	EQ_R = 6.378137E+6;
	P_R = 6.3567523142E+6;
	fr = 298.257223563;
	Req = 6.378137E+6;
	d2r = 3.1415926535897931 / 180.0;
	m2f = 3.28084;


	elev0 = 0.2;
	aile0 = 0.0;
	eccen = 1.0 - P_R * P_R / (EQ_R * EQ_R);
	bInitialized = false;

	BB = new CPPBlackBoard();
}


UCPPBehaviorTree::~UCPPBehaviorTree()
{
	delete BB;
}


void UCPPBehaviorTree::init()
{
	bInitialized = false;

	try
	{
		/*
		노드 입력 : 구현해둔 노드들을 Factory 객체에 입력해주는 과정
		
		새로 생성한 노드를 여기에 입력해주세요!!!!!!
		*/
		Factory.registerNodeType<Action::SelectTarget>("SelectTarget");
		Factory.registerNodeType<Action::DistanceUpdate>("DistanceUpdate");
		Factory.registerNodeType<Action::CheckSight>("CheckSight");
		Factory.registerNodeType<Action::AngleOffUpdate>("AngleOffUpdate");
		Factory.registerNodeType<Action::DirectionVectorUpdate>("DirectionVectorUpdate");
		Factory.registerNodeType<Action::AspectAngleUpdate>("AspectAngleUpdate");
		Factory.registerNodeType<Action::BFMDecision>("BFMDecision");
		Factory.registerNodeType<Action::WEZUpdate>("WEZUpdate");
		Factory.registerNodeType<Action::DECO_BFMCheck>("DECO_BFMCheck");
		Factory.registerNodeType<Action::DECO_DistanceCheck>("DECO_DistanceCheck");
		Factory.registerNodeType<Action::DECO_LOSCheck>("DECO_LOSCheck");
		Factory.registerNodeType<Action::DECO_TargetLOSCheck>("DECO_TargetLOSCheck");
		Factory.registerNodeType<Action::DECO_StuckCheck>("DECO_StuckCheck");
		Factory.registerNodeType<Action::DECO_AngleOffCheck>("DECO_AngleOffCheck");
		Factory.registerNodeType<Action::DECO_EnergyCheck>("DECO_EnergyCheck");
		Factory.registerNodeType<Action::DECO_WEZCheck>("DECO_WEZCheck");
		Factory.registerNodeType<Action::DECO_AltitudeCheck>("DECO_AltitudeCheck");
		Factory.registerNodeType<Action::Task_Empty>("Task_Empty");

		// maneuver Task nodes (VP generation)
		Factory.registerNodeType<Action::Pure>("Pure");
		Factory.registerNodeType<Action::Lead>("Lead");
		Factory.registerNodeType<Action::MergeReversal>("MergeReversal");
		Factory.registerNodeType<Action::Lag>("Lag");
		Factory.registerNodeType<Action::BreakTurn>("BreakTurn");
		Factory.registerNodeType<Action::Jinking>("Jinking");
		Factory.registerNodeType<Action::OneCircle>("OneCircle");
		Factory.registerNodeType<Action::TwoCircle>("TwoCircle");
		Factory.registerNodeType<Action::Scissors>("Scissors");
		Factory.registerNodeType<Action::PreventLandCrash>("PreventLandCrash");
		Factory.registerNodeType<Action::Task_ClimbOut>("Task_ClimbOut");



		//파일로 트리 구조 정의
		//자신의 팀 이름으로	xml 파일 만들어서 입력해주세요!!!!!! (Rule_forTraining.xml은 예시입니다)
		tree = Factory.createTreeFromFile("./Rule_mine.xml");


		//블랙보드 연결 : 원래는 블랙보드 내에 있는 모든 변수를 하나하나 이런식으로 입력해줘야하는 미친 비효율을 보이는 방식이지만 커스텀 블랙보드를 만들어 해당 블랙보드를 입력시킴
		tree.rootBlackboard()->set<CPPBlackBoard*>("BB", BB);
		
		bInitialized = true;
		std::cout << "Behavior Tree Initialized Successfully" << std::endl;
	}
	catch (const std::exception& e)
	{

		std::cout << "Behavior Tree Initialization Failed: " << e.what() << std::endl;

		std::cout << "It appears that the process failed while parsing the XML." << std::endl;
		std::cout << " -Please check whether the XML file is located in the correct path." << std::endl;
		std::cout << " -Please check whether the XML file is calling any node with an invalid or incorrect name." << std::endl;
		std::cout << " -Please check whether the node was added to the Factory when building the DLL." << std::endl;
		throw;
	}
	
}

bool UCPPBehaviorTree::IsInitialized() const
{
	return bInitialized;
}

StickValue UCPPBehaviorTree::Step(PlaneInfo MyInfo, int NumofOtherPlane, PlaneInfo* OthersInfo, Vector3& VP, float& Throttle)
{
	PlaneInfo Myinfo;
	Myinfo.Location = MyInfo.Location;
	Myinfo.Rotation = EulerAngle(MyInfo.Rotation.Yaw, MyInfo.Rotation.Pitch, MyInfo.Rotation.Roll);
	Myinfo.AngleAcceleration = MyInfo.AngleAcceleration;
	Myinfo.Speed = MyInfo.Speed;
	Myinfo.Team = MyInfo.Team;
	Myinfo.Resv0 = MyInfo.Resv0;		//ID
	Myinfo.Resv1 = MyInfo.Resv1;		//HP
	Myinfo.Resv2 = MyInfo.Resv2;		//OperationMode

	//다른 비행기들 위치 좌표계 변환
	PlaneInfo others[4];
	for (int i = 0; i < NumofOtherPlane; i++)
	{
		Vector3 Enemylocation_Cartesian = OthersInfo[i].Location;
		others[i].Location = Enemylocation_Cartesian;
		others[i].Rotation = EulerAngle(OthersInfo[i].Rotation.Yaw, OthersInfo[i].Rotation.Pitch, OthersInfo[i].Rotation.Roll);
		others[i].Speed = OthersInfo[i].Speed;
		others[i].Team = OthersInfo[i].Team;
		others[i].Resv0 = OthersInfo[i].Resv0;
		others[i].Resv1 = OthersInfo[i].Resv1;
		others[i].Resv2 = OthersInfo[i].Resv2;
	}

	//블랙보드의 아군기, 적군기 List 초기화
	BB->Friendly.clear();
	BB->Enemy.clear();

	//블랙보드에 내 정보(위치, 자세, 속력, 팀) 업데이트
	BB->MyLocation_Cartesian = MyInfo.Location;
	BB->MyRotation_EDegree = EulerAngle(Myinfo.Rotation.Yaw, Myinfo.Rotation.Pitch, Myinfo.Rotation.Roll);
	BB->MyAngleAcceleration = Myinfo.AngleAcceleration;
	BB->MySpeed_MS = Myinfo.Speed;
	BB->Team = (TeamColor)Myinfo.Team;

	//아군기 리스트에 내 정보 추가. Friendly의 index 0번은 무조건 나 자신
	BB->Friendly.push_back(Myinfo);

	//생존중인 비행기들의 적아 구분
	for (int i = 0; i < NumofOtherPlane; i++)
	{
		if (others[i].Resv1 > 0)
		{
			if (others[i].Team == Myinfo.Team)
			{
				BB->Friendly.push_back(others[i]);
			}
			else
			{
				BB->Enemy.push_back(others[i]);
			}
		}
		else
		{

		}
	}


	bool AimmingMode;

	StickValue R;

	//블랙보드에 입력된 정보를 바탕으로 비헤비어트리 Run
	RunCPPBT(VP, Throttle, AimmingMode);

	// [boresight 클램프]
	// Controller_CY는 VP가 기수에서 90도 이상 벗어나면 PitchCMD가 연속식이 아니라
	// UpVector.Z 부호만 보는 이진(+-1) 커맨드로 바뀌어, 롤이 90/270도를 지날 때마다
	// 부호가 뒤집히며 조준이 고착(limit cycle)된다. VP를 기수 기준 일정 각도 이내로
	// 제한해 제어기가 항상 연속식 안전구간(LOS<90)에서만 동작하게 우회한다.
	// 매 틱 기수가 갱신되며 재계산되는 폐루프라, 기수가 돌아온 만큼 VP도 원래 목표로 수렴한다.
	double DebugOffBoresightDeg = 0.0;
	{
		const double MAX_OFFBORESIGHT_DEG = 75.0;
		const double RAD2DEG = 57.2957795;

		Vector3 Fwd = BB->MyForwardVector;			// 트리 tick에서 이미 갱신됨
		Vector3 RawDir = VP - BB->MyLocation_Cartesian;
		double RawDist = RawDir.length();

		if (RawDist > 1e-3 && Fwd.length() > 1e-6)
		{
			Fwd.normalize();
			Vector3 DirUnit = RawDir;
			DirUnit.normalize();

			double OffBoresightDeg = Fwd.angleBetween(DirUnit) * RAD2DEG;
			DebugOffBoresightDeg = OffBoresightDeg;
			if (OffBoresightDeg > MAX_OFFBORESIGHT_DEG)
			{
				double Factor = MAX_OFFBORESIGHT_DEG / OffBoresightDeg;
				Vector3 ClampedDir;
				ClampedDir.sLerp(Fwd, DirUnit, Factor);		// 기수->목표 방향으로 75도만 회전
				ClampedDir.normalize();
				VP = BB->MyLocation_Cartesian + ClampedDir * RawDist;
			}
		}
	}

	R = Controller.GetStick(
		BB->MyLocation_Cartesian,
		Vector3(BB->MyRotation_EDegree.Roll * DEG2RAD,
			BB->MyRotation_EDegree.Pitch * DEG2RAD,
			BB->MyRotation_EDegree.Yaw * DEG2RAD),
		VP);

#ifdef MINE_DBG_TRACE
	std::fprintf(stdout,
		"[MINE_DBG] team=%d bfm=%d dist=%.1f losMe=%.2f losTgt=%.2f aa=%.2f offb=%.2f "
		"attRoll=%.2f attPitch=%.2f attYaw=%.2f roll=%.3f pitch=%.3f rud=%.3f thr=%.2f\n",
		(int)BB->Team, (int)BB->BFM, BB->Distance, BB->Los_Degree, BB->Los_Degree_Target,
		BB->MyAspectAngle_Degree, DebugOffBoresightDeg,
		BB->MyRotation_EDegree.Roll, BB->MyRotation_EDegree.Pitch, BB->MyRotation_EDegree.Yaw,
		R.RollCMD, R.PitchCMD, R.RudderCMD, Throttle);
	std::fflush(stdout);
#endif

	return R;

}

Vector3 UCPPBehaviorTree::GetVP()
{
	Vector3 Vp = (*BB).VP_Cartesian;
	return Vp;
}



 void UCPPBehaviorTree::RunCPPBT(Vector3& VP, float& Throttle, bool& AimmingMode)
{
	
	BB->RunningTime += BB->DeltaSecond;	//시뮬레이선 타임에 따른 델타 타임 설정
	
	try
	{
		BB->Throttle = -1.0f;	// sentinel: Task가 스로틀을 직접 정하면 그 값을 존중

		tree.tickRoot(); //트리 작동
		VP = BB->VP_Cartesian;	// VP 값

		// 스로틀 제어: Task가 지정했으면(>=0) 그대로, 아니면 기본 스테이션키핑 적용
		if (BB->Throttle >= 0.0f)
		{
			Throttle = BB->Throttle;
		}
		else
		{
			float d = BB->Distance;
			float thr = 1.0f;
			if (d < 1500.0f)
			{
				// 스테이션키핑: 적 속도에 맞추고(속도매칭) 목표거리(~500m)로 거리 조절
				float dSpeed = BB->TargetSpeed_MS - BB->MySpeed_MS;	// 적이 빠르면 +(가속 필요)
				float dRange = d - 500.0f;							// 멀면 +(접근), 가까우면 -(감속)
				thr = 0.5f + dSpeed / 40.0f + dRange / 1500.0f;
				if (thr < 0.15f) thr = 0.15f;
				if (thr > 1.0f)  thr = 1.0f;
			}
			Throttle = thr;
			BB->Throttle = thr;
		}
	}
	catch (const std::exception& e)
	{
		//원인을 알 수 없는 예외가 발생할 경우 VP를 (0,0,0)으로 설정하고 Throttle을 1로 설정하여 일단 최대한 안전하게 행동하도록 설정
		VP = Vector3(0,0,0);	// VP 값
		Throttle = 1.0f;	//

		std::cout << "ERROR!!!!!!!!! Behavior Tree Execution Failed: " << e.what() << std::endl;
		std::cout << "Temp Result VP : (0,0,0), Throttle : 1" << std::endl;
		
		throw;
	}

	

	
}

 void UCPPBehaviorTree::SetDeltaTime(double DT)
 {
	 BB->DeltaSecond = DT;
 }

