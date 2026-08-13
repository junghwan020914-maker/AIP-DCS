// Fill out your copyright notice in the Description page of Project Settings.


#include "CPPBehaviorTree.h"

#include <exception>
#include <cstdio>
#include <cmath>
// #define RESET_DBG_TRACE

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
		Factory.registerNodeType<Action::DECO_TimeCheck>("DECO_TimeCheck");
		Factory.registerNodeType<Action::Task_Empty>("Task_Empty");

		// maneuver Task nodes (VP generation)
		Factory.registerNodeType<Action::Pure>("Pure");
		Factory.registerNodeType<Action::Extend>("Extend");
		Factory.registerNodeType<Action::Sprint>("Sprint");
		Factory.registerNodeType<Action::EnergyTurn>("EnergyTurn");
		Factory.registerNodeType<Action::FlatPursuit>("FlatPursuit");
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
		// 08-08 실험: 이 클램프가 "제어기 불연속을 피하려는 우회책"이라 보고 피치를
		// 연속화한 뒤 완화해봤으나 **명확한 회귀**였다(공식조건 40시드 vs v32, 순이득/승패):
		//        75도(현행)  +0.7474  38승 2패
		//        90도        +0.6423  31승 8패
		//        120도       +0.7010  32승 7패
		// 계단을 없앴는데도 나빠졌으므로 원인은 불연속이 아니라 제어기 구조다
		// (Roll_Effect = 1 - |UT|/90 이 큰 각도에서 0으로 죽음). 이 클램프는 우회책이
		// 아니라 **제어기를 자기가 잘 동작하는 영역 안에 붙잡아두는 장치**다. 유지할 것.
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

#ifdef VPJUMP_DBG_TRACE
	// 08-08: 트리 복잡도 가설 계측. 한 틱 사이 VP 방향이 몇 도 튀는지 잰다.
	// 같은 DLL로 Rule_mine.xml / Rule_arcA.xml 을 각각 돌리면 제어기·환경이 완전히
	// 동일하고 **트리만 다른** 비교가 된다. 밴드(152~914m) 안에서만 찍는다 — 득점이
	// 나는 구간의 추종 품질만 관심사이기 때문.
	{
		Vector3 vpDir = VP - BB->MyLocation_Cartesian;
		double vlen = vpDir.length();
		if (vlen > 1e-6)
		{
			vpDir = vpDir / vlen;
			double jump = -1.0;
			if (HasLastVPDir)
			{
				double c = vpDir.dot(LastVPDir);
				if (c > 1.0) c = 1.0;
				if (c < -1.0) c = -1.0;
				jump = std::acos(c) * 57.2957795;
			}
			LastVPDir = vpDir;
			HasLastVPDir = true;
			if (jump >= 0.0 && BB->Distance > 152.0f && BB->Distance < 914.0f)
			{
				std::fprintf(stdout, "[VPJUMP] jump=%.3f losMe=%.3f dist=%.1f bfm=%d\n",
					jump, BB->Los_Degree, BB->Distance, (int)BB->BFM);
				std::fflush(stdout);
			}
		}
	}
#endif

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

#ifdef BFM_HIST_TRACE
	// 08-09 진단: BFM 국면 + 최상위 반격분기 발동 여부를 1초에 한 번 샘플링.
	// 배관 테스트(Seconds=0/Distance=3000, 거의 항상 발동해야 함)가 현행과 자릿수까지
	// 동일하게 나와, 그 분기가 속한 **OBFM 블록에 도달 자체를 못 한다**는 의심이 생겼다.
	// 위협시점 트레이스에서도 HABFM 60.4% / DBFM 39.6% / OBFM 0% 였다.
	{
		static int tick = 0;
		if ((tick++ % 60) == 0)
		{
			std::fprintf(stdout, "[BFMHIST] bfm=%d losTgt=%.1f dist=%.0f losMe=%.1f\n",
				(int)BB->BFM, BB->Los_Degree_Target, BB->Distance, BB->Los_Degree);
			std::fflush(stdout);
		}
	}
#endif

#ifdef THREAT_DBG_TRACE
	// 08-08 진단용: **상대가 나에게 조준을 잡고 있는 순간에만** 찍는다.
	// arcA(앵글 파이터) 매치업에서 상대만 득점하는 틱의 46.4%가 내 ATA>90도였는데
	// (v32 상대는 4.8%), 이게 (a) 선회 선택 실패로 뒤를 잡힌 것인지 (b) 내 방어기동
	// (BreakTurn은 설계상 적 반대쪽으로 기수를 돌린다)이 안 먹혀 그대로 물린 것인지
	// 외부 관측만으로는 못 가른다. 그 순간의 BFM 국면을 직접 본다.
	// 매틱이 아니라 위협 구간(losTgt<3도 + 밴드 내)에서만 찍으므로 출력량은 판당 수백 줄.
	// bfm: 0=OBFM 1=HABFM 2=DBFM 3=DETECTING 4=SCISSORS 5=NONE
	if (BB->Los_Degree_Target < 3.0f && BB->Distance > 152.0f && BB->Distance < 914.0f)
	{
		std::fprintf(stdout, "[THREAT] team=%d bfm=%d dist=%.1f losTgt=%.2f losMe=%.2f aa=%.2f\n",
			(int)BB->Team, (int)BB->BFM, BB->Distance,
			BB->Los_Degree_Target, BB->Los_Degree, BB->MyAspectAngle_Degree);
		std::fflush(stdout);
	}
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
	
	// 08-07: RunningTime을 라운드 경과시간으로 신뢰할 수 있게 만든다.
	// 주최측 reset()이 no-op이라 BT 인스턴스가 에피소드/라운드 경계에서 재생성되지 않고,
	// RunningTime도 리셋되지 않아 배치·다라운드 전체에 걸쳐 계속 누적된다. 지금까지는
	// BreakTurn/Controller_CY가 각자 "위치 점프" 휴리스틱으로 개별 대응해왔는데, WEZ Phase가
	// 시간 게이트(t>=100s 2도, t>=150s 3도)라는 것이 확인된 이상 경과시간 자체가 전술 입력이
	// 되므로 발생원에서 한 번에 리셋한다.
	// 감지는 두 신호를 OR로 쓴다.
	//  (1) 위치 점프: 한 틱 실이동은 350m/s * 0.0167s ≒ 5.8m라 300m는 물리적으로 불가능.
	//      100m로 잡으면 네트워크 지연으로 프레임이 밀렸을 때(0.33s 갭이면 100m) 라운드
	//      중간에 오탐이 나서 Phase3 구간인데 Phase1로 착각할 수 있다. 300m면 1초(60프레임)
	//      연속 유실이 있어야 오탐인데, 그 정도면 이미 "응답불능" 판정 영역이다.
	//  (2) 상한 클램프: 라운드는 최대 200초다. 혹시 (1)이 리스폰을 놓쳐도(새 스폰이 직전
	//      종료지점 근처인 희귀 케이스) 210초를 넘으면 무조건 새 라운드로 보고 되돌린다.
	// (MyLocation_Cartesian은 이 함수 호출 전에 이미 갱신됨)
	//  (3) 08-10 추가 — **표적과의 거리 점프.** (1)(2)만으로는 구멍이 남는다:
	//      라운드가 조기 종료되고(격추 시 100초대에 끝나는 판이 실제로 있다) 새 스폰이
	//      직전 종료지점 300m 안이면 (1)도 (2)도 발동하지 않는다. 그러면 다음 라운드가
	//      RunningTime=112 같은 값으로 시작해 **개시 즉시 Phase2가 활성**된다(t>=100s).
	//      한 틱 최대 폐쇄는 약 500m/s * 0.0167s = 8.3m이므로 300m 점프는 물리적으로
	//      불가능하다. 내 위치가 안 변해도 **상대가 리스폰하면 이 신호가 잡는다.**
	//      라운드 중에는 원리상 발동할 수 없어 단일 라운드 동작은 불변이다.
	{
		const Vector3 myNow = BB->MyLocation_Cartesian;
		const float distNow = BB->Distance;
		const bool respawned = HasLastBTLocation && myNow.distance(LastBTLocation) > 300.0;
		const bool distJumped = HasLastBTDistance
			&& std::fabs(distNow - LastBTDistance) > 300.0f;
		if (respawned || distJumped || BB->RunningTime > 210.0)
		{
#ifdef RESET_DBG_TRACE
			std::fprintf(stdout, "[RESET] t=%.2f pos=%d dist=%d clamp=%d  d=%.1f->%.1f\n",
				BB->RunningTime, (int)respawned, (int)distJumped,
				(int)(BB->RunningTime > 210.0), LastBTDistance, distNow);
			std::fflush(stdout);
#endif
			BB->RunningTime = 0.0;
		}
		LastBTLocation = myNow;
		HasLastBTLocation = true;
		// 유효한 거리를 본 뒤에만 무장한다. 첫 틱에는 BB->Distance가 아직 0이라
		// 그대로 저장하면 다음 틱에 0->914 점프로 오발동한다(계측으로 확인).
		LastBTDistance = distNow;
		if (distNow > 1.0f) HasLastBTDistance = true;
	}

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
		// 🔴 08-10 수정. 원본은 세 가지가 잘못돼 있었다:
		//  (1) `VP = (0,0,0)` 은 "안전"이 아니라 **좌표 원점을 조준하라**는 뜻이다.
		//      우리 위치에 따라 지면으로 내리꽂는 명령이 되고, 규정 제6조1항상
		//      **추락은 즉시 패배**다. 일시적 오류가 패배로 확정된다.
		//  (2) 마지막의 `throw;` 때문에 위에서 설정한 "안전값"이 **실제로 쓰이지 않는다.**
		//      예외가 Step() 밖으로 전파된다(로컬 하네스에서는 `OSError: BT Step failed`로
		//      나타난다 — 실제로 겪었다). 서버에서는 기권/중단으로 이어질 수 있다.
		//  (3) 매 틱 예외가 나면 stdout 2줄씩 60Hz로 쏟는다.
		// -> **안전한 퇴화**로 바꾼다: 수평 기수 방향으로 완만히 상승하며 직진.
		//    지면에서 멀어지는 방향이라 최악의 경우에도 추락을 부르지 않는다.
		//    로그는 최초 1회만 남긴다(진단은 되되 틱 예산을 갉지 않게).
		{
			Vector3 my = BB->MyLocation_Cartesian;
			Vector3 f = BB->MyForwardVector;
			Vector3 fh(f.X, f.Y, 0.0);
			if (fh.length() < 1e-3) fh = Vector3(1.0, 0.0, 0.0);
			fh.normalize();
			VP = my + fh * 2000.0 + Vector3(0.0, 0.0, 200.0);	// 직진 + 완만한 상승
			Throttle = 0.9f;
		}

		static bool s_reported = false;
		if (!s_reported)
		{
			s_reported = true;
			std::cout << "BT Execution Failed (once): " << e.what() << std::endl;
		}
		// 재던지지 않는다 — 안전값으로 계속 비행하는 편이 항상 낫다.
	}

	// 🔴 08-10 최종 위생: Throttle/VP가 NaN이나 범위 밖이면 서버에 그대로 나간다.
	// 트리의 어느 노드든 계산이 틀어지면 여기로 새므로 마지막에 한 번 걸러낸다.
	if (!(Throttle >= 0.0f) || !(Throttle <= 1.0f))	// NaN이면 두 비교 모두 거짓
	{
		Throttle = (Throttle > 1.0f) ? 1.0f : ((Throttle < 0.0f) ? 0.0f : 0.9f);
	}
	if (std::isnan(VP.X) || std::isnan(VP.Y) || std::isnan(VP.Z))
	{
		Vector3 my = BB->MyLocation_Cartesian;
		Vector3 f = BB->MyForwardVector;
		Vector3 fh(f.X, f.Y, 0.0);
		if (fh.length() < 1e-3) fh = Vector3(1.0, 0.0, 0.0);
		fh.normalize();
		VP = my + fh * 2000.0 + Vector3(0.0, 0.0, 200.0);	// 안전 기본값(직진+완만상승)
	}


	

	
}

 void UCPPBehaviorTree::SetDeltaTime(double DT)
 {
	 BB->DeltaSecond = DT;
 }

