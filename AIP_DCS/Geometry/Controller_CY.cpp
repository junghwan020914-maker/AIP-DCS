#include "Controller_CY.h"
#include <math.h>
#include <cstdio>

// #define CTRL_DBG_TRACE	// uncomment for one-off [CTRL_DBG] per-tick trace
float clamp(float input, float RangeDown, float RangeUp)
{
	if (input <= RangeDown)
	{
		return RangeDown;
	}
	else if (input >= RangeUp)
	{
		return RangeUp;
	}
	else
	{
		return input;
	}
}

StickController::StickController()
{
	SumCount = 0;
	for (int i = 0; i < 20; i++)
		MF[i] = 0;
	FilterIndex = 0;

	for (int i = 0; i < 60; i++)
		ErrorSum.push_back(0.0);
}

float StickController::GetLOSErrorSUM(float LOSError)
{
	// 08-07: 주최측이 "JSBSim controller는 열화된 모델로 제공되며 고쳐야 성능의 고점을
	// 볼 수 있다(Controller_CY.h/.cpp)"고 공식 답변 — 이 함수에 열화가 3중으로 겹쳐있다.
	//   1) sum이 int인데 ErrorSum은 float — 매 항목이 소수부 절삭됨.
	//   2) sum/60이 정수 나눗셈 — 반환값이 0,1,2… 계단값만 나온다. 정밀조준 구간
	//      (LOS<1도)에서는 모든 항목이 0으로 절삭돼 적분항이 항상 정확히 0이 된다.
	//      즉 "적분 보정이 가장 필요한 구간에서만 적분기가 꺼지는" 구조였다.
	//   3) 생성자가 이미 60개를 채워놨는데 SumCount<60 동안 push_back을 또 해서
	//      size가 119까지 늘어난다. 이후 인덱스 60~118은 첫 59틱 값이 영구 박제되고,
	//      합산은 119개를 도는데 나눗셈은 하드코딩 60 → 스케일도 약 2배로 부풀었다.
	// [실측 경위 — 이 수정을 되돌리지 말 것]
	// 이 수정 단독으로는 오히려 해로웠다(공식조건 40시드 vs v32 채점단위):
	//        기준선(무수정)                  순이득 +0.7036
	//        적분/링버퍼만 수정(AIP_intonly) 순이득 +0.6610  (-6.1%)
	//    원본의 정수절삭이 정밀조준 구간에서 적분항을 항상 0으로 만들어 사실상
	//    **안티와인드업**으로 기능하고 있었기 때문이다(적분 기여 최대 0.25가 근접 지속
	//    조준 중 상수 당김으로 붙어 오버슛 유발. P항은 LOS=1.5도에서야 0.25다).
	// 그러나 아래 D항(LOS<5도 게이트)을 넣으면 그 오버슛이 감쇠돼 손해가 사라진다:
	//        D항 + 이 수정 유지 (AIP_dgate) 순이득 +0.7474
	//        D항 + 이 수정 되돌림(AIP_best) 순이득 +0.7467   (차이 0.1% = 노이즈)
	// 성능이 동률이면 **다라운드 정확성**이 갈림표다. 원본은 리스폰 시 SumCount가 0으로
	// 리셋되면서 push_back을 다시 시작해 ErrorSum이 라운드마다 59개씩 무한 증가하고
	// (60 -> 119 -> 178 -> 237 ...), 합산은 전체를 도는데 나눗셈은 하드코딩 60이라
	// **적분항 스케일이 라운드마다 부풀어 오른다.** 로컬은 시드마다 환경을 새로 만들어
	// 드러나지 않지만 3라운드를 한 프로세스로 치르는 대회에서는 실제 결함이다.
	// 고정크기 60 링버퍼 + float 누산/나눗셈으로 교체한다.
	ErrorSum[SumCount % 60] = (LOSError <= 10.0f) ? LOSError : 0.0f;
	SumCount++;

	float sum = 0.0f;
	for (size_t i = 0; i < ErrorSum.size(); i++)
	{
		sum += ErrorSum[i];
	}

	return sum / static_cast<float>(ErrorSum.size());
}

StickValue StickController::GetStick(Vector3 MyLocation_FNED, Vector3 MyRotation_FNED, Vector3 VP)
{
	Vector3 Mylocation(MyLocation_FNED.X, MyLocation_FNED.Y, MyLocation_FNED.Z);
	Vector3 TargetLocation(VP.X, VP.Y, VP.Z);

	// 07-31 실측: 배치 테스트에서 네이티브 BT/컨트롤러 인스턴스가 에피소드마다 재생성되지
	// 않고 그대로 재사용됨(주최측 bt_action_provider.py가 reset()을 no-op 처리) -- 즉
	// MF[]/ErrorSum/SumCount 등이 이전 판(전혀 다른 기하)의 값을 다음 판 시작에 그대로
	// 들고 감. 실측으로 확인(같은 시드가 배치 안/단독 실행에서 다른 결과 -- 심지어
	// termination 종류까지 다름). 위치가 큰 폭으로 점프하면(=새 판 리스폰) 필터 상태를
	// 리셋해서 이전 판의 잔여 상태가 새 판으로 새는 것을 막는다.
	if (HasLastLocation && Mylocation.distance(LastKnownLocation) > 3000.0)
	{
		SumCount = 0;
		FilterIndex = 0;
		for (int i = 0; i < 20; i++) MF[i] = 0;
		for (size_t i = 0; i < ErrorSum.size(); i++) ErrorSum[i] = 0;
		HasLastLOS = false;		// D항도 이전 판의 LOS를 물고 가지 않도록 초기화
	}
	LastKnownLocation = Mylocation;
	HasLastLocation = true;

	//오일러 각을 입력. 이 부분은 언리얼4의 각도를 회사의 ECEF_LLA_Converter 쪽의 각도와 함수들을 이용하기 위해 이쪽 양식에 맞추는 과정
	EulerAngle EA;
	EA.Roll = MyRotation_FNED.X;
	EA.Pitch = MyRotation_FNED.Y;
	EA.Yaw = MyRotation_FNED.Z;

	//오일러각을 이용하면 축변화에 따른 오차가 생기기 때문에 쿼터니언으로 변환하여 사용
	Quaternion QU = EA.toQuaternion();

	//쿼터니언을 이용하여 전방벡터(ForwardVector)를 생성 
	Vector3 ForwardVector;
	ForwardVector.X = 1 - 2 * (QU.X * QU.X + QU.Y * QU.Y);
	ForwardVector.Y = 2 * (QU.X * QU.Z + QU.W * QU.Y);
	ForwardVector.Z = -2 * (QU.Y * QU.Z - QU.W * QU.X);

	//쿼터니언을 이용하여 수직벡터(UpVector)를 생성 
	Vector3 UpVector;
	UpVector.X = -2 * (QU.Y * QU.Z + QU.W * QU.X);
	UpVector.Y = -2 * (QU.X * QU.Y - QU.W * QU.Z);
	UpVector.Z = 1 - 2 * (QU.X * QU.X + QU.Z * QU.Z);

	//쿼터니언을 이용하여 오른쪽벡터(RightVector)를 생성 
	Vector3 RightVector;
	RightVector.X = 2 * (QU.X * QU.Z - QU.W * QU.Y);
	RightVector.Y = 1 - 2 * (QU.Y * QU.Y + QU.Z * QU.Z);
	RightVector.Z = -2 * (QU.X * QU.Y + QU.W * QU.Z);


	Vector3 ForwardVectorPoint = ForwardVector * 1000 + Mylocation;

	Vector3 ForwardVectorPoint2VP = TargetLocation - ForwardVectorPoint;

	Vector3 Proj_V = (ForwardVectorPoint2VP.dot(ForwardVector)) * ForwardVector;

	Vector3 Proj_P = TargetLocation - Proj_V;
	Vector3 Proj_TV = Proj_P - ForwardVectorPoint;

	// 롤커멘드 생성 부분

	float UpVector2Proj_TV_Angle = std::acos(UpVector.dot(Proj_TV / Proj_TV.length()));
	float UTAngle;
	float LOS = std::acos(ForwardVector.dot((TargetLocation - Mylocation)) / (TargetLocation - Mylocation).length()) * RADTODEG;

	if (_isnan(UpVector2Proj_TV_Angle) != 0)
	{
		UpVector2Proj_TV_Angle = 0;
	}

	float Proj_TV_Length = Proj_TV.length();

	if(Proj_TV_Length <= 0)
	{
		Proj_TV_Length = 0.0001;
	}

	if (RightVector.dot(Proj_TV / Proj_TV_Length) >= 0)
	{
		UTAngle = UpVector2Proj_TV_Angle;
	}
	else
	{
		UTAngle = UpVector2Proj_TV_Angle * (-1);
	}

	float RollCMD;

	if (std::abs(UTAngle * RADTODEG) > 90)
	{
		RollCMD = (std::sin(UTAngle) * 1);

		if (LOS > 3)
			RollCMD = clamp(RollCMD, -1, 1);
		else
			RollCMD = RollCMD * LOS * (-0.1);
	}
	else
	{
		RollCMD = (std::sin(UTAngle) * 1.0);

		RollCMD = clamp(RollCMD, -1, 1);

		RollCMD = RollCMD * std::abs(RollCMD);
	}


	if (_isnan(LOS) != 0)
	{
		LOS = 0;
	}

	// 08-07 열화 수정: 원본은 `if (RollCMD < 0.1) RollCMD *= 3;` 이었다.
	// 이 조건은 **음수 롤 전체**(-1 ~ 0)와 0~0.1 구간만 3배한다. 즉 |cmd|>=0.1 에서
	// 좌우 게인이 3배 차이나 한쪽으로만 조기 포화한다(음수는 |cmd|>0.33이면 이미 -1로
	// 클램프, 양수는 1.0이 돼야 포화). 결과적으로 한쪽 선회만 공격적이고 반대쪽은 굼떠서
	// 도그파이트 선회가 계속 한 방향으로 편향된다. 정밀조준 구간에서는 이 편향이 그대로
	// 조준점 주위 리밋사이클로 나타난다.
	// 부호에 무관하게 동일 게인을 적용해 좌우 대칭으로 만든다(약한 쪽을 강한 쪽에 맞춤).
	// 대안: `if (std::abs(RollCMD) < 0.1)` (소량명령만 부스트 = 강한 쪽을 약한 쪽에 맞춤).
	// 현재 동작이 90%대를 내고 있으므로 "약한 쪽 상향"을 먼저 시도한다.
	// ✅ 08-07 검증 완료: 공식조건 40시드 vs v32 채점단위에서 이 대칭화가 **처음으로
	// 정밀조준(<1도) 시간을 움직인 변경**이다. 1.27s -> 1.39s(+9.4%), <3도 5.58->9.05s,
	// 순이득 +0.7036 -> +0.7206 HP/판, 승패 36/3 -> 38/1.
	// 좌우 게인 3배 차이가 조준점 주위에 리밋사이클을 만들고 있었다는 가설과 일치.
	RollCMD = RollCMD * 3;

	RollCMD = RollCMD * clamp(LOS, 0, 1);
	//러더 커맨드 생성 부분
	float RudderCMD = 0;

	RudderCMD = -std::sin(UTAngle) * clamp(LOS, 0, 6) * 1;

	MF[FilterIndex % 20] = RudderCMD;
	FilterIndex++;

	// 08-07: 위 GetLOSErrorSUM과 같은 정수절삭 열화. MF[]에는 [-6,6] 범위의 float
	// 러더값이 들어가는데 MFsum이 int라 매 항목이 절삭되고, MFsum/20도 정수 나눗셈이라
	// |MFsum|<20이면 결과가 통째로 0이 된다. LOS가 작은 정밀조준 구간에서는 러더값이
	// 작아 20개 전부 0으로 절삭 → 필터 출력이 정확히 0 → RudderCMD/2.
	// 즉 이동평균 필터가 아니라 "러더 게인 50% 감쇠기"로만 동작했고, 큰 각도에서는
	// 정수 계단 때문에 오히려 불연속 점프를 만들었다. float 누산으로 교체.
	float MFsum = 0.0f;
	for (int i = 0; i < 20; i++)
		MFsum += MF[i];
	RudderCMD = (MFsum / 20.0f + RudderCMD) / 2.0f;

	//피치 커맨드 생성 부분
	float PitchCMD = 0;;

	// 08-07 D항 추가. 원본은 P항(LOS/6) + 계단형 I항뿐이고 미분항이 없어서, 조준점으로
	// 수렴하는 중에도 명령이 줄지 않아 반드시 오버슛하고 그 주위를 진동한다. 실측에서
	// 이 증상이 그대로 보인다 — <10도 13.2s인데 <1도는 1.27s(90% 소실)이고, 제어기
	// 정수절삭을 고쳐 <10도를 +21% 늘려도 <1도는 +0.8%로 요지부동이었다.
	// dLOS < 0 (좁혀지는 중)이면 명령을 줄여 감쇠, dLOS > 0 (벌어지는 중)이면 더 당긴다.
	//
	// ❌ 1차 시도(무조건 적용, Kd=0.01, 클램프 ±0.5) 실패 — 40시드 채점단위에서
	//    순이득 +0.7036 -> +0.6773(-3.7%), 특히 **상대 HP 0.1405 -> 0.1686(+20%)** 으로
	//    우리가 더 많이 맞았다. 원인: 격렬한 기동 중에는 dLOS가 수십 deg/s로 요동쳐
	//    ±0.5 클램프가 P항(최대 1.5)에 맞먹는 교란이 된다. 감쇠가 필요한 건 하드턴이
	//    아니라 **최종 정착 구간**뿐인데 전 구간에 걸어버린 것이 잘못이었다.
	// → 정밀조준 진입 구간(LOS < DGateDeg)에만 걸고, 기여 상한도 크게 낮춘다.
	//    이 구간에서 P항은 최대 DGateDeg/6 = 0.83이므로 ±0.15면 유의미하되 지배적이지 않다.
	const float Kd = 0.01f;
	const float DGateDeg = 5.0f;
	const float DClamp = 0.15f;
	float dLOS = 0.0f;
	if (HasLastLOS)
		dLOS = (LOS - LastLOS) * 60.0f;   // deg/s (틱 0.0166666s 고정)
	LastLOS = LOS;
	HasLastLOS = true;

	float dTerm = (LOS < DGateDeg) ? clamp(Kd * dLOS, -DClamp, DClamp) : 0.0f;

	float ERROR_Effect = clamp(LOS / 6 + clamp(GetLOSErrorSUM(LOS) / 7.5, 0, 0.25)
	                           + dTerm, 0, 1.5);
	//float ERROR_Effect = clamp(LOS / 6, 0, 1.5);


	float Roll_Effect = 1 - clamp(std::abs(UTAngle * RADTODEG) / 90, 0, 1);

	float Horizon_Effect;
	if (std::abs(UTAngle * RADTODEG) <= 90)
	{
		Horizon_Effect = 1;
	}
	else
		Horizon_Effect = 0.5;

	//std::cout << "ERROR_Effect : " << ERROR_Effect << " Roll_Effect : " << Roll_Effect << " Horizon_Effect : " << Horizon_Effect << std::endl;

	if (LOS < 90)
		PitchCMD = ERROR_Effect * Roll_Effect * Horizon_Effect * (-1);//+Roll_Effect2;
	else
		PitchCMD = -1;

#ifdef CTRL_DBG_TRACE
	std::fprintf(stdout,
		"[CTRL_DBG] LOS=%.2f UT=%.2f rollEff=%.3f errEff=%.3f horizEff=%.2f pitch=%.3f\n",
		LOS, UTAngle * RADTODEG, Roll_Effect, ERROR_Effect, Horizon_Effect, PitchCMD);
	std::fflush(stdout);
#endif

	StickValue Result;
	Result.RollCMD = clamp(RollCMD, -1, 1);
	Result.PitchCMD = clamp(PitchCMD, -1, 1);
	Result.RudderCMD = clamp(RudderCMD, -1, 1);
	//Result.RudderCMD = RudderCMD;
	return Result;
}
