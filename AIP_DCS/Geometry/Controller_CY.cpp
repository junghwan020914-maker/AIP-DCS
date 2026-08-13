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

	// 🔴 08-10: acos 인자 클램프. 쿼터니언 유래 벡터라 크기가 정확히 1이 아니어서
	// 거의 평행할 때 비율이 1을 넘어 NaN이 된다. **조준이 완벽할 때** 발동한다.
	// 게다가 아래 `_isnan(LOS)` 검사는 RollCMD 계산 **뒤에** 있어 이미 오염된 뒤다.
	double projLen = Proj_TV.length();
	if (projLen < 1e-9) projLen = 1e-9;
	double dotUP = UpVector.dot(Proj_TV / projLen);
	if (dotUP >  1.0) dotUP =  1.0;
	if (dotUP < -1.0) dotUP = -1.0;
	float UpVector2Proj_TV_Angle = (float)std::acos(dotUP);
	float UTAngle;
	double tgtLen = (TargetLocation - Mylocation).length();
	if (tgtLen < 1e-9) tgtLen = 1e-9;
	double dotFL = ForwardVector.dot(TargetLocation - Mylocation) / tgtLen;
	if (dotFL >  1.0) dotFL =  1.0;
	if (dotFL < -1.0) dotFL = -1.0;
	float LOS = (float)(std::acos(dotFL) * RADTODEG);

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

	// 🔴 08-10 발견: **UT = ±180도가 롤/피치 권한이 동시에 사라지는 가짜 평형**이다.
	// UT는 "양력벡터로부터 표적까지 굴러야 하는 각"이므로 오차이고, 명령은 |UT|에 대해
	// 단조증가해야 한다. 그런데 원본은 sin(UT)라 90도에서 최대, **180도에서 0**이다 —
	// 오차가 최대일 때 명령이 사라진다. 게다가 같은 구간에서
	//   Roll_Effect = clamp(cos(UT), 0, 1) = 0  ->  PitchCMD = 0
	// 이라 당길 수도 없다. 즉 표적이 기체 배면 쪽에 오면 **구를 수도 당길 수도 없이 갇힌다.**
	// LOS < 3 가지는 `* LOS * (-0.1)`로 **부호까지 뒤집혀** 반대로 구른다.
	//
	// 실측(`AIP_dbg.dll` CTRL_DBG 계측, 직진표적 추격 seed0 t=100~105s):
	//   LOS=2.22 UT= 173.98 rollEff=0.000 errEff=0.785 pitch=-0.000
	//   LOS=5.92 UT= 178.12 rollEff=0.000 errEff=1.445 pitch=-0.000
	//   LOS=3.14 UT=-179.23 rollEff=0.000 errEff=0.815 pitch=-0.000
	//   errEff는 "당겨라"(0.5~1.4)라고 말하는데 cos(UT)=0이 전부 곱해 없앤다.
	//   이 상태로 수십 초를 머문다. 그 사이 조준은 전혀 개선되지 않고,
	//   간헐적 탈출-재진입이 반복되며 진동 항력만 쌓인다.
	//   결과: 전추력 수평 505m/s가 가능한데(`_thrust_probe.py`) 추격 실속도 421m/s,
	//   0.70스로틀 직진표적(407m/s)조차 못 잡음(`_degenerate_probe.py` duck 4/6 무득점).
	//
	// -> |UT| > 90 구간은 **방향만 맞으면 최대 롤**이 정답이다. 피치가 어차피 0이라
	//    이 구간에서 할 수 있는 일은 구르는 것뿐이고, 90도로 되돌아오면 아래 가지가
	//    이어받는다. 부호는 원본 sin(UT)와 동일하므로 **선회 방향은 바뀌지 않는다**
	//    (0<UT<180에서 sin>0, -180<UT<0에서 sin<0). 크기만 복원하는 수정이다.
	//    LOS 스케일은 아래 공통 `clamp(LOS/RollAuthDeg, 0, 1)`이 이미 담당하므로
	//    부호가 뒤집힌 `* LOS * (-0.1)` 가지는 제거한다.
	// ❌ 08-10 시도/기각: |UT|>90에서 최대롤(`sign(UT)`)로 교체 -> **명확히 악화.**
	//    직진표적 추격 seed0에서 거리가 단조증가(4161->6915m), 지속속도도 350~421 -> 365~383.
	//    이유: UT~180은 "표적이 기수보다 약간 **아래**"일 때도 성립한다(투영각이라 0.5도
	//    차이여도 UT는 180이다). 거기서 최대롤을 때리면 사소한 오차를 고치려고 180도를
	//    구르고, 넘어가면 반대쪽에서 또 구른다. 원본의 sin(UT)->0 감쇠는 버그가 아니라
	//    **"작은 오차에 큰 롤을 쓰지 마라"는 안정화 장치**였다.
	//    (단, 트랩 자체는 실재한다 — cos(UT)=0으로 피치도 0이라 그 구간에선 조준이
	//     전혀 개선되지 않는다. 해법은 롤 강화가 아니라 다른 곳에 있다.)
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

	// 🔴 08-10 발견: **조준점 주위의 롤 리밋사이클**. UTAngle은 기수 수직평면에 투영한
	// 표적 방향(Proj_TV)과 양력벡터 사이의 각이다. LOS가 작아지면 |Proj_TV| ~ 거리*sin(LOS)로
	// 줄지만(3000m/3도 = 157m) 부동소수점 정밀도 자체는 충분하다 — 문제는 정밀도가 아니라
	// **기하학적 악조건**이다. 표적이 거의 정면이면 내 기수가 1~2도만 흔들려도
	// Proj_TV의 **방향이 크게 회전한다**. 롤 명령은 그 방향을 최대 레이트로 쫓아가고,
	// 도착할 즈음 방향이 또 옮겨가 있어 수렴하지 못하고 계속 구른다.
	// 게인이 이를 증폭한다: RollCMD = sin(UT)*|sin(UT)|*3*clamp(LOS,0,1)이라
	// UT가 35도만 넘으면 이미 포화(=1.0)다. 감쇠 창은 `clamp(LOS, 0, 1)` — LOS 단위가
	// 도(deg)이므로 **0~1도뿐**이고, LOS 1도만 넘으면 전 게인이 그대로 걸린다.
	//
	// 실측(퇴화상대 위생검사 `_degenerate_probe.py`, 직진수평 표적 duck):
	//   t=30~190s 내내 ATA 0.1~5.8도(기수는 표적에 정확히 물려 있음)인데
	//   롤 각이 -151.9 / +176.7 / -95.5 / -101.9도 — **뒤집힌 채 계속 구른다.**
	//   결과: 코르크스크루 궤적이라 경로가 길어지고 유도항력으로 에너지를 잃어
	//   **0.70 스로틀 직진 표적을 전추력으로도 못 따라잡는다.**
	//   6시드 중 5시드 무득점, 최소거리가 시작거리(915m/762m) 그대로 = 1m도 못 좁힘.
	// 이 실패는 기존 상대 풀(전부 제대로 기동하는 BT)에서는 보이지 않았다 — 그쪽은
	// LOS가 작은 상태로 오래 머물지 않기 때문이다.
	//
	// 또한 이 구간이 곧 **3도->1도 수렴 병목**(시간의 80% 소실) 구간이기도 하다.
	// 감쇠 창을 넓혀 "각도가 못 믿을 만큼 작을 때는 구르지 않는다"로 만든다.
	// 작은 LOS에서는 필요한 롤 응답 자체도 작으므로 손해가 아니다.
	const float RollAuthDeg = 1.0f;
	RollCMD = RollCMD * clamp(LOS / RollAuthDeg, 0, 1);

	// 🔁 08-10 재도전: **롤 명령에 이동평균 필터**를 건다.
	// 러더에는 원래 20샘플 이동평균이 있고(위 MF[]), 그 정수절삭을 고쳤더니 단독 +7.3%가
	// 나왔다. 그런데 **롤에는 필터가 아예 없다** — 매 틱 계산된 값이 그대로 나간다.
	// 앞서 시도한 롤 감쇠창(RollAuthDeg 3/6/12)은 *게인*을 줄인 것이라 진동 자체는
	// 그대로 남았고 duck 무득점 4/6이 요지부동이었다. 필터는 다른 처방이다 —
	// 정상상태 권한은 유지하면서 **고주파 성분만** 걷어낸다.
	//
	// 왜 지금 다시 하는가(08-10 세 증상이 하나로 수렴):
	//   duck(직진 도주)  6시드 중 4판 무득점, 시작거리에서 1m도 못 좁힘
	//   arcV(수직 도주)  승점 6.0/10로 전 상대 중 최악, 무득점 8판, 득점 100%가 Phase3
	//   계측           전추력 수평 505m/s 가능한데 추격 실속도 421m/s (-17%)
	// 셋 다 "이탈하는 상대를 못 따라잡는다"이고 원인은 조준 진동의 유도항력이다.
	// RollWindow=1이면 필터 없음(원본과 동일)이라 A/B가 깨끗하다.
	//
	// ❌ 08-10 **기각.** duck에서는 이득이었지만(seed1 0.1535 -> 1.0170, 잃는 판 없음)
	//    코어 전수평가에서 명확히 나빴다. 같은 트리(e3) 동일 4상대 30시드:
	//        상대   필터없음   RollWindow=8
	//        v32     28.5      **25.5**  (27승 -> 21승, 무득점 3 -> 9)
	//        v29     27.0      **25.0**  (21승 1패)
	//        arcA    30.0        30.0
	//        arcE    30.0        30.0
	//        합계   115.5      **110.5**  (-5.0)
	//    원인이 지표에 그대로 보인다 — **`<1도` 조준시간이 2.34s -> 1.72s**로 떨어졌다.
	//    필터의 위상 지연이 정밀조준을 직접 갉아먹는다. 러더는 명령이 작고 보조축이라
	//    필터가 이득이지만, 롤은 주 조향축이라 지연의 대가가 훨씬 크다.
	//    (RollWindow=4도 duck seed5를 1.0091 -> 0.1676으로 죽였다 — 창이 좁으면
	//     감쇠는 부족하고 지연만 남는다)
	const int RollWindow = 1;
	if (RollWindow > 1)
	{
		RollMF[RollFilterIndex % 20] = RollCMD;
		RollFilterIndex++;
		int n = (RollWindow < 20) ? RollWindow : 20;
		float sum = 0.0f;
		for (int i = 0; i < n; i++)
		{
			int idx = (RollFilterIndex - 1 - i) % 20;
			if (idx < 0) idx += 20;
			sum += RollMF[idx];
		}
		RollCMD = sum / (float)n;
	}
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
	const float Kd = 0.02f;
	const float DGateDeg = 8.0f;
	const float DClamp = 0.25f;
	float dLOS = 0.0f;
	if (HasLastLOS)
		dLOS = (LOS - LastLOS) * 60.0f;   // deg/s (틱 0.0166666s 고정)
	LastLOS = LOS;
	HasLastLOS = true;

	float dTerm = (LOS < DGateDeg) ? clamp(Kd * dLOS, -DClamp, DClamp) : 0.0f;

	float ERROR_Effect = clamp(LOS / 6 + clamp(GetLOSErrorSUM(LOS) / 7.5, 0, 0.25)
	                           + dTerm, 0, 1.5);
	//float ERROR_Effect = clamp(LOS / 6, 0, 1.5);


	// ✅ 08-08: UT각 감쇠를 **코사인 투영**으로 교체. 기하학적으로 이쪽이 정확하다 —
	// 필요한 선회 중 실제로 당김 평면에 놓인 성분(UT=45도에서 선형 0.5 vs 코사인 0.707).
	// 중간 각도에서 피치를 더 줘 **수렴이 빨라진다**.
	// ⚠️ 단, 이 변경은 **위 D항 강화와 반드시 짝으로만** 이득이다. 공식조건 40시드 vs v32:
	//     선형 + D약(Kd .01/5도/.15)   +0.7474  38승 2패   <- 이전 기준선
	//     코사인 + D약                 +0.7096  37승 3패   (-5.1%)
	//     선형  + D강(Kd .02/8도/.25)  +0.7107  37승 3패   (-4.9%)
	//     **코사인 + D강**             **+0.8391  40승 0패**  (+12.3%)
	// 각각 단독으로는 -5%인데 합치면 +12.3%다. 코사인이 만드는 진동을 강화된 D가
	// 잡아줘서 '빠른 수렴 + 안정된 정착'이 동시에 성립한다 — <3도 7.94->9.86s(수렴),
	// <1도 1.36->1.49s(정착), 피격 0.1459->0.1395(최저).
	// **한쪽만 되돌리면 반드시 회귀한다. 짝으로 유지할 것.**
	float Roll_Effect = clamp(std::cos(UTAngle), 0, 1);

	float Horizon_Effect;
	if (std::abs(UTAngle * RADTODEG) <= 90)
	{
		Horizon_Effect = 1;
	}
	else
		Horizon_Effect = 0.5;

	//std::cout << "ERROR_Effect : " << ERROR_Effect << " Roll_Effect : " << Roll_Effect << " Horizon_Effect : " << Horizon_Effect << std::endl;

	// 08-08: 원본은 LOS<90에서는 연속식, LOS>=90에서는 통째로 -1인 **불연속** 구조였다.
	// LOS가 90도 바로 아래일 때 Roll_Effect(=1-|UT|/90)가 0에 가까우면 PitchCMD도 0에
	// 가까운데, 90도를 넘는 순간 -1로 튄다 — 최대 1.0 크기의 계단이다.
	// 이 불연속이 롤 90/270도를 지날 때마다 부호를 뒤집으며 조준을 고착시켰고, 그래서
	// CPPBehaviorTree에 75도 보어사이트 클램프(VP를 기수 기준 75도 이내로 제한)를 우회책
	// 으로 넣어야 했다. 즉 클램프는 이 계단을 피하려고 **표적이 크게 벗어난 상황마다 VP를
	// 왜곡**하는 대가를 치르고 있었다(방어/리버설 국면에 직접 영향).
	// ❌ 그래서 70~110도 선형 블렌드로 계단을 없애고 클램프를 완화해봤으나 **실패**했다.
	// 공식조건 40시드 vs v32 (순이득 HP/판, 승/패, <1도):
	//        75도(현행)  +0.7474  38/2  1.36s
	//        90도        +0.6423  31/8  1.14s   (-14.1%)
	//        120도       +0.7010  32/7  1.10s   (-6.2%)
	// 계단을 없앴는데도 나빠졌으므로 원인은 불연속이 아니라 제어기 구조다 —
	// Roll_Effect = 1 - |UT|/90 이 큰 각도에서 0으로 죽어, VP를 멀리 찍어도 제어기가
	// 그쪽으로 갈 능력 자체가 없다. 즉 75도 클램프는 우회책이 아니라 **제어기를 자기가
	// 잘 동작하는 영역 안에 붙잡아두는 장치**였다. 원본 불연속 구조로 원복한다.
	// (클램프가 75도면 제어기가 보는 LOS는 항상 75도 이하라 이 분기는 발동하지 않는다)
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

	// 🔴 08-10 최종 방어선. `clamp`는 **NaN을 걸러내지 못한다**:
	//     if (input <= down) ... else if (input >= up) ... else return input;
	//   NaN은 두 비교가 모두 거짓이라 **그대로 반환된다.**
	// acos 인자를 클램프해 발생원은 막았지만, 어떤 경로로든 NaN이 새면 조종 명령이
	// 통째로 NaN인 채 서버로 나간다. 여기서 0으로 치환한다(= 스틱 중립).
	StickValue Result;
	Result.RollCMD = clamp(RollCMD, -1, 1);
	Result.PitchCMD = clamp(PitchCMD, -1, 1);
	Result.RudderCMD = clamp(RudderCMD, -1, 1);
	if (_isnan(Result.RollCMD)   != 0) Result.RollCMD = 0.0f;
	if (_isnan(Result.PitchCMD)  != 0) Result.PitchCMD = 0.0f;
	if (_isnan(Result.RudderCMD) != 0) Result.RudderCMD = 0.0f;
	//Result.RudderCMD = RudderCMD;
	return Result;
}
