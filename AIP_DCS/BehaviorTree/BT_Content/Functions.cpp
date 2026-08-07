#include "Functions.h"

namespace BTFunc
{
	void AddNodeExcute(std::string * out, std::string input)
	{
		out->append(input);
		out->append("\n");
	}
	void SaveTextData(std::string * tempString, std::string * BT_Text)
	{
		if (tempString != nullptr && BT_Text != nullptr)
		{
			if (tempString->length() > 910)
				tempString->clear();

			BT_Text->clear();

			BT_Text->append((*tempString));
			tempString->clear();
		}
	}

	void ApplyClosureThrottle(CPPBlackBoard* BB)
	{
		if (BB == nullptr) return;

		double dist = BB->Distance;
		const double WEZ_MAX = 914.0, WEZ_MIN = 152.0;

		// 08-07: 주최측이 데미지 산식을 공개하면서 이 구간 설정이 틀렸음이 확인됐다.
		//     d_wez(r) = (3000ft - r) / 2500ft   (500~3000ft 구간, 그 밖은 0)
		// 즉 밴드(152~914m)는 균질하지 않고 152m에서 계수 1.0, 914m에서 0인 선형 램프다.
		//     250m -> 0.872,  400m -> 0.675,  600m -> 0.413,  850m -> 0.085
		// 기존 설정은 밴드를 균질하게 보고 400m를 스윗스팟으로 잡아 그 아래에서는 -10m/s로
		// 오히려 물러났는데, 이건 계수가 가장 높은 구간을 스스로 회피하는 동작이었다.
		// 실측(공식조건 20시드): 밴드 체류의 42.3%가 500~914m(계수 0.14~0.41)에 몰려있고
		// 평균계수가 0.584 — 이걸 안쪽으로 당기면 산술적으로 ~1.45배다.
		// 새 목표대(200~300m)는 계수 0.87~0.94이면서 152m 사각지대(데미지 0)와는
		// 충분한 여유를 둔다.
		// ❌ 08-07 실험 결과: 위 산술(계수 1.45배)에 근거해 목표대를 200~300m로 당겼으나
		// **역효과 확인, 원복.** 공식조건 40시드 vs v32:
		//     제어기수정만        38승 2패 0무, WEZ 99.0
		//     +거리정책(200~300m) 35승 5패 0무, WEZ 90.7   (승 -3, WEZ -8.4%)
		// 데미지는 `계수 x 조준유지시간`인데, 가까워질수록 표적의 각속도가 커져
		// **조준유지시간이 계수보다 빠르게 무너진다.** 즉 계수만 보고 당기면 손해다.
		// 실측 교전 중앙값 376m(계수 0.71)는 우연이 아니라 이미 최적점 근처였다.
		// 원래 임계값(400m에서 속도매칭)을 유지한다.
		double dvTarget;                        // 목표 속도차(m/s) = 내속도-적속도
		if (dist > WEZ_MAX)      dvTarget = 999.0;  // 사거리 밖: 제한 없이 접근
		else if (dist > 400.0)   dvTarget =   0.0;  // 사거리 바깥쪽: 속도 매칭
		else if (dist > WEZ_MIN) dvTarget = -10.0;  // 사거리 안쪽: 살짝 후퇴
		else                     dvTarget = -25.0;  // 과근접: 적극 후퇴

		if (dvTarget > 900.0)
		{
			BB->Throttle = 1.0f;
			return;
		}

		double speedMargin = (double)BB->MySpeed_MS - (double)BB->TargetSpeed_MS;
		double dvErr = speedMargin - dvTarget;      // +면 내가 너무 빠름 -> 줄여야
		double u = 1.0 - dvErr * 0.012;
		if (u > 1.0)  u = 1.0;
		if (u < 0.55) u = 0.55;
		BB->Throttle = (float)u;
	}

}