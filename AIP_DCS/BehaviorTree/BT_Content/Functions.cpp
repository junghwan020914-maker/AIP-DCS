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