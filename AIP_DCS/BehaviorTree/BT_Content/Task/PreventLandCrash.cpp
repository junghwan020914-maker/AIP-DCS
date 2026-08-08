#include "PreventLandCrash.h"

namespace Action
{
	PortsList PreventLandCrash::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
			InputPort<double>("FloorHard", 1000.0, "hard pull-up altitude (m)"),
			InputPort<double>("FloorSoft", 2000.0, "soft blend altitude (m)"),
			// 08-08 신규: 급강하 **진입 차단**. 아래 tick() 주석 참고.
			InputPort<double>("DiveGuardAlt", 0.0, "고도 이 값 아래에서 하방 VP 제한 (0=끄기)"),
			InputPort<double>("MaxDiveDeg", 30.0, "DiveGuardAlt 지점에서 허용할 최대 강하각(도)")
		};
	}

	NodeStatus PreventLandCrash::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");
		double FloorHard = getInput<double>("FloorHard").value();
		double FloorSoft = getInput<double>("FloorSoft").value();

		double DiveGuardAlt = 0.0, MaxDiveDeg = 30.0;
		{
			Optional<double> a = getInput<double>("DiveGuardAlt");
			Optional<double> b = getInput<double>("MaxDiveDeg");
			if (a) DiveGuardAlt = a.value();
			if (b) MaxDiveDeg = b.value();
		}

		Vector3 my  = (*BB)->MyLocation_Cartesian;
		double alt = my.Z;						// Z = 고도(up, m)

		// ❌ 08-08 실험 실패(원복): 급강하 **진입 차단**.
		// 근거는 정확했다 — 추락 시드 경위(`_crash_forensics.py`, 30시드 vs v29):
		//     1500m 통과 시점 피치 중앙값 **-65.5도**, 속도 335m/s, 강하율 **-287m/s**
		// 남은 1200m를 약 4초에 통과하므로 사후 pull-up으로는 물리적으로 못 뺀다.
		// 그래서 DiveGuardAlt(3000m) 아래에서 강하각 상한을 고도비례로 걸어봤다(35도->0도).
		// 결과: 추락 14 -> 11판(목표 2판 미달)인데 **arcA 매치업이 붕괴**했다.
		//     arcA  20승4패 +0.2789  ->  13승13패 **-0.0467**,  피격 0.1350 -> **0.4592(3.4배)**
		// 이유: 강하각 제한이 **아래로 도는 회피 기동**을 봉쇄한다. 최대선회형 상대에겐
		// 수직 평면을 쓰는 방어가 필수인데 그걸 막으니 그대로 얻어맞는다.
		// "수평 성분은 살리니 선회는 유지된다"고 봤으나 **수직 선회가 곧 방어**였다.
		//
		// ⚠️ 저고도 기동을 억제하는 방식은 네 번 모두 실패했다(플로어 800/1500 -> 1000/2000
		//    -> 1200/2400, 그리고 이 강하각 제한). 전부 방어력을 깎는다.
		//    **급강하는 추락의 원인이자 동시에 방어 수단이다.**
		// -> 다음 방향은 억제가 아니라 회복 타이밍이다. 고도만 보지 말고
		//    **`고도 / 강하율` = 지면까지 남은 초**로 위험을 판단할 것
		//    (1500m에서 -287m/s면 4.2초 — 고도 1500m라는 사실보다 훨씬 정확한 신호).
		// 포트(DiveGuardAlt/MaxDiveDeg)는 남겨두되 XML에서 지정하지 않으면 꺼진다.

		if (alt >= FloorSoft)
			return NodeStatus::SUCCESS;			// 충분히 높음 -> 개입 안 함

		Vector3 WorldUp(0.0, 0.0, 1.0);

		// 수평 추격 방향(기동 VP 기준)은 살리되, 상승을 크게 실어 확실히 pull-up
		Vector3 cur = (*BB)->VP_Cartesian;
		Vector3 horiz(cur.X - my.X, cur.Y - my.Y, 0.0);
		if (horiz.length() < 1e-3)
		{
			Vector3 F = (*BB)->MyForwardVector;
			horiz = Vector3(F.X, F.Y, 0.0);
		}
		if (horiz.length() < 1e-3) horiz = Vector3(1, 0, 0);
		horiz.normalize();
		horiz = horiz * 1500.0;								// 수평은 작게(상승 우선)

		// 강한 상승 목표점 (원래 검증된 값: Up*8000)
		Vector3 climbVP = my + horiz + WorldUp * 8000.0;

		if (alt < FloorHard)
		{
			(*BB)->VP_Cartesian = climbVP;					// 무조건 강한 pull-up
		}
		else
		{
			double t = (FloorSoft - alt) / (FloorSoft - FloorHard);
			if (t < 0.0) t = 0.0; if (t > 1.0) t = 1.0;
			(*BB)->VP_Cartesian = cur * (1.0 - t) + climbVP * t;
		}

		return NodeStatus::SUCCESS;
	}
}
