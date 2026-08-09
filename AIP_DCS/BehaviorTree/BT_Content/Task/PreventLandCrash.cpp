#include "PreventLandCrash.h"

namespace Action
{
	PortsList PreventLandCrash::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
			InputPort<double>("FloorHard", 1000.0, "hard pull-up altitude (m)"),
			InputPort<double>("FloorSoft", 2000.0, "soft blend altitude (m)"),
			// 08-08 실패(원복). 포트만 남김 — 지정 안 하면 꺼진다. tick() 주석 참고.
			InputPort<double>("DiveGuardAlt", 0.0, "고도 이 값 아래에서 하방 VP 제한 (0=끄기)"),
			InputPort<double>("MaxDiveDeg", 30.0, "DiveGuardAlt 지점에서 허용할 최대 강하각(도)"),
			// 08-09 신규: 시간여유(TTG) 기반 개입. 아래 tick() 주석 참고.
			InputPort<double>("TTGSeconds", 0.0, "지면까지 남은 시간이 이 값 미만이면 pull-up (0=끄기)")
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

		double TTGSeconds = 0.0;
		{
			Optional<double> t = getInput<double>("TTGSeconds");
			if (t) TTGSeconds = t.value();
		}

		Vector3 my  = (*BB)->MyLocation_Cartesian;
		double alt = my.Z;						// Z = 고도(up, m)

		// ── 강하율 추정 (위치 차분 + EMA) ──
		// 에피소드/라운드 경계에서 위치가 점프하면 강하율이 터무니없이 잡히므로
		// 물리적으로 불가능한 값(|v| > 700 m/s)은 버린다.
		double sink = 0.0;						// +면 하강 중(m/s)
		{
			const double dt = (*BB)->DeltaSecond > 1e-6 ? (*BB)->DeltaSecond : (1.0 / 60.0);
			if (HasLastAlt)
			{
				double v = (LastAlt - alt) / dt;
				if (v > -700.0 && v < 700.0)
					SinkRateEMA = SinkRateEMA * 0.8 + v * 0.2;
				else
					SinkRateEMA = 0.0;			// 리스폰으로 판단 -> 초기화
			}
			LastAlt = alt;
			HasLastAlt = true;
			sink = SinkRateEMA;
		}

		// ── 시간여유(TTG) 기반 개입 (08-09 신규) ─────────────────────────
		// 고도 기반 억제는 네 번 모두 실패했다(플로어 800/1500 -> 1000/2000 -> 1200/2400,
		// 강하각 제한). 이유는 **고도만으로는 위험한 급강하와 안전한 급강하를 구분할 수
		// 없기 때문**이다. 실측상 추락 시드는 1500m에서 강하율 -287m/s(남은 4.2초)인데,
		// 같은 1500m라도 -50m/s면 30초가 남아 전혀 위험하지 않다. 고도로 자르면 둘 다
		// 막게 되고, 그때 잃는 것이 **아래로 도는 회피 기동 = 방어력**이었다
		// (강하각 제한 실험에서 arcA 피격이 3.4배로 폭증).
		// 그래서 판정을 고도가 아니라 **지면까지 남은 시간**으로 바꾼다.
		// 규정 하한 300m를 기준선으로 삼아 여유를 계산하고, 그 여유가 TTGSeconds 미만일
		// 때만 강한 pull-up을 건다. 완만한 강하는 시간여유가 크므로 건드리지 않는다.
		bool ttgCritical = false;
		if (TTGSeconds > 0.0 && sink > 1.0)		// 실제로 하강 중일 때만
		{
			double margin = alt - 300.0;		// 규정 하한까지 남은 고도
			if (margin < 0.0) margin = 0.0;
			double ttg = margin / sink;			// 남은 초
			ttgCritical = (ttg < TTGSeconds);
		}
		// ────────────────────────────────────────────────────────────────

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

		if (alt >= FloorSoft && !ttgCritical)
			return NodeStatus::SUCCESS;			// 충분히 높고 시간여유도 있음 -> 개입 안 함

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

		if (alt < FloorHard || ttgCritical)
		{
			// 시간여유가 임계 미만이면 고도와 무관하게 최우선으로 기수를 든다.
			// 고도가 높아도 -287m/s로 꽂히는 중이면 지금 빼야 4초 뒤에 산다.
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
