#pragma once
#include "../../Geometry/Vector3.h"
#include "../../Geometry/EulerAngle.h"
#include "../../Geometry/Quaternion.h"
#include "BlackBoard/CPPBlackBoard.h"
#include <vector>

using namespace BT_Geometry;

namespace BTFunc
{
	/*
	국과연에서 요구한 비헤비어트리 결정 과정을 보여주기 위해 각 노드에서 실행 과정(결과)를 문자열로 저장하기 위한 함수
	기존 문자열, 추가할 문자열
	*/
	void AddNodeExcute(std::string * out, std::string input);
	void SaveTextData(std::string * tempString, std::string * BT_Text);

	// 폐쇄율(dV) 관리: BB->Throttle을 목표 속도차(dV = 내속도-적속도) 기준으로 설정.
	// 조준(ATA)은 되는데 사거리를 그냥 관통해버리는 "사격전환 벽" 대응 — 거리를 직접
	// 제어하지 않고 dV를 0(또는 WEZ 안에서는 음수)으로 수렴시켜 사거리 안에 머무는 시간을 번다.
	void ApplyClosureThrottle(CPPBlackBoard* BB);

}