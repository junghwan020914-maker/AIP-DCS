#include "DECO_AltitudeCheck.h"

namespace Action
{
	PortsList DECO_AltitudeCheck::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
			InputPort<double>("MinAlt", 3000.0, "minimum altitude in meters")
		};
	}

	NodeStatus DECO_AltitudeCheck::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");
		double MinAlt = getInput<double>("MinAlt").value();

		double alt = (*BB)->MyLocation_Cartesian.Z;		// Z = 고도(up, m)

		// 최저고도 미만이면 SUCCESS (상승 필요)
		return (alt < MinAlt) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
	}
}
