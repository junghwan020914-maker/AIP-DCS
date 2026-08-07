#include "WEZUpdate.h"

namespace Action
{
	PortsList WEZUpdate::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
			InputPort<double>("ConeDeg", 1.0,   "WEZ cone half-angle in deg (2deg total)"),
			InputPort<double>("MinM",    152.0, "min range in meters (500ft)"),
			InputPort<double>("MaxM",    914.0, "max range in meters (3000ft)")
		};
	}

	NodeStatus WEZUpdate::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");
		double Cone = getInput<double>("ConeDeg").value();
		double MinM = getInput<double>("MinM").value();
		double MaxM = getInput<double>("MaxM").value();

		float d      = (*BB)->Distance;
		float myLOS  = (*BB)->Los_Degree;			// 내 기수 -> 적
		float tgtLOS = (*BB)->Los_Degree_Target;	// 적 기수 -> 나

		bool inRange = (d >= MinM && d <= MaxM);

		(*BB)->TargetInMyWEZ  = inRange && (myLOS  <= Cone);
		(*BB)->IamInTargetWEZ = inRange && (tgtLOS <= Cone);

		return NodeStatus::SUCCESS;
	}
}
