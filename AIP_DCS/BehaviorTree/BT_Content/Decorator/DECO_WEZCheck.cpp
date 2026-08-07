#include "DECO_WEZCheck.h"

namespace Action
{
	PortsList DECO_WEZCheck::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
			InputPort<std::string>("Side", "Target", "Target = enemy in my WEZ, Me = I am in enemy WEZ")
		};
	}

	NodeStatus DECO_WEZCheck::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");
		std::string side = getInput<std::string>("Side").value();

		bool ok = (side == "Me") ? (*BB)->IamInTargetWEZ : (*BB)->TargetInMyWEZ;
		return ok ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
	}
}
