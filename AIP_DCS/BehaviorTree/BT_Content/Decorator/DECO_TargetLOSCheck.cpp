#include "DECO_TargetLOSCheck.h"

namespace Action
{
	PortsList DECO_TargetLOSCheck::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
			InputPort<std::string>("UpDown"),
			InputPort<std::string>("InputLOS")
		};
	}

	NodeStatus DECO_TargetLOSCheck::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");
		Optional<std::string> UpOrDown = getInput<std::string>("UpDown");
		Optional<std::string> Dist = getInput<std::string>("InputLOS");

		float CurrentLOS = (*BB)->Los_Degree_Target;
		std::string UD = UpOrDown.value();
		float InputLOS = std::stof(Dist.value());

		if (UD == "Greater")
		{
			if (CurrentLOS >= InputLOS)
			{
				std::fprintf(stdout, "[WOBBLE_FIRE] team=%d losTgt=%.2f\n", (int)(*BB)->Team, CurrentLOS);
				std::fflush(stdout);
				return NodeStatus::SUCCESS;
			}
			else
			{
				return NodeStatus::FAILURE;
			}
		}
		else if (UD == "Less")
		{
			if (CurrentLOS <= InputLOS)
			{
				return NodeStatus::SUCCESS;
			}
			else
				return NodeStatus::FAILURE;
		}
		else
		{
			return NodeStatus::FAILURE;
		}
	}
}
