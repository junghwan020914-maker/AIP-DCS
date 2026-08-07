#include "DECO_EnergyCheck.h"

namespace Action
{
	PortsList DECO_EnergyCheck::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
			InputPort<std::string>("UpDown"),
			InputPort<double>("Margin", 0.0, "energy margin in meters")
		};
	}

	NodeStatus DECO_EnergyCheck::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");
		std::string UD = getInput<std::string>("UpDown").value();
		double Margin = getInput<double>("Margin").value();

		float MyEs  = (*BB)->MySpecificEnergy;
		float TgtEs = (*BB)->TargetSpecificEnergy;

		if (UD == "Greater")
			return (MyEs >= TgtEs + Margin) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
		else if (UD == "Less")
			return (MyEs <= TgtEs - Margin) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;

		// UpDown 오타 확인 필요 (Greater/Less 아님)
		return NodeStatus::FAILURE;
	}
}
