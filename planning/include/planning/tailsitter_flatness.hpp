#pragma once

#include "planning/planning_types.hpp"

namespace planning
{

class TailsitterFlatness
{
public:
    explicit TailsitterFlatness(VehicleParameters parameters);

    ActuatorPrediction forward(const FlatOutputSample& sample,
                               const BranchState& previous_branch = BranchState{}) const;

    ActuatorLinearization linearize(
        const FlatOutputSample& sample,
        const BranchState& previous_branch = BranchState{}) const;

    // Linearize on attitude branches already selected by a previous forward()
    // call for the same sample and branch state. Skips the duplicate forward
    // evaluation; the caller supplies that prediction so the linearization
    // carries the complete actuator record.
    ActuatorLinearization linearize(
        const FlatOutputSample& sample,
        const ActuatorPrediction& prediction,
        int roll_branch,
        int pitch_branch) const;

    const VehicleParameters& parameters() const { return parameters_; }

private:
    VehicleParameters parameters_;
};

}  // namespace planning
