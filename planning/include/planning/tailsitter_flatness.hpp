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

    const VehicleParameters& parameters() const { return parameters_; }

private:
    VehicleParameters parameters_;
};

}  // namespace planning

