#pragma once

#include "planning/flat_output_trajectory.hpp"
#include "planning/tailsitter_flatness.hpp"

namespace planning
{

class FeasibilityChecker
{
public:
    explicit FeasibilityChecker(TailsitterFlatness flatness,
                                double sample_interval = 0.002,
                                double actuator_margin = 1.0,
                                bool enforce_velocity_constraint = true);

    FeasibilityReport check(const FlatOutputTrajectory& trajectory,
                            bool keep_predictions = false) const;

private:
    TailsitterFlatness flatness_;
    double sample_interval_;
    double actuator_margin_;
    bool enforce_velocity_constraint_;
};

}  // namespace planning
