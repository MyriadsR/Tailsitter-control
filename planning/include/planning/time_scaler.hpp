#pragma once

#include "planning/feasibility_checker.hpp"

namespace planning
{

struct ScaleEvaluation
{
    double scale{1.0};
    bool feasible{false};
    double max_normalized_violation{0.0};
};

struct TimeScaleResult
{
    bool found{false};
    double minimum_feasible_scale{0.0};
    std::vector<double> durations;
    FeasibilityReport report;
    std::vector<ScaleEvaluation> coarse_scan;
};

class TimeScaler
{
public:
    explicit TimeScaler(FeasibilityChecker checker);

    TimeScaleResult search(const FlatOutputTrajectory::PositionMatrix& position_waypoints,
                           const FlatOutputTrajectory::YawMatrix& yaw_waypoints,
                           const std::vector<double>& base_durations,
                           double minimum_scale,
                           double maximum_scale,
                           int coarse_samples = 25,
                           double scale_tolerance = 1e-3) const;

private:
    FeasibilityChecker checker_;
};

}  // namespace planning

