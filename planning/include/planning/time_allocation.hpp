#pragma once

#include "planning/flat_output_trajectory.hpp"

namespace planning
{

struct TimeAllocationResult
{
    std::vector<double> durations;
    double initial_cost{0.0};
    double final_cost{0.0};
    int evaluations{0};
    bool optimized{false};
};

class FixedTotalTimeAllocator
{
public:
    explicit FixedTotalTimeAllocator(double yaw_weight = 1.0);

    TimeAllocationResult optimize(double total_time,
                                  const FlatOutputTrajectory::PositionMatrix& position_waypoints,
                                  const FlatOutputTrajectory::YawMatrix& yaw_waypoints) const;

    static std::vector<double> distanceInitialGuess(
        double total_time,
        const FlatOutputTrajectory::PositionMatrix& position_waypoints);
    static std::vector<double> durationsFromTau(double total_time,
                                                const std::vector<double>& tau_free);

private:
    double yaw_weight_;
};

}  // namespace planning

