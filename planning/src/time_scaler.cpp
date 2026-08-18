#include "planning/time_scaler.hpp"

#include <algorithm>
#include <stdexcept>

namespace planning
{
namespace
{
FlatOutputTrajectory buildScaled(const FlatOutputTrajectory::PositionMatrix& position_waypoints,
                                 const FlatOutputTrajectory::YawMatrix& yaw_waypoints,
                                 const std::vector<double>& base_durations,
                                 double scale)
{
    std::vector<double> durations = base_durations;
    for (double& duration : durations)
        duration *= scale;
    FlatOutputTrajectory trajectory;
    trajectory.update(durations, position_waypoints, yaw_waypoints);
    return trajectory;
}
}  // namespace

TimeScaler::TimeScaler(FeasibilityChecker checker) : checker_(std::move(checker)) {}

TimeScaleResult TimeScaler::search(
    const FlatOutputTrajectory::PositionMatrix& position_waypoints,
    const FlatOutputTrajectory::YawMatrix& yaw_waypoints,
    const std::vector<double>& base_durations,
    double minimum_scale,
    double maximum_scale,
    int coarse_samples,
    double scale_tolerance) const
{
    if (!(minimum_scale > 0.0 && maximum_scale > minimum_scale) || coarse_samples < 2 ||
        !(scale_tolerance > 0.0))
        throw std::invalid_argument("invalid time-scale search bounds");

    TimeScaleResult result;
    result.coarse_scan.reserve(static_cast<std::size_t>(coarse_samples));
    for (int i = 0; i < coarse_samples; ++i)
    {
        const double scale = minimum_scale + (maximum_scale - minimum_scale) * i / (coarse_samples - 1);
        const auto report = checker_.check(buildScaled(position_waypoints, yaw_waypoints, base_durations, scale));
        result.coarse_scan.push_back({scale, report.feasible, report.max_normalized_violation});
    }

    std::vector<double> feasible_candidates;
    if (result.coarse_scan.front().feasible)
        feasible_candidates.push_back(minimum_scale);
    for (std::size_t i = 1; i < result.coarse_scan.size(); ++i)
    {
        if (result.coarse_scan[i].feasible && !result.coarse_scan[i - 1].feasible)
        {
            double lower = result.coarse_scan[i - 1].scale;
            double upper = result.coarse_scan[i].scale;
            while (upper - lower > scale_tolerance)
            {
                const double middle = 0.5 * (lower + upper);
                const bool feasible = checker_.check(
                    buildScaled(position_waypoints, yaw_waypoints, base_durations, middle)).feasible;
                if (feasible)
                    upper = middle;
                else
                    lower = middle;
            }
            feasible_candidates.push_back(upper);
        }
    }
    if (feasible_candidates.empty())
        return result;

    result.found = true;
    result.minimum_feasible_scale = *std::min_element(feasible_candidates.begin(), feasible_candidates.end());
    result.durations = base_durations;
    for (double& duration : result.durations)
        duration *= result.minimum_feasible_scale;
    result.report = checker_.check(
        buildScaled(position_waypoints, yaw_waypoints, base_durations, result.minimum_feasible_scale), true);
    return result;
}

}  // namespace planning
