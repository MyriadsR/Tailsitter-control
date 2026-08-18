#include "planning/flat_output_trajectory.hpp"

#include <cmath>
#include <stdexcept>

namespace planning
{

void FlatOutputTrajectory::update(const std::vector<double>& durations,
                                  const PositionMatrix& position_waypoints,
                                  const YawMatrix& yaw_waypoints)
{
    if (durations.empty() || position_waypoints.rows() != static_cast<Eigen::Index>(durations.size() + 1) ||
        yaw_waypoints.rows() != position_waypoints.rows())
    {
        throw std::invalid_argument("durations and waypoint dimensions are inconsistent");
    }
    for (double duration : durations)
    {
        if (!(duration > 0.0) || !std::isfinite(duration))
            throw std::invalid_argument("every segment duration must be finite and positive");
    }

    SplineTrajectory::BoundaryConditions<3> position_boundary;
    SplineTrajectory::BoundaryConditions<1> yaw_boundary;
    position_.update(durations, position_waypoints, 0.0, position_boundary);
    yaw_.update(durations, unwrapYawWaypoints(yaw_waypoints), 0.0, yaw_boundary);
}

FlatOutputSample FlatOutputTrajectory::sample(double time) const
{
    const double t = std::max(0.0, std::min(time, duration()));
    FlatOutputSample output;
    output.time = t;
    output.position = position_.getTrajectory().evaluate(t, SplineTrajectory::Deriv::Pos);
    output.velocity = position_.getTrajectory().evaluate(t, SplineTrajectory::Deriv::Vel);
    output.acceleration = position_.getTrajectory().evaluate(t, SplineTrajectory::Deriv::Acc);
    output.jerk = position_.getTrajectory().evaluate(t, SplineTrajectory::Deriv::Jerk);
    output.snap = position_.getTrajectory().evaluate(t, SplineTrajectory::Deriv::Snap);
    output.crackle = position_.getTrajectory().evaluate(t, SplineTrajectory::Deriv::Crackle);
    output.yaw = yaw_.getTrajectory().evaluate(t, SplineTrajectory::Deriv::Pos).x();
    output.yaw_rate = yaw_.getTrajectory().evaluate(t, SplineTrajectory::Deriv::Vel).x();
    output.yaw_acceleration = yaw_.getTrajectory().evaluate(t, SplineTrajectory::Deriv::Acc).x();
    output.yaw_jerk = yaw_.getTrajectory().evaluate(t, SplineTrajectory::Deriv::Jerk).x();
    return output;
}

double FlatOutputTrajectory::energy(double yaw_weight) const
{
    return position_.getEnergy() + yaw_weight * yaw_.getEnergy();
}

Eigen::VectorXd FlatOutputTrajectory::energyTimeGradient(double yaw_weight) const
{
    return position_.getEnergyGradTimes() + yaw_weight * yaw_.getEnergyGradTimes();
}

double FlatOutputTrajectory::duration() const { return position_.getDuration(); }
const std::vector<double>& FlatOutputTrajectory::durations() const { return position_.getTimeSegments(); }
const FlatOutputTrajectory::PositionMatrix& FlatOutputTrajectory::positionWaypoints() const { return position_.getSpacePoints(); }
const FlatOutputTrajectory::YawMatrix& FlatOutputTrajectory::yawWaypoints() const { return yaw_.getSpacePoints(); }

FlatOutputTrajectory::YawMatrix unwrapYawWaypoints(const FlatOutputTrajectory::YawMatrix& yaw_waypoints)
{
    FlatOutputTrajectory::YawMatrix result = yaw_waypoints;
    constexpr double two_pi = 2.0 * 3.14159265358979323846;
    for (Eigen::Index i = 1; i < result.rows(); ++i)
    {
        double delta = result(i, 0) - result(i - 1, 0);
        delta -= two_pi * std::round(delta / two_pi);
        result(i, 0) = result(i - 1, 0) + delta;
    }
    return result;
}

}  // namespace planning
