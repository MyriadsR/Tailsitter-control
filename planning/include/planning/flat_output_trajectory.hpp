#pragma once

#include "planning/planning_types.hpp"

#include "SplineTrajectory.hpp"

namespace planning
{

class FlatOutputTrajectory
{
public:
    using PositionSpline = SplineTrajectory::SepticSplineND<3>;
    using YawSpline = SplineTrajectory::CubicSplineND<1>;
    using PositionMatrix = PositionSpline::MatrixType;
    using YawMatrix = YawSpline::MatrixType;

    struct CostGradient
    {
        PositionMatrix position_inner_points;
        YawMatrix yaw_inner_points;
        Eigen::VectorXd times;
    };

    void update(const std::vector<double>& durations,
                const PositionMatrix& position_waypoints,
                const YawMatrix& yaw_waypoints);

    FlatOutputSample sample(double time) const;
    double energy(double yaw_weight) const;
    Eigen::VectorXd energyTimeGradient(double yaw_weight) const;
    CostGradient energyGradient(double yaw_weight) const;
    CostGradient propagateCostGradient(
        const PositionMatrix& partial_position_coefficients,
        const Eigen::VectorXd& partial_position_times,
        const YawMatrix& partial_yaw_coefficients,
        const Eigen::VectorXd& partial_yaw_times);
    double duration() const;
    const std::vector<double>& durations() const;
    const PositionMatrix& positionWaypoints() const;
    const YawMatrix& yawWaypoints() const;

private:
    PositionSpline position_;
    YawSpline yaw_;
};

FlatOutputTrajectory::YawMatrix unwrapYawWaypoints(
    const FlatOutputTrajectory::YawMatrix& yaw_waypoints);

}  // namespace planning
