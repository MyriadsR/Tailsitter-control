#include "planning/actuator_penalty_optimizer.hpp"
#include "planning/feasibility_checker.hpp"

#include <cmath>
#include <iostream>
#include <numeric>

namespace
{

int fail(const char* message)
{
    std::cerr << "FAIL: " << message << '\n';
    return 1;
}

}  // namespace

int main()
{
    constexpr double distance = 12.0;
    constexpr int waypoint_count = 5;
    constexpr int segment_count = waypoint_count - 1;
    planning::FlatOutputTrajectory::PositionMatrix positions(waypoint_count, 3);
    planning::FlatOutputTrajectory::YawMatrix yaws(waypoint_count, 1);
    planning::FlatOutputTrajectory::YawMatrix zero_yaws =
        planning::FlatOutputTrajectory::YawMatrix::Zero(waypoint_count, 1);
    for (int waypoint = 0; waypoint < waypoint_count; ++waypoint)
    {
        const double fraction =
            static_cast<double>(waypoint) / static_cast<double>(segment_count);
        positions.row(waypoint) << distance * fraction, 0.0, -10.0;
        yaws(waypoint, 0) = fraction * 3.14159265358979323846;
    }
    const std::vector<double> initial_durations(segment_count, 0.7);
    const std::vector<int> free_waypoints{1, 2, 3};

    planning::ActuatorPenaltyConfig config;
    config.enable_high_level_constraints = true;
    config.auxiliary_half_range = Eigen::Vector3d(1.5, 2.0, 1.5);
    config.samples_per_segment = 12;
    config.max_evaluations = 180;
    planning::ActuatorPenaltyOptimizer optimizer(
        planning::TailsitterFlatness(planning::VehicleParameters::paperVehicle()),
        config);

    const auto zero_check = optimizer.checkInitialJointGradient(
        initial_durations, positions, zero_yaws, free_waypoints);
    const auto pi_check = optimizer.checkInitialJointGradient(
        initial_durations, positions, yaws, free_waypoints);
    if (!zero_check.valid || !pi_check.valid ||
        zero_check.maximum_relative_error >= 1e-3 ||
        pi_check.maximum_relative_error >= 1e-3)
    {
        std::cerr << "joint gradient errors: yaw0="
                  << zero_check.maximum_relative_error << ", yawpi="
                  << pi_check.maximum_relative_error << '\n';
        return fail("joint objective gradient failed finite-difference validation");
    }

    const auto result = optimizer.optimizeJoint(
        initial_durations, positions, yaws, free_waypoints);
    if (!result.optimized || result.evaluations <= 0)
        return fail("joint optimizer did not run");
    if (!(result.final_terms.total <= result.initial_terms.total * (1.0 + 1e-8)))
        return fail("joint optimizer increased its objective");
    if (!(result.final_terms.total_time > 0.0) ||
        !std::isfinite(result.final_terms.total_time))
        return fail("joint optimizer produced an invalid total time");
    if (!(result.initial_terms.time > 0.0) ||
        !(result.initial_terms.actuator.velocity > 0.0) ||
        !(result.initial_terms.actuator.collective_thrust > 0.0))
    {
        return fail("joint regression did not activate time and high-level terms");
    }
    const double initial_total =
        std::accumulate(initial_durations.begin(), initial_durations.end(), 0.0);
    if (std::abs(result.final_terms.total_time - initial_total) < 1e-4)
        return fail("joint optimizer did not change the free total time");
    if ((result.position_waypoints.row(0) - positions.row(0)).norm() > 1e-12 ||
        (result.position_waypoints.row(waypoint_count - 1) -
         positions.row(waypoint_count - 1)).norm() > 1e-12)
        return fail("joint optimizer moved a hover endpoint");
    for (int waypoint : free_waypoints)
    {
        const Eigen::Array3d normalized =
            (result.position_waypoints.row(waypoint) - positions.row(waypoint))
                .transpose()
                .array() /
            config.auxiliary_half_range.array();
        if ((normalized.abs() > 1.0 + 1e-10).any())
            return fail("joint optimizer moved a waypoint outside its search box");
    }

    planning::FlatOutputTrajectory trajectory;
    trajectory.update(result.durations, result.position_waypoints, yaws);
    if ((trajectory.sample(0.0).position - positions.row(0).transpose()).norm() > 1e-9 ||
        (trajectory.sample(trajectory.duration()).position -
         positions.row(waypoint_count - 1).transpose()).norm() > 1e-8)
        return fail("joint trajectory hover endpoint mismatch");

    planning::ActuatorPenaltyConfig no_velocity_config = config;
    no_velocity_config.enable_velocity_constraint = false;
    no_velocity_config.max_evaluations = 2;
    planning::ActuatorPenaltyOptimizer no_velocity_optimizer(
        planning::TailsitterFlatness(planning::VehicleParameters::paperVehicle()),
        no_velocity_config);
    const auto no_velocity_result = no_velocity_optimizer.optimizeJoint(
        initial_durations, positions, yaws, free_waypoints);
    if (std::abs(no_velocity_result.initial_terms.actuator.velocity) > 1e-15 ||
        !(no_velocity_result.initial_terms.actuator.collective_thrust > 0.0))
        return fail("disabling velocity also disabled or retained the wrong high-level term");

    auto low_speed_parameters = planning::VehicleParameters::paperVehicle();
    low_speed_parameters.velocity_max = 0.1;
    planning::FlatOutputTrajectory speed_only_trajectory;
    speed_only_trajectory.update(
        std::vector<double>(segment_count, 3.0), positions, zero_yaws);
    const auto speed_enforced = planning::FeasibilityChecker(
        planning::TailsitterFlatness(low_speed_parameters), 0.005, 1.0, true)
                                    .check(speed_only_trajectory);
    const auto speed_ignored = planning::FeasibilityChecker(
        planning::TailsitterFlatness(low_speed_parameters), 0.005, 1.0, false)
                                   .check(speed_only_trajectory);
    if (speed_enforced.feasible || !speed_ignored.feasible)
        return fail("feasibility checker velocity switch changed the wrong limits");

    std::cout << "joint optimizer test passed; objective "
              << result.initial_terms.total << " -> " << result.final_terms.total
              << ", total time " << initial_total << " -> "
              << result.final_terms.total_time << ", gradient error "
              << std::max(zero_check.maximum_relative_error,
                          pi_check.maximum_relative_error) << '\n';
    return 0;
}
