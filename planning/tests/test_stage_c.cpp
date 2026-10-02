#include "planning/actuator_penalty_optimizer.hpp"
#include "planning/feasibility_checker.hpp"

#include <algorithm>
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
    config.max_evaluations = 240;
    planning::ActuatorPenaltyOptimizer optimizer(
        planning::TailsitterFlatness(planning::VehicleParameters::paperVehicle()),
        config);

    // 1. The explicit constraint Jacobian must match centered finite
    //    differences of the constraint rows at the initial point. The 1e-2
    //    threshold accounts for the inherent knot-sample kink: yaw jerk is
    //    discontinuous at segment knots (C2 yaw spline), so dynamics rows
    //    have one-sided moving-sample derivatives there while centered
    //    differences average both sides; genuine assembly failures (zero or
    //    sign-flipped rows) show up as O(1) errors.
    const auto zero_check = optimizer.checkInitialJointConstraintGradient(
        initial_durations, positions, zero_yaws, free_waypoints);
    const auto pi_check = optimizer.checkInitialJointConstraintGradient(
        initial_durations, positions, yaws, free_waypoints);
    if (!zero_check.valid || !pi_check.valid ||
        zero_check.maximum_relative_error >= 1e-2 ||
        pi_check.maximum_relative_error >= 1e-2)
    {
        std::cerr << "constraint Jacobian errors: yaw0="
                  << zero_check.maximum_relative_error << ", yawpi="
                  << pi_check.maximum_relative_error << '\n';
        return fail("constraint Jacobian failed finite-difference validation");
    }

    // 2. The constrained optimizer must return a candidate that satisfies the
    //    sampled hard constraints, keeps endpoints and search box, and
    //    actually moves the free variables.
    const auto result = optimizer.optimizeJointConstrained(
        initial_durations, positions, yaws, free_waypoints);
    if (!result.optimized || result.objective_evaluations <= 0)
        return fail("constrained optimizer did not run");
    if (!result.constraints_satisfied)
    {
        double worst = -std::numeric_limits<double>::infinity();
        for (double value : result.final_constraint_values)
            worst = std::max(worst, value);
        std::cerr << "worst final constraint row: " << worst << '\n';
        return fail("constrained optimizer left constraint rows violated");
    }
    const double initial_total =
        std::accumulate(initial_durations.begin(), initial_durations.end(), 0.0);
    if (!(result.final_terms.total_time > 0.0) ||
        !std::isfinite(result.final_terms.total_time))
        return fail("constrained optimizer produced an invalid total time");
    if (std::abs(result.final_terms.total_time - initial_total) < 1e-4)
        return fail("constrained optimizer did not change the free total time");
    if ((result.position_waypoints.row(0) - positions.row(0)).norm() > 1e-12 ||
        (result.position_waypoints.row(waypoint_count - 1) -
         positions.row(waypoint_count - 1))
                .norm() > 1e-12)
        return fail("constrained optimizer moved a hover endpoint");
    for (int waypoint : free_waypoints)
    {
        const Eigen::Array3d normalized =
            (result.position_waypoints.row(waypoint) - positions.row(waypoint))
                .transpose()
                .array() /
            config.auxiliary_half_range.array();
        if ((normalized.abs() > 1.0 + 1e-10).any())
            return fail("constrained optimizer moved a waypoint outside its search box");
    }

    // The initial equal-time straight line must violate at least one sampled
    // planning envelope, and the returned candidate must not.
    const double initial_worst =
        *std::max_element(result.initial_constraint_values.begin(),
                          result.initial_constraint_values.end());
    if (!(initial_worst > 0.0))
        return fail("regression did not activate any constraint row");
    if (!std::isfinite(result.final_max_sample_violation) ||
        result.final_max_sample_violation > 1e-3)
    {
        std::cerr << "final max raw sample violation: "
                  << result.final_max_sample_violation << '\n';
        return fail("returned candidate still violates the planning envelope");
    }

    // 3. Rebuild the trajectory and confirm the endpoints, then certify with
    //    the independent dense checker at the full physical envelope.
    planning::FlatOutputTrajectory trajectory;
    trajectory.update(result.durations, result.position_waypoints, yaws);
    if ((trajectory.sample(0.0).position - positions.row(0).transpose()).norm() > 1e-9 ||
        (trajectory.sample(trajectory.duration()).position -
         positions.row(waypoint_count - 1).transpose())
                .norm() > 1e-8)
        return fail("constrained trajectory hover endpoint mismatch");
    const auto dense_report = planning::FeasibilityChecker(
        planning::TailsitterFlatness(
            planning::VehicleParameters::paperVehicle()),
        0.002,
        1.0,
        true)
                                  .check(trajectory);
    if (!dense_report.feasible)
    {
        std::cerr << "dense check failed at t="
                  << dense_report.first_failure_time << ": "
                  << dense_report.first_failure_reason << '\n';
        return fail("constrained trajectory failed the dense full-envelope check");
    }

    // 4. The warm-start path: a coarse soft-penalty preoptimization must
    //    decode into valid variables and lead the constrained solver to the
    //    same hard-constraint satisfaction as the cold start.
    planning::ActuatorPenaltyConfig coarse_config = config;
    coarse_config.samples_per_segment = 6;
    planning::ActuatorPenaltyOptimizer coarse_optimizer(
        planning::TailsitterFlatness(planning::VehicleParameters::paperVehicle()),
        coarse_config);
    const auto coarse = coarse_optimizer.optimizeJoint(
        initial_durations, positions, yaws, free_waypoints);
    const auto warm_result = optimizer.optimizeJointConstrained(
        initial_durations, positions, yaws, free_waypoints,
        coarse.durations, coarse.position_waypoints);
    if (!warm_result.optimized || !warm_result.constraints_satisfied)
        return fail("warm-started constrained optimizer did not satisfy constraints");
    if (warm_result.final_terms.total_time > 1.2 * initial_total ||
        warm_result.final_terms.total_time < 0.5 * initial_total)
        return fail("warm-started constrained optimizer produced an implausible total time");

    // 5. A solver switch must keep the same constraint semantics: with SLSQP
    //    as the AUGLAG subsolver the initial evaluation is unchanged.
    planning::ActuatorPenaltyConfig slsqp_config = config;
    slsqp_config.constrained_solver = planning::ConstrainedSolver::AuglagSlsqp;
    slsqp_config.max_evaluations = 2;
    planning::ActuatorPenaltyOptimizer slsqp_optimizer(
        planning::TailsitterFlatness(planning::VehicleParameters::paperVehicle()),
        slsqp_config);
    const auto slsqp_result = slsqp_optimizer.optimizeJointConstrained(
        initial_durations, positions, yaws, free_waypoints);
    if (std::abs(slsqp_result.initial_terms.total - result.initial_terms.total) > 1e-12)
        return fail("solver switch changed the initial objective evaluation");

    std::cout << "stage C constrained test passed; worst constraint row "
              << initial_worst << " -> "
              << *std::max_element(result.final_constraint_values.begin(),
                                   result.final_constraint_values.end())
              << ", total time " << initial_total << " -> "
              << result.final_terms.total_time
              << ", gradient error "
              << std::max(zero_check.maximum_relative_error,
                          pi_check.maximum_relative_error)
              << ", evaluations "
              << result.objective_evaluations << '\n';
    return 0;
}
