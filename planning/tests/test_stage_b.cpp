#include "planning/actuator_penalty_optimizer.hpp"
#include "planning/time_allocation.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>

namespace
{

int fail(const char* message)
{
    std::cerr << "FAIL: " << message << '\n';
    return 1;
}

double flatDerivativeComponent(const planning::FlatOutputSample& sample, int index)
{
    if (index < planning::ActuatorLinearization::acceleration_offset)
        return sample.velocity[index - planning::ActuatorLinearization::velocity_offset];
    if (index < planning::ActuatorLinearization::jerk_offset)
        return sample.acceleration[index - planning::ActuatorLinearization::acceleration_offset];
    if (index < planning::ActuatorLinearization::snap_offset)
        return sample.jerk[index - planning::ActuatorLinearization::jerk_offset];
    if (index < planning::ActuatorLinearization::yaw_index)
        return sample.snap[index - planning::ActuatorLinearization::snap_offset];
    if (index == planning::ActuatorLinearization::yaw_index)
        return sample.yaw;
    if (index == planning::ActuatorLinearization::yaw_rate_index)
        return sample.yaw_rate;
    return sample.yaw_acceleration;
}

void perturbFlatDerivative(planning::FlatOutputSample& sample, int index, double increment)
{
    if (index < planning::ActuatorLinearization::acceleration_offset)
        sample.velocity[index - planning::ActuatorLinearization::velocity_offset] += increment;
    else if (index < planning::ActuatorLinearization::jerk_offset)
        sample.acceleration[index - planning::ActuatorLinearization::acceleration_offset] += increment;
    else if (index < planning::ActuatorLinearization::snap_offset)
        sample.jerk[index - planning::ActuatorLinearization::jerk_offset] += increment;
    else if (index < planning::ActuatorLinearization::yaw_index)
        sample.snap[index - planning::ActuatorLinearization::snap_offset] += increment;
    else if (index == planning::ActuatorLinearization::yaw_index)
        sample.yaw += increment;
    else if (index == planning::ActuatorLinearization::yaw_rate_index)
        sample.yaw_rate += increment;
    else
        sample.yaw_acceleration += increment;
}

Eigen::Vector4d actuatorVector(const planning::ActuatorPrediction& prediction)
{
    return {prediction.thrust_1,
            prediction.thrust_2,
            prediction.flap_1,
            prediction.flap_2};
}

bool checkActuatorJacobian(const planning::FlatOutputTrajectory& trajectory,
                           const planning::TailsitterFlatness& flatness)
{
    planning::ActuatorLinearization linearization;
    planning::FlatOutputSample sample;
    for (int index = 1; index < 20; ++index)
    {
        sample = trajectory.sample(trajectory.duration() * static_cast<double>(index) / 20.0);
        if (sample.velocity.norm() < 0.25)
            continue;
        linearization = flatness.linearize(sample);
        if (linearization.differentiable)
            break;
    }
    if (!linearization.differentiable)
        return false;

    planning::BranchState fixed_branch;
    fixed_branch.initialized = true;
    fixed_branch.attitude = linearization.prediction.attitude;
    double maximum_relative_error = 0.0;
    double maximum_body_rate_relative_error = 0.0;
    for (int variable = 0;
         variable < planning::ActuatorLinearization::flat_derivative_dimension;
         ++variable)
    {
        const double value = flatDerivativeComponent(sample, variable);
        const double step = 2e-6 * std::max(1.0, std::abs(value));
        auto plus_sample = sample;
        auto minus_sample = sample;
        perturbFlatDerivative(plus_sample, variable, step);
        perturbFlatDerivative(minus_sample, variable, -step);
        const auto plus = flatness.forward(plus_sample, fixed_branch);
        const auto minus = flatness.forward(minus_sample, fixed_branch);
        if (!plus.valid || !minus.valid ||
            plus.roll_branch != linearization.prediction.roll_branch ||
            plus.pitch_branch != linearization.prediction.pitch_branch ||
            minus.roll_branch != linearization.prediction.roll_branch ||
            minus.pitch_branch != linearization.prediction.pitch_branch)
            return false;
        const Eigen::Vector4d finite_difference =
            (actuatorVector(plus) - actuatorVector(minus)) / (2.0 * step);
        const Eigen::Vector3d body_rate_finite_difference =
            (plus.angular_velocity - minus.angular_velocity) / (2.0 * step);
        for (int actuator = 0;
             actuator < planning::ActuatorLinearization::actuator_dimension;
             ++actuator)
        {
            const double analytic = linearization.jacobian(actuator, variable);
            const double numerical = finite_difference[actuator];
            const double scale = std::max({1.0, std::abs(analytic), std::abs(numerical)});
            maximum_relative_error = std::max(
                maximum_relative_error, std::abs(analytic - numerical) / scale);
        }
        for (int axis = 0; axis < 3; ++axis)
        {
            const double analytic =
                linearization.angular_velocity_jacobian(axis, variable);
            const double numerical = body_rate_finite_difference[axis];
            const double scale = std::max({1.0, std::abs(analytic), std::abs(numerical)});
            maximum_body_rate_relative_error = std::max(
                maximum_body_rate_relative_error,
                std::abs(analytic - numerical) / scale);
        }
    }
    if (!(maximum_relative_error < 2e-4))
    {
        std::cerr << "actuator Jacobian maximum relative error: "
                  << maximum_relative_error << '\n';
        return false;
    }
    if (!(maximum_body_rate_relative_error < 2e-4))
    {
        std::cerr << "body-rate Jacobian maximum relative error: "
                  << maximum_body_rate_relative_error << '\n';
        return false;
    }
    return linearization.jacobian.norm() > std::numeric_limits<double>::epsilon() &&
           linearization.angular_velocity_jacobian.norm() >
               std::numeric_limits<double>::epsilon();
}

}  // namespace

int main()
{
    constexpr double total_time = 1.8;
    planning::FlatOutputTrajectory::PositionMatrix positions(4, 3);
    positions << 0.0, 0.0, -10.0,
                 1.5, 0.0, -10.0,
                 3.75, 0.0, -10.0,
                 6.0, 0.0, -10.0;
    planning::FlatOutputTrajectory::YawMatrix yaws(4, 1);
    yaws << 0.0,
            0.5 * 3.14159265358979323846,
            0.75 * 3.14159265358979323846,
            3.14159265358979323846;

    const auto stage_a = planning::FixedTotalTimeAllocator(1.0).optimize(
        total_time, positions, yaws);
    planning::ActuatorPenaltyConfig config;
    config.samples_per_segment = 20;
    config.max_evaluations = 160;
    auto test_parameters = planning::VehicleParameters::paperVehicle();
    // Deliberately tighten the test-only envelopes so every newly added
    // penalty contributes a nonzero gradient to the end-to-end check.
    test_parameters.body_rate_max = 5.0;
    test_parameters.servo_time_constant = 0.2;
    test_parameters.servo_rate_max = 2.0;
    planning::ActuatorPenaltyOptimizer optimizer(
        planning::TailsitterFlatness(test_parameters), config);
    const auto objective_gradient_check = optimizer.checkInitialGradient(
        total_time, stage_a.durations, positions, yaws, {2});
    const auto validate_objective_gradient = [](const auto& check, const char* label) {
        if (check.valid && check.maximum_relative_error < 5e-4)
            return true;
        std::cerr << label << " objective gradient maximum relative error: "
                  << check.maximum_relative_error << '\n';
        for (std::size_t i = 0; i < check.analytic.size(); ++i)
        {
            std::cerr << "  variable " << i << ": analytic="
                      << check.analytic[i] << ", finite-difference="
                      << check.finite_difference[i] << '\n';
        }
        return false;
    };
    if (!validate_objective_gradient(objective_gradient_check, "yaw 0 -> pi"))
        return fail("stage B objective gradient failed finite-difference validation");

    planning::FlatOutputTrajectory::YawMatrix zero_yaws =
        planning::FlatOutputTrajectory::YawMatrix::Zero(4, 1);
    const auto zero_yaw_stage_a = planning::FixedTotalTimeAllocator(1.0).optimize(
        total_time, positions, zero_yaws);
    const auto zero_yaw_gradient_check = optimizer.checkInitialGradient(
        total_time, zero_yaw_stage_a.durations, positions, zero_yaws, {2});
    if (!validate_objective_gradient(zero_yaw_gradient_check, "yaw 0 -> 0"))
    {
        return fail("stage B objective gradient failed finite-difference validation");
    }
    const auto result = optimizer.optimize(total_time, stage_a.durations, positions, yaws, {2});

    if (!result.optimized || result.evaluations <= 0)
        return fail("stage B optimizer did not run");
    const double duration_sum =
        std::accumulate(result.durations.begin(), result.durations.end(), 0.0);
    if (std::abs(duration_sum - total_time) > 1e-12)
        return fail("stage B changed the fixed total time");
    if ((result.position_waypoints.row(0) - positions.row(0)).norm() > 1e-12 ||
        (result.position_waypoints.row(1) - positions.row(1)).norm() > 1e-12 ||
        (result.position_waypoints.row(3) - positions.row(3)).norm() > 1e-12)
        return fail("stage B moved a fixed key waypoint");
    if (result.final_terms.total > result.initial_terms.total * (1.0 + 1e-8))
        return fail("stage B increased its combined objective");
    if (result.final_terms.actuator.total() >= result.initial_terms.actuator.total())
        return fail("stage B did not reduce the sampled actuator penalty");
    if (!std::isfinite(result.final_terms.actuator.total()) ||
        !std::isfinite(result.final_terms.smoothness))
        return fail("stage B objective contains a non-finite term");
    if (!(result.initial_terms.actuator.body_rate > 0.0 &&
          result.initial_terms.actuator.motor_dynamics > 0.0 &&
          result.initial_terms.actuator.flap_rate > 0.0 &&
          result.initial_terms.actuator.servo_dynamics > 0.0))
    {
        return fail("stage B regression did not activate all rate/dynamic penalties");
    }

    planning::FlatOutputTrajectory trajectory;
    trajectory.update(result.durations, result.position_waypoints, yaws);
    if ((trajectory.sample(0.0).position - positions.row(0).transpose()).norm() > 1e-9 ||
        (trajectory.sample(total_time).position - positions.row(3).transpose()).norm() > 1e-8)
        return fail("stage B trajectory endpoint mismatch");
    if (!checkActuatorJacobian(
            trajectory, planning::TailsitterFlatness(test_parameters)))
        return fail("fixed-branch actuator Jacobian failed finite-difference validation");

    std::cout << "stage B test passed; objective " << result.initial_terms.total << " -> "
              << result.final_terms.total << ", actuator integral "
              << result.initial_terms.actuator.total() << " -> "
              << result.final_terms.actuator.total() << ", objective gradient error "
              << std::max(objective_gradient_check.maximum_relative_error,
                          zero_yaw_gradient_check.maximum_relative_error) << '\n';
    return 0;
}
