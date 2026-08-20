#include "planning/actuator_penalty_optimizer.hpp"

#include "planning/time_allocation.hpp"

#include <nlopt.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>

namespace planning
{
namespace
{

struct PenaltyValueDerivative
{
    double value{0.0};
    double derivative{0.0};
};

PenaltyValueDerivative squaredPositivePartWithDerivative(double value)
{
    if (!(value > 0.0))
        return {};
    return {value * value, 2.0 * value};
}

PenaltyValueDerivative boundedSquaredPenalty(double value,
                                             double lower_bound,
                                             double upper_bound,
                                             double scale)
{
    PenaltyValueDerivative result;
    if (!std::isfinite(value))
        return result;
    const auto lower = squaredPositivePartWithDerivative(
        (lower_bound - value) / scale);
    const auto upper = squaredPositivePartWithDerivative(
        (value - upper_bound) / scale);
    result.value = lower.value + upper.value;
    result.derivative = (-lower.derivative + upper.derivative) / scale;
    return result;
}

PenaltyValueDerivative symmetricSquaredPenalty(double value, double magnitude_bound)
{
    PenaltyValueDerivative result;
    if (!std::isfinite(value))
        return result;
    const auto magnitude = squaredPositivePartWithDerivative(
        (std::abs(value) - magnitude_bound) / magnitude_bound);
    result.value = magnitude.value;
    if (magnitude.derivative != 0.0)
    {
        result.derivative = magnitude.derivative *
                            (value >= 0.0 ? 1.0 : -1.0) /
                            magnitude_bound;
    }
    return result;
}

struct ObjectiveContext
{
    double total_time;
    double smoothness_reference;
    bool fixed_total_time;
    const std::vector<double>* initial_durations;
    const FlatOutputTrajectory::PositionMatrix* nominal_positions;
    const FlatOutputTrajectory::YawMatrix* yaws;
    const std::vector<int>* auxiliary_indices;
    const TailsitterFlatness* flatness;
    const ActuatorPenaltyConfig* config;
    int evaluations{0};
};

std::size_t durationVariableCount(const ObjectiveContext& context)
{
    return context.fixed_total_time
        ? static_cast<std::size_t>(context.nominal_positions->rows() - 2)
        : context.initial_durations->size();
}

FlatOutputTrajectory::PositionMatrix positionsFromVariables(
    const std::vector<double>& variables,
    const ObjectiveContext& context)
{
    auto positions = *context.nominal_positions;
    const std::size_t duration_variable_count = durationVariableCount(context);
    for (std::size_t auxiliary = 0; auxiliary < context.auxiliary_indices->size(); ++auxiliary)
    {
        const Eigen::Index row = (*context.auxiliary_indices)[auxiliary];
        for (int axis = 0; axis < 3; ++axis)
        {
            const std::size_t variable = duration_variable_count + 3 * auxiliary +
                                         static_cast<std::size_t>(axis);
            positions(row, axis) +=
                context.config->auxiliary_half_range[axis] * variables[variable];
        }
    }
    return positions;
}

std::vector<double> durationsFromVariables(const std::vector<double>& variables,
                                           const ObjectiveContext& context)
{
    const std::size_t count = durationVariableCount(context);
    if (context.fixed_total_time)
    {
        return FixedTotalTimeAllocator::durationsFromTau(
            context.total_time,
            std::vector<double>(variables.begin(), variables.begin() + count));
    }
    std::vector<double> durations(count);
    for (std::size_t segment = 0; segment < count; ++segment)
    {
        durations[segment] = (*context.initial_durations)[segment] *
                             std::exp(variables[segment]);
    }
    return durations;
}

struct ActuatorPenaltyEvaluation
{
    ActuatorPenaltyIntegral integral;
    FlatOutputTrajectory::PositionMatrix partial_position_coefficients;
    FlatOutputTrajectory::YawMatrix partial_yaw_coefficients;
    Eigen::VectorXd partial_position_times;
    Eigen::VectorXd partial_yaw_times;
};

using FlatGradient = Eigen::Matrix<
    double, ActuatorLinearization::flat_derivative_dimension, 1>;
using ActuatorStateJacobian = Eigen::Matrix<
    double,
    ActuatorLinearization::actuator_dimension,
    ActuatorLinearization::flat_derivative_dimension>;

struct PenaltySample
{
    double local_fraction{0.0};
    double local_time{0.0};
    FlatOutputSample sample;
    ActuatorPrediction prediction;
    ActuatorLinearization linearization;
};

struct DynamicActuatorState
{
    Eigen::Vector4d value{Eigen::Vector4d::Zero()};
    ActuatorStateJacobian jacobian{ActuatorStateJacobian::Zero()};
    bool valid{false};
    bool differentiable{false};
};

DynamicActuatorState dynamicActuatorState(const PenaltySample& sample,
                                          const VehicleParameters& parameters,
                                          bool compute_gradient)
{
    DynamicActuatorState state;
    state.value << sample.prediction.rotor_speed_1,
                   sample.prediction.rotor_speed_2,
                   sample.prediction.flap_1,
                   sample.prediction.flap_2;
    state.valid = sample.prediction.valid && state.value.allFinite();
    if (!state.valid || !compute_gradient || !sample.linearization.differentiable)
        return state;

    for (int motor = 0; motor < 2; ++motor)
    {
        const double rotor_speed = state.value[motor];
        if (!(rotor_speed > 1e-9))
            return state;
        state.jacobian.row(motor) =
            sample.linearization.jacobian.row(motor) /
            (2.0 * parameters.c_T * rotor_speed);
    }
    state.jacobian.bottomRows<2>() = sample.linearization.jacobian.bottomRows<2>();
    state.differentiable = state.jacobian.allFinite();
    return state;
}

void accumulateFlatGradient(ActuatorPenaltyEvaluation& evaluation,
                            Eigen::Index segment_index,
                            const PenaltySample& sample,
                            const FlatGradient& contribution_gradient)
{
    Eigen::Matrix<double, 1, FlatOutputTrajectory::PositionSpline::COEFF_NUM>
        position_basis;
    Eigen::Matrix<double, 1, FlatOutputTrajectory::PositionSpline::COEFF_NUM>
        velocity_basis;
    Eigen::Matrix<double, 1, FlatOutputTrajectory::PositionSpline::COEFF_NUM>
        acceleration_basis;
    Eigen::Matrix<double, 1, FlatOutputTrajectory::PositionSpline::COEFF_NUM>
        jerk_basis;
    Eigen::Matrix<double, 1, FlatOutputTrajectory::PositionSpline::COEFF_NUM>
        snap_basis;
    Eigen::Matrix<double, 1, FlatOutputTrajectory::PositionSpline::COEFF_NUM>
        crackle_basis;
    FlatOutputTrajectory::PositionSpline::computeBasisFunctions(
        sample.local_time,
        position_basis,
        velocity_basis,
        acceleration_basis,
        jerk_basis,
        snap_basis,
        crackle_basis);
    auto position_block = evaluation.partial_position_coefficients.middleRows(
        segment_index * FlatOutputTrajectory::PositionSpline::COEFF_NUM,
        FlatOutputTrajectory::PositionSpline::COEFF_NUM);
    position_block.noalias() +=
        velocity_basis.transpose() *
            contribution_gradient.segment<3>(ActuatorLinearization::velocity_offset).transpose() +
        acceleration_basis.transpose() *
            contribution_gradient.segment<3>(ActuatorLinearization::acceleration_offset).transpose() +
        jerk_basis.transpose() *
            contribution_gradient.segment<3>(ActuatorLinearization::jerk_offset).transpose() +
        snap_basis.transpose() *
            contribution_gradient.segment<3>(ActuatorLinearization::snap_offset).transpose();

    Eigen::Matrix<double, 1, FlatOutputTrajectory::YawSpline::COEFF_NUM> yaw_basis;
    Eigen::Matrix<double, 1, FlatOutputTrajectory::YawSpline::COEFF_NUM> yaw_rate_basis;
    Eigen::Matrix<double, 1, FlatOutputTrajectory::YawSpline::COEFF_NUM>
        yaw_acceleration_basis;
    Eigen::Matrix<double, 1, FlatOutputTrajectory::YawSpline::COEFF_NUM> yaw_jerk_basis;
    Eigen::Matrix<double, 1, FlatOutputTrajectory::YawSpline::COEFF_NUM> yaw_snap_basis;
    Eigen::Matrix<double, 1, FlatOutputTrajectory::YawSpline::COEFF_NUM> yaw_crackle_basis;
    FlatOutputTrajectory::YawSpline::computeBasisFunctions(
        sample.local_time,
        yaw_basis,
        yaw_rate_basis,
        yaw_acceleration_basis,
        yaw_jerk_basis,
        yaw_snap_basis,
        yaw_crackle_basis);
    auto yaw_block = evaluation.partial_yaw_coefficients.middleRows(
        segment_index * FlatOutputTrajectory::YawSpline::COEFF_NUM,
        FlatOutputTrajectory::YawSpline::COEFF_NUM);
    yaw_block.noalias() +=
        yaw_basis.transpose() * contribution_gradient[ActuatorLinearization::yaw_index] +
        yaw_rate_basis.transpose() *
            contribution_gradient[ActuatorLinearization::yaw_rate_index] +
        yaw_acceleration_basis.transpose() *
            contribution_gradient[ActuatorLinearization::yaw_acceleration_index];

    FlatGradient flat_time_derivative;
    flat_time_derivative << sample.sample.acceleration,
                            sample.sample.jerk,
                            sample.sample.snap,
                            sample.sample.crackle,
                            sample.sample.yaw_rate,
                            sample.sample.yaw_acceleration,
                            sample.sample.yaw_jerk;
    evaluation.partial_position_times[segment_index] +=
        sample.local_fraction * contribution_gradient.dot(flat_time_derivative);
}

ActuatorPenaltyEvaluation integrateActuatorPenalty(
    FlatOutputTrajectory& trajectory,
    const TailsitterFlatness& flatness,
    const ActuatorPenaltyConfig& config,
    bool compute_gradient)
{
    const auto& parameters = flatness.parameters();
    if (!(parameters.body_rate_max > 0.0))
        throw std::invalid_argument("body-rate envelope must be positive");
    if (config.enable_high_level_constraints && !(parameters.velocity_max > 0.0))
        throw std::invalid_argument("velocity envelope must be positive");
    if (config.enable_actuator_dynamics &&
        (!(parameters.c_T > 0.0) || !(parameters.motor_time_constant > 0.0) ||
         !(parameters.servo_time_constant > 0.0) ||
         !(parameters.servo_rate_max > 0.0)))
    {
        throw std::invalid_argument("actuator dynamics parameters must be positive");
    }
    const double thrust_min = parameters.c_T * parameters.omega_min * parameters.omega_min;
    const double planning_omega_max = config.actuator_margin * parameters.omega_max;
    const double thrust_max = parameters.c_T * planning_omega_max * planning_omega_max;
    const double thrust_scale = std::max(1e-9, thrust_max - thrust_min);
    const double flap_max = config.actuator_margin * parameters.delta_max;
    const double body_rate_max = config.actuator_margin * parameters.body_rate_max;
    const double velocity_max = config.actuator_margin * parameters.velocity_max;
    const double collective_thrust_min = 2.0 * thrust_min;
    const double collective_thrust_max = 2.0 * thrust_max;
    const double collective_thrust_scale =
        std::max(1e-9, collective_thrust_max - collective_thrust_min);

    const Eigen::Index segment_count =
        static_cast<Eigen::Index>(trajectory.durations().size());
    ActuatorPenaltyEvaluation evaluation;
    evaluation.partial_position_coefficients =
        FlatOutputTrajectory::PositionMatrix::Zero(
            segment_count * FlatOutputTrajectory::PositionSpline::COEFF_NUM, 3);
    evaluation.partial_yaw_coefficients =
        FlatOutputTrajectory::YawMatrix::Zero(
            segment_count * FlatOutputTrajectory::YawSpline::COEFF_NUM, 1);
    evaluation.partial_position_times = Eigen::VectorXd::Zero(segment_count);
    evaluation.partial_yaw_times = Eigen::VectorXd::Zero(segment_count);

    BranchState branch;
    double segment_start = 0.0;
    for (Eigen::Index segment_index = 0; segment_index < segment_count; ++segment_index)
    {
        const double duration = trajectory.durations()[static_cast<std::size_t>(segment_index)];
        const double step = duration / config.samples_per_segment;
        std::vector<PenaltySample> segment_samples;
        segment_samples.reserve(static_cast<std::size_t>(config.samples_per_segment + 1));
        for (int sample_index = 0; sample_index <= config.samples_per_segment; ++sample_index)
        {
            const double trap_weight =
                (sample_index == 0 || sample_index == config.samples_per_segment) ? 0.5 : 1.0;
            const double quadrature_weight = step * trap_weight;
            const double local_fraction =
                static_cast<double>(sample_index) / config.samples_per_segment;
            const double local_time = local_fraction * duration;
            const double time = segment_start + local_time;
            const FlatOutputSample sample = trajectory.sample(time);
            ActuatorLinearization linearization;
            ActuatorPrediction prediction;
            if (compute_gradient)
            {
                linearization = flatness.linearize(sample, branch);
                prediction = linearization.prediction;
            }
            else
            {
                prediction = flatness.forward(sample, branch);
            }

            if (prediction.attitude.coeffs().allFinite() && prediction.attitude.norm() > 0.5)
            {
                branch.initialized = true;
                branch.attitude = prediction.attitude;
            }
            segment_samples.push_back(
                {local_fraction, local_time, sample, prediction, linearization});

            const auto thrust_penalty = [&](double thrust) {
                PenaltyValueDerivative result;
                if (!std::isfinite(thrust))
                {
                    result.value = config.invalid_mapping_penalty;
                    return result;
                }
                const auto lower = squaredPositivePartWithDerivative(
                    (thrust_min - thrust) / thrust_scale);
                const auto upper = squaredPositivePartWithDerivative(
                    (thrust - thrust_max) / thrust_scale);
                result.value = lower.value + upper.value;
                result.derivative =
                    (-lower.derivative + upper.derivative) / thrust_scale;
                return result;
            };
            const auto thrust_1_penalty = thrust_penalty(prediction.thrust_1);
            const auto thrust_2_penalty = thrust_penalty(prediction.thrust_2);
            evaluation.integral.thrust_1 += quadrature_weight * thrust_1_penalty.value;
            evaluation.integral.thrust_2 += quadrature_weight * thrust_2_penalty.value;

            const auto flap_penalty = [&](double flap) {
                PenaltyValueDerivative result;
                if (!std::isfinite(flap))
                {
                    result.value = config.invalid_mapping_penalty;
                    return result;
                }
                const auto magnitude = squaredPositivePartWithDerivative(
                    (std::abs(flap) - flap_max) / flap_max);
                result.value = magnitude.value;
                if (magnitude.derivative != 0.0)
                    result.derivative = magnitude.derivative *
                                        (flap >= 0.0 ? 1.0 : -1.0) / flap_max;
                return result;
            };
            const auto flap_1_penalty = flap_penalty(prediction.flap_1);
            const auto flap_2_penalty = flap_penalty(prediction.flap_2);
            evaluation.integral.flap_1 += quadrature_weight * flap_1_penalty.value;
            evaluation.integral.flap_2 += quadrature_weight * flap_2_penalty.value;

            PenaltyValueDerivative body_rate_penalty;
            Eigen::Vector3d body_rate_penalty_gradient = Eigen::Vector3d::Zero();
            if (config.enable_body_rate_constraint)
            {
                const double body_rate = prediction.angular_velocity.norm();
                if (!std::isfinite(body_rate))
                {
                    body_rate_penalty.value = config.invalid_mapping_penalty;
                }
                else
                {
                    body_rate_penalty = squaredPositivePartWithDerivative(
                        (body_rate - body_rate_max) / body_rate_max);
                    if (body_rate_penalty.derivative != 0.0 && body_rate > 1e-12)
                    {
                        body_rate_penalty_gradient =
                            body_rate_penalty.derivative / body_rate_max *
                            prediction.angular_velocity / body_rate;
                    }
                }
                evaluation.integral.body_rate +=
                    quadrature_weight * body_rate_penalty.value;
            }

            PenaltyValueDerivative velocity_penalty;
            Eigen::Vector3d velocity_penalty_gradient = Eigen::Vector3d::Zero();
            PenaltyValueDerivative collective_thrust_penalty;
            if (config.enable_high_level_constraints)
            {
                const double speed = sample.velocity.norm();
                velocity_penalty = squaredPositivePartWithDerivative(
                    (speed - velocity_max) / velocity_max);
                if (velocity_penalty.derivative != 0.0 && speed > 1e-12)
                {
                    velocity_penalty_gradient =
                        velocity_penalty.derivative / velocity_max *
                        sample.velocity / speed;
                }
                collective_thrust_penalty = boundedSquaredPenalty(
                    prediction.collective_thrust,
                    collective_thrust_min,
                    collective_thrust_max,
                    collective_thrust_scale);
                evaluation.integral.velocity +=
                    quadrature_weight * velocity_penalty.value;
                evaluation.integral.collective_thrust +=
                    quadrature_weight * collective_thrust_penalty.value;
            }
            double invalid_penalty = 0.0;
            if (!prediction.valid)
            {
                invalid_penalty = config.invalid_mapping_penalty;
                evaluation.integral.invalid_mapping += quadrature_weight * invalid_penalty;
            }

            if (!compute_gradient)
                continue;
            if (!linearization.differentiable)
            {
                evaluation.partial_position_times[segment_index] +=
                    trap_weight / config.samples_per_segment *
                    (thrust_1_penalty.value + thrust_2_penalty.value +
                     flap_1_penalty.value + flap_2_penalty.value +
                     body_rate_penalty.value + velocity_penalty.value +
                     collective_thrust_penalty.value + invalid_penalty);
                continue;
            }

            const Eigen::Vector4d actuator_penalty_gradient(
                thrust_1_penalty.derivative + collective_thrust_penalty.derivative,
                thrust_2_penalty.derivative + collective_thrust_penalty.derivative,
                flap_1_penalty.derivative,
                flap_2_penalty.derivative);
            FlatGradient flat_gradient =
                linearization.jacobian.transpose() * actuator_penalty_gradient;
            flat_gradient.noalias() +=
                linearization.angular_velocity_jacobian.transpose() *
                body_rate_penalty_gradient;
            flat_gradient.segment<3>(ActuatorLinearization::velocity_offset) +=
                velocity_penalty_gradient;

            const double sample_penalty =
                thrust_1_penalty.value + thrust_2_penalty.value +
                flap_1_penalty.value + flap_2_penalty.value +
                body_rate_penalty.value + velocity_penalty.value +
                collective_thrust_penalty.value + invalid_penalty;
            evaluation.partial_position_times[segment_index] +=
                trap_weight / config.samples_per_segment * sample_penalty;
            accumulateFlatGradient(evaluation,
                                   segment_index,
                                   segment_samples.back(),
                                   quadrature_weight * flat_gradient);
        }

        if (config.enable_actuator_dynamics)
        {
            const double rotor_command_min = parameters.omega_min;
            const double rotor_command_max = planning_omega_max;
            const double rotor_command_scale =
                std::max(1e-9, rotor_command_max - rotor_command_min);
            const double flap_rate_max =
                config.actuator_margin * parameters.servo_rate_max;
            for (int interval = 0; interval < config.samples_per_segment; ++interval)
            {
                const auto& first = segment_samples[static_cast<std::size_t>(interval)];
                const auto& second = segment_samples[static_cast<std::size_t>(interval + 1)];
                const auto first_state =
                    dynamicActuatorState(first, parameters, compute_gradient);
                const auto second_state =
                    dynamicActuatorState(second, parameters, compute_gradient);
                if (!first_state.valid || !second_state.valid)
                    continue;

                Eigen::Vector4d first_cost_gradient = Eigen::Vector4d::Zero();
                Eigen::Vector4d second_cost_gradient = Eigen::Vector4d::Zero();
                double explicit_duration_gradient = 0.0;

                const auto add_first_order_dynamics =
                    [&](int actuator,
                        double time_constant,
                        bool symmetric_bound,
                        double lower_bound,
                        double upper_bound,
                        double& integral) {
                        const double decay = std::exp(-step / time_constant);
                        const double response = 1.0 - decay;
                        const double command =
                            (second_state.value[actuator] -
                             decay * first_state.value[actuator]) /
                            response;
                        const auto penalty = symmetric_bound
                            ? symmetricSquaredPenalty(command, upper_bound)
                            : boundedSquaredPenalty(command,
                                                    lower_bound,
                                                    upper_bound,
                                                    rotor_command_scale);
                        integral += step * penalty.value;
                        const double command_first = -decay / response;
                        const double command_second = 1.0 / response;
                        first_cost_gradient[actuator] +=
                            step * penalty.derivative * command_first;
                        second_cost_gradient[actuator] +=
                            step * penalty.derivative * command_second;
                        const double command_step =
                            -decay / time_constant *
                            (second_state.value[actuator] - first_state.value[actuator]) /
                            (response * response);
                        explicit_duration_gradient +=
                            (penalty.value +
                             step * penalty.derivative * command_step) /
                            config.samples_per_segment;
                    };

                add_first_order_dynamics(0,
                                         parameters.motor_time_constant,
                                         false,
                                         rotor_command_min,
                                         rotor_command_max,
                                         evaluation.integral.motor_dynamics);
                add_first_order_dynamics(1,
                                         parameters.motor_time_constant,
                                         false,
                                         rotor_command_min,
                                         rotor_command_max,
                                         evaluation.integral.motor_dynamics);
                add_first_order_dynamics(2,
                                         parameters.servo_time_constant,
                                         true,
                                         -flap_max,
                                         flap_max,
                                         evaluation.integral.servo_dynamics);
                add_first_order_dynamics(3,
                                         parameters.servo_time_constant,
                                         true,
                                         -flap_max,
                                         flap_max,
                                         evaluation.integral.servo_dynamics);

                for (int flap = 2; flap < 4; ++flap)
                {
                    const double rate =
                        (second_state.value[flap] - first_state.value[flap]) / step;
                    const auto penalty = symmetricSquaredPenalty(rate, flap_rate_max);
                    evaluation.integral.flap_rate += step * penalty.value;
                    first_cost_gradient[flap] -= penalty.derivative;
                    second_cost_gradient[flap] += penalty.derivative;
                    explicit_duration_gradient +=
                        (penalty.value - penalty.derivative * rate) /
                        config.samples_per_segment;
                }
                evaluation.partial_position_times[segment_index] +=
                    explicit_duration_gradient;

                if (!compute_gradient || !first_state.differentiable ||
                    !second_state.differentiable)
                    continue;
                accumulateFlatGradient(
                    evaluation,
                    segment_index,
                    first,
                    first_state.jacobian.transpose() * first_cost_gradient);
                accumulateFlatGradient(
                    evaluation,
                    segment_index,
                    second,
                    second_state.jacobian.transpose() * second_cost_gradient);
            }
        }
        segment_start += duration;
    }
    return evaluation;
}

StageBObjectiveTerms evaluate(const std::vector<double>& variables,
                              const ObjectiveContext& context,
                              std::vector<double>* objective_gradient = nullptr)
{
    StageBObjectiveTerms terms;
    if (objective_gradient != nullptr)
        objective_gradient->assign(variables.size(), 0.0);
    try
    {
        const auto durations = durationsFromVariables(variables, context);
        const auto positions = positionsFromVariables(variables, context);
        FlatOutputTrajectory trajectory;
        trajectory.update(durations, positions, *context.yaws);
        terms.smoothness = trajectory.energy(context.config->yaw_weight);
        terms.normalized_smoothness = terms.smoothness / context.smoothness_reference;
        terms.total_time = std::accumulate(durations.begin(), durations.end(), 0.0);
        terms.time = context.fixed_total_time
            ? 0.0
            : context.config->time_weight * terms.total_time;
        const std::size_t duration_variable_count = durationVariableCount(context);
        for (std::size_t i = duration_variable_count; i < variables.size(); ++i)
            terms.geometry += variables[i] * variables[i];
        const auto actuator_evaluation = integrateActuatorPenalty(
            trajectory,
            *context.flatness,
            *context.config,
            objective_gradient != nullptr);
        terms.actuator = actuator_evaluation.integral;
        terms.total = context.config->smoothness_weight * terms.normalized_smoothness +
                      context.config->geometry_weight * terms.geometry +
                      context.config->actuator_weight * terms.actuator.total() +
                      terms.time;
        if (objective_gradient != nullptr)
        {
            const auto smoothness_gradient =
                trajectory.energyGradient(context.config->yaw_weight);
            const auto actuator_gradient = trajectory.propagateCostGradient(
                actuator_evaluation.partial_position_coefficients,
                actuator_evaluation.partial_position_times,
                actuator_evaluation.partial_yaw_coefficients,
                actuator_evaluation.partial_yaw_times);
            Eigen::VectorXd duration_gradient =
                context.config->smoothness_weight / context.smoothness_reference *
                    smoothness_gradient.times +
                context.config->actuator_weight * actuator_gradient.times;
            if (!context.fixed_total_time)
                duration_gradient.array() += context.config->time_weight;

            if (context.fixed_total_time)
            {
                double weighted_mean = 0.0;
                for (Eigen::Index segment = 0;
                     segment < static_cast<Eigen::Index>(durations.size());
                     ++segment)
                {
                    weighted_mean += durations[static_cast<std::size_t>(segment)] *
                                     duration_gradient[segment];
                }
                weighted_mean /= context.total_time;
                for (std::size_t variable = 0;
                     variable < duration_variable_count;
                     ++variable)
                {
                    (*objective_gradient)[variable] = durations[variable] *
                        (duration_gradient[static_cast<Eigen::Index>(variable)] -
                         weighted_mean);
                }
            }
            else
            {
                for (std::size_t variable = 0;
                     variable < duration_variable_count;
                     ++variable)
                {
                    (*objective_gradient)[variable] = durations[variable] *
                        duration_gradient[static_cast<Eigen::Index>(variable)];
                }
            }

            const auto position_inner_gradient =
                context.config->smoothness_weight / context.smoothness_reference *
                    smoothness_gradient.position_inner_points +
                context.config->actuator_weight *
                    actuator_gradient.position_inner_points;
            for (std::size_t auxiliary = 0;
                 auxiliary < context.auxiliary_indices->size();
                 ++auxiliary)
            {
                const Eigen::Index waypoint = (*context.auxiliary_indices)[auxiliary];
                for (int axis = 0; axis < 3; ++axis)
                {
                    const std::size_t variable = duration_variable_count + 3 * auxiliary +
                                                 static_cast<std::size_t>(axis);
                    (*objective_gradient)[variable] =
                        context.config->auxiliary_half_range[axis] *
                            position_inner_gradient(waypoint - 1, axis) +
                        2.0 * context.config->geometry_weight * variables[variable];
                }
            }
        }
        const bool gradient_finite = objective_gradient == nullptr ||
            std::all_of(objective_gradient->begin(), objective_gradient->end(),
                        [](double value) { return std::isfinite(value); });
        if (!std::isfinite(terms.total) || !gradient_finite)
        {
            terms.total = 1e30;
            if (objective_gradient != nullptr)
                std::fill(objective_gradient->begin(), objective_gradient->end(), 0.0);
        }
    }
    catch (const std::exception&)
    {
        terms.total = 1e30;
        if (objective_gradient != nullptr)
            std::fill(objective_gradient->begin(), objective_gradient->end(), 0.0);
    }
    return terms;
}

double objective(const std::vector<double>& variables,
                 std::vector<double>& gradient,
                 void* raw_context)
{
    auto& context = *static_cast<ObjectiveContext*>(raw_context);
    ++context.evaluations;
    return evaluate(variables, context, gradient.empty() ? nullptr : &gradient).total;
}

void validateInputs(double total_time,
                    const std::vector<double>& initial_durations,
                    const FlatOutputTrajectory::PositionMatrix& positions,
                    const FlatOutputTrajectory::YawMatrix& yaws,
                    const std::vector<int>& auxiliary_indices,
                    const ActuatorPenaltyConfig& config,
                    bool fixed_total_time)
{
    if (positions.rows() < 2 ||
        positions.rows() != yaws.rows() ||
        initial_durations.size() != static_cast<std::size_t>(positions.rows() - 1))
        throw std::invalid_argument("invalid stage B trajectory dimensions");
    if (std::any_of(initial_durations.begin(), initial_durations.end(),
                    [](double value) {
                        return !(value > 0.0) || !std::isfinite(value);
                    }))
        throw std::invalid_argument("initial segment durations must be finite and positive");
    const double duration_sum =
        std::accumulate(initial_durations.begin(), initial_durations.end(), 0.0);
    if (fixed_total_time &&
        (!(total_time > 0.0) ||
         std::abs(duration_sum - total_time) >
             1e-9 * std::max(1.0, total_time)))
        throw std::invalid_argument("stage B initial durations must sum to total_time");
    if (!(config.yaw_weight >= 0.0 && config.smoothness_weight >= 0.0 &&
          config.actuator_weight >= 0.0 && config.geometry_weight >= 0.0 &&
          config.actuator_margin > 0.0 && config.actuator_margin <= 1.0 &&
          config.samples_per_segment > 0 && config.invalid_mapping_penalty > 0.0 &&
          config.tau_bound > 0.0 && config.max_evaluations > 0 &&
          config.free_time_log_bound > 0.0 && config.time_weight >= 0.0 &&
          config.relative_tolerance > 0.0) ||
        !(config.auxiliary_half_range.array() >= 0.0).all())
        throw std::invalid_argument("invalid stage B optimizer configuration");

    std::vector<bool> used(static_cast<std::size_t>(positions.rows()), false);
    for (int index : auxiliary_indices)
    {
        if (index <= 0 || index + 1 >= positions.rows() || used[static_cast<std::size_t>(index)])
            throw std::invalid_argument("auxiliary waypoints must be unique interior points");
        used[static_cast<std::size_t>(index)] = true;
    }
}

}  // namespace

ActuatorPenaltyOptimizer::ActuatorPenaltyOptimizer(TailsitterFlatness flatness,
                                                   ActuatorPenaltyConfig config)
    : flatness_(std::move(flatness)), config_(std::move(config))
{
}

StageBOptimizationResult ActuatorPenaltyOptimizer::optimize(
    double total_time,
    const std::vector<double>& initial_durations,
    const FlatOutputTrajectory::PositionMatrix& nominal_position_waypoints,
    const FlatOutputTrajectory::YawMatrix& yaw_waypoints,
    const std::vector<int>& auxiliary_waypoint_indices) const
{
    validateInputs(total_time,
                   initial_durations,
                   nominal_position_waypoints,
                   yaw_waypoints,
                   auxiliary_waypoint_indices,
                   config_,
                   true);

    FlatOutputTrajectory reference;
    reference.update(initial_durations, nominal_position_waypoints, yaw_waypoints);
    const double smoothness_reference =
        std::max(1e-12, reference.energy(config_.yaw_weight));
    ObjectiveContext context{total_time,
                             smoothness_reference,
                             true,
                             &initial_durations,
                             &nominal_position_waypoints,
                             &yaw_waypoints,
                             &auxiliary_waypoint_indices,
                             &flatness_,
                             &config_};

    const std::size_t tau_count = durationVariableCount(context);
    std::vector<double> variables(tau_count + 3 * auxiliary_waypoint_indices.size(), 0.0);
    const double last_duration = initial_durations.back();
    for (std::size_t i = 0; i < tau_count; ++i)
        variables[i] = std::log(initial_durations[i] / last_duration);

    std::vector<double> lower(variables.size(), -1.0);
    std::vector<double> upper(variables.size(), 1.0);
    std::fill(lower.begin(), lower.begin() + tau_count, -config_.tau_bound);
    std::fill(upper.begin(), upper.begin() + tau_count, config_.tau_bound);

    StageBOptimizationResult result;
    result.initial_terms = evaluate(variables, context);
    double minimum = result.initial_terms.total;
    nlopt::opt optimizer(nlopt::LD_LBFGS, static_cast<unsigned>(variables.size()));
    optimizer.set_min_objective(objective, &context);
    optimizer.set_lower_bounds(lower);
    optimizer.set_upper_bounds(upper);
    optimizer.set_ftol_rel(config_.relative_tolerance);
    optimizer.set_xtol_rel(config_.relative_tolerance);
    optimizer.set_maxeval(config_.max_evaluations);
    try
    {
        optimizer.optimize(variables, minimum);
        result.optimized = true;
    }
    catch (const std::exception& error)
    {
        throw std::runtime_error(std::string("NLopt stage B optimization failed: ") + error.what());
    }

    result.evaluations = context.evaluations;
    result.durations = durationsFromVariables(variables, context);
    result.position_waypoints = positionsFromVariables(variables, context);
    result.final_terms = evaluate(variables, context);
    return result;
}

StageBOptimizationResult ActuatorPenaltyOptimizer::optimizeJoint(
    const std::vector<double>& initial_durations,
    const FlatOutputTrajectory::PositionMatrix& nominal_position_waypoints,
    const FlatOutputTrajectory::YawMatrix& yaw_waypoints,
    const std::vector<int>& auxiliary_waypoint_indices) const
{
    const double initial_total_time =
        std::accumulate(initial_durations.begin(), initial_durations.end(), 0.0);
    validateInputs(initial_total_time,
                   initial_durations,
                   nominal_position_waypoints,
                   yaw_waypoints,
                   auxiliary_waypoint_indices,
                   config_,
                   false);

    FlatOutputTrajectory reference;
    reference.update(initial_durations, nominal_position_waypoints, yaw_waypoints);
    const double smoothness_reference =
        std::max(1e-12, reference.energy(config_.yaw_weight));
    ObjectiveContext context{initial_total_time,
                             smoothness_reference,
                             false,
                             &initial_durations,
                             &nominal_position_waypoints,
                             &yaw_waypoints,
                             &auxiliary_waypoint_indices,
                             &flatness_,
                             &config_};

    const std::size_t duration_variable_count = durationVariableCount(context);
    std::vector<double> variables(
        duration_variable_count + 3 * auxiliary_waypoint_indices.size(), 0.0);
    std::vector<double> lower(variables.size(), -1.0);
    std::vector<double> upper(variables.size(), 1.0);
    std::fill(lower.begin(),
              lower.begin() + duration_variable_count,
              -config_.free_time_log_bound);
    std::fill(upper.begin(),
              upper.begin() + duration_variable_count,
              config_.free_time_log_bound);

    StageBOptimizationResult result;
    result.initial_terms = evaluate(variables, context);
    double minimum = result.initial_terms.total;
    nlopt::opt optimizer(nlopt::LD_LBFGS, static_cast<unsigned>(variables.size()));
    optimizer.set_min_objective(objective, &context);
    optimizer.set_lower_bounds(lower);
    optimizer.set_upper_bounds(upper);
    optimizer.set_ftol_rel(config_.relative_tolerance);
    optimizer.set_xtol_rel(config_.relative_tolerance);
    optimizer.set_maxeval(config_.max_evaluations);
    try
    {
        optimizer.optimize(variables, minimum);
        result.optimized = true;
    }
    catch (const std::exception& error)
    {
        throw std::runtime_error(
            std::string("NLopt joint stage optimization failed: ") + error.what());
    }

    result.evaluations = context.evaluations;
    result.durations = durationsFromVariables(variables, context);
    result.position_waypoints = positionsFromVariables(variables, context);
    result.final_terms = evaluate(variables, context);
    return result;
}

StageBGradientCheckResult ActuatorPenaltyOptimizer::checkInitialGradient(
    double total_time,
    const std::vector<double>& initial_durations,
    const FlatOutputTrajectory::PositionMatrix& nominal_position_waypoints,
    const FlatOutputTrajectory::YawMatrix& yaw_waypoints,
    const std::vector<int>& auxiliary_waypoint_indices,
    double relative_step) const
{
    validateInputs(total_time,
                   initial_durations,
                   nominal_position_waypoints,
                   yaw_waypoints,
                   auxiliary_waypoint_indices,
                   config_,
                   true);
    if (!(relative_step > 0.0) || !std::isfinite(relative_step))
        throw std::invalid_argument("gradient-check step must be finite and positive");

    FlatOutputTrajectory reference;
    reference.update(initial_durations, nominal_position_waypoints, yaw_waypoints);
    const double smoothness_reference =
        std::max(1e-12, reference.energy(config_.yaw_weight));
    ObjectiveContext context{total_time,
                             smoothness_reference,
                             true,
                             &initial_durations,
                             &nominal_position_waypoints,
                             &yaw_waypoints,
                             &auxiliary_waypoint_indices,
                             &flatness_,
                             &config_};
    const std::size_t tau_count = durationVariableCount(context);
    std::vector<double> variables(
        tau_count + 3 * auxiliary_waypoint_indices.size(), 0.0);
    const double last_duration = initial_durations.back();
    for (std::size_t i = 0; i < tau_count; ++i)
        variables[i] = std::log(initial_durations[i] / last_duration);

    StageBGradientCheckResult result;
    const auto analytic_terms = evaluate(variables, context, &result.analytic);
    bool objective_values_valid = std::isfinite(analytic_terms.total) &&
                                  analytic_terms.total < 1e29;
    result.finite_difference.resize(variables.size(), 0.0);
    result.maximum_relative_error = 0.0;
    for (std::size_t variable = 0; variable < variables.size(); ++variable)
    {
        const double step = relative_step * std::max(1.0, std::abs(variables[variable]));
        auto plus = variables;
        auto minus = variables;
        plus[variable] += step;
        minus[variable] -= step;
        const double plus_value = evaluate(plus, context).total;
        const double minus_value = evaluate(minus, context).total;
        objective_values_valid = objective_values_valid &&
                                 std::isfinite(plus_value) && plus_value < 1e29 &&
                                 std::isfinite(minus_value) && minus_value < 1e29;
        result.finite_difference[variable] =
            (plus_value - minus_value) / (2.0 * step);
        const double scale = std::max(
            {1.0,
             std::abs(result.analytic[variable]),
             std::abs(result.finite_difference[variable])});
        result.maximum_relative_error = std::max(
            result.maximum_relative_error,
            std::abs(result.analytic[variable] - result.finite_difference[variable]) / scale);
    }
    result.valid = objective_values_valid &&
                   std::isfinite(result.maximum_relative_error) &&
                   std::all_of(result.analytic.begin(), result.analytic.end(),
                               [](double value) { return std::isfinite(value); }) &&
                   std::all_of(result.finite_difference.begin(), result.finite_difference.end(),
                               [](double value) { return std::isfinite(value); });
    return result;
}

StageBGradientCheckResult ActuatorPenaltyOptimizer::checkInitialJointGradient(
    const std::vector<double>& initial_durations,
    const FlatOutputTrajectory::PositionMatrix& nominal_position_waypoints,
    const FlatOutputTrajectory::YawMatrix& yaw_waypoints,
    const std::vector<int>& auxiliary_waypoint_indices,
    double relative_step) const
{
    const double initial_total_time =
        std::accumulate(initial_durations.begin(), initial_durations.end(), 0.0);
    validateInputs(initial_total_time,
                   initial_durations,
                   nominal_position_waypoints,
                   yaw_waypoints,
                   auxiliary_waypoint_indices,
                   config_,
                   false);
    if (!(relative_step > 0.0) || !std::isfinite(relative_step))
        throw std::invalid_argument("gradient-check step must be finite and positive");

    FlatOutputTrajectory reference;
    reference.update(initial_durations, nominal_position_waypoints, yaw_waypoints);
    const double smoothness_reference =
        std::max(1e-12, reference.energy(config_.yaw_weight));
    ObjectiveContext context{initial_total_time,
                             smoothness_reference,
                             false,
                             &initial_durations,
                             &nominal_position_waypoints,
                             &yaw_waypoints,
                             &auxiliary_waypoint_indices,
                             &flatness_,
                             &config_};
    const std::size_t duration_variable_count = durationVariableCount(context);
    std::vector<double> variables(
        duration_variable_count + 3 * auxiliary_waypoint_indices.size(), 0.0);

    StageBGradientCheckResult result;
    const auto analytic_terms = evaluate(variables, context, &result.analytic);
    bool objective_values_valid = std::isfinite(analytic_terms.total) &&
                                  analytic_terms.total < 1e29;
    result.finite_difference.resize(variables.size(), 0.0);
    result.maximum_relative_error = 0.0;
    for (std::size_t variable = 0; variable < variables.size(); ++variable)
    {
        const double step = relative_step * std::max(1.0, std::abs(variables[variable]));
        auto plus = variables;
        auto minus = variables;
        plus[variable] += step;
        minus[variable] -= step;
        const double plus_value = evaluate(plus, context).total;
        const double minus_value = evaluate(minus, context).total;
        objective_values_valid = objective_values_valid &&
                                 std::isfinite(plus_value) && plus_value < 1e29 &&
                                 std::isfinite(minus_value) && minus_value < 1e29;
        result.finite_difference[variable] =
            (plus_value - minus_value) / (2.0 * step);
        const double scale = std::max(
            {1.0,
             std::abs(result.analytic[variable]),
             std::abs(result.finite_difference[variable])});
        result.maximum_relative_error = std::max(
            result.maximum_relative_error,
            std::abs(result.analytic[variable] - result.finite_difference[variable]) /
                scale);
    }
    result.valid = objective_values_valid &&
                   std::isfinite(result.maximum_relative_error) &&
                   std::all_of(result.analytic.begin(), result.analytic.end(),
                               [](double value) { return std::isfinite(value); }) &&
                   std::all_of(result.finite_difference.begin(),
                               result.finite_difference.end(),
                               [](double value) { return std::isfinite(value); });
    return result;
}

}  // namespace planning
