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

// Penalty/constraint type indices. The first ten become explicit inequality
// rows in the Stage C constrained optimizer; kTypeInvalidMapping stays a
// barrier in the objective because it has no usable gradient.
constexpr int kPenaltyTypeCount = 11;
constexpr int kConstraintRowCount = kPenaltyTypeCount - 1;
constexpr int kTypeThrust1 = 0;
constexpr int kTypeThrust2 = 1;
constexpr int kTypeFlap1 = 2;
constexpr int kTypeFlap2 = 3;
constexpr int kTypeBodyRate = 4;
constexpr int kTypeVelocity = 5;
constexpr int kTypeCollective = 6;
constexpr int kTypeMotorDynamics = 7;
constexpr int kTypeFlapRate = 8;
constexpr int kTypeServoDynamics = 9;
constexpr int kTypeInvalidMapping = 10;

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
    // Per-type coefficient/time partials sharing the layout of the combined
    // matrices above. Filled whenever gradients are requested; the combined
    // matrices are their exact sum.
    std::vector<FlatOutputTrajectory::PositionMatrix> type_position_coefficients;
    std::vector<FlatOutputTrajectory::YawMatrix> type_yaw_coefficients;
    std::vector<Eigen::VectorXd> type_position_times;
    std::vector<Eigen::VectorXd> type_yaw_times;
    // Diagnostics on the optimization grid: worst raw normalized violation of
    // any planning-margin envelope (positive means violated), and how many
    // samples had an invalid flatness mapping (which sets the violation to
    // +infinity because no actuator value exists there).
    double max_raw_violation{-std::numeric_limits<double>::infinity()};
    int invalid_samples{0};
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
    double trap_weight{0.5};
    FlatOutputSample sample;
    ActuatorPrediction prediction;
    ActuatorLinearization linearization;
    // First-pass penalty values and scalar derivatives, reused by the gradient
    // pass. A default-constructed PenaltyValueDerivative is an inactive hinge.
    PenaltyValueDerivative thrust_1;
    PenaltyValueDerivative thrust_2;
    PenaltyValueDerivative flap_1;
    PenaltyValueDerivative flap_2;
    PenaltyValueDerivative body_rate;
    PenaltyValueDerivative velocity;
    PenaltyValueDerivative collective_thrust;
    Eigen::Vector3d body_rate_gradient{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity_gradient{Eigen::Vector3d::Zero()};
    double invalid_penalty{0.0};
};

// Stored endpoint gradient slots of one dynamics interval, consumed by the
// gradient pass after the lazy linearizations are available.
struct DynamicsIntervalGradient
{
    bool active{false};
    Eigen::Vector4d first[3];
    Eigen::Vector4d second[3];
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

void accumulateFlatGradient(
    FlatOutputTrajectory::PositionMatrix& position_partial,
    FlatOutputTrajectory::YawMatrix& yaw_partial,
    Eigen::VectorXd& position_time_partial,
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
    auto position_block = position_partial.middleRows(
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
    auto yaw_block = yaw_partial.middleRows(
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
    position_time_partial[segment_index] +=
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
    if (config.enable_high_level_constraints && config.enable_velocity_constraint &&
        !(parameters.velocity_max > 0.0))
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
    evaluation.type_position_coefficients.assign(
        kPenaltyTypeCount,
        FlatOutputTrajectory::PositionMatrix::Zero(
            segment_count * FlatOutputTrajectory::PositionSpline::COEFF_NUM, 3));
    evaluation.type_yaw_coefficients.assign(
        kPenaltyTypeCount,
        FlatOutputTrajectory::YawMatrix::Zero(
            segment_count * FlatOutputTrajectory::YawSpline::COEFF_NUM, 1));
    evaluation.type_position_times.assign(
        kPenaltyTypeCount, Eigen::VectorXd::Zero(segment_count));
    evaluation.type_yaw_times.assign(
        kPenaltyTypeCount, Eigen::VectorXd::Zero(segment_count));

    BranchState branch;
    double segment_start = 0.0;
    for (Eigen::Index segment_index = 0; segment_index < segment_count; ++segment_index)
    {
        const double duration = trajectory.durations()[static_cast<std::size_t>(segment_index)];
        const double step = duration / config.samples_per_segment;
        std::vector<PenaltySample> segment_samples;
        segment_samples.reserve(static_cast<std::size_t>(config.samples_per_segment) + 1);
        std::vector<char> need_jacobian(
            static_cast<std::size_t>(config.samples_per_segment) + 1, 0);
        // Endpoint gradient slots of dynamics intervals, filled during the
        // value pass and consumed after the lazy linearizations.
        std::vector<DynamicsIntervalGradient> interval_gradients(
            static_cast<std::size_t>(config.samples_per_segment));
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
            // First pass evaluates values only; the AutoDiff linearization is
            // deferred to the samples that actually carry gradient mass.
            ActuatorPrediction prediction = flatness.forward(sample, branch);

            if (prediction.attitude.coeffs().allFinite() && prediction.attitude.norm() > 0.5)
            {
                branch.initialized = true;
                branch.attitude = prediction.attitude;
            }
            PenaltySample record;
            record.local_fraction = local_fraction;
            record.local_time = local_time;
            record.trap_weight = trap_weight;
            record.sample = sample;
            record.prediction = prediction;
            segment_samples.push_back(std::move(record));
            PenaltySample& stored = segment_samples.back();

            double thrust_1_raw = -std::numeric_limits<double>::infinity();
            double thrust_2_raw = -std::numeric_limits<double>::infinity();
            const auto thrust_penalty = [&](double thrust, double& raw) {
                PenaltyValueDerivative result;
                raw = -std::numeric_limits<double>::infinity();
                if (!std::isfinite(thrust))
                {
                    result.value = config.invalid_mapping_penalty;
                    return result;
                }
                const double lower_input = (thrust_min - thrust) / thrust_scale;
                const double upper_input = (thrust - thrust_max) / thrust_scale;
                raw = std::max(lower_input, upper_input);
                const auto lower = squaredPositivePartWithDerivative(lower_input);
                const auto upper = squaredPositivePartWithDerivative(upper_input);
                result.value = lower.value + upper.value;
                result.derivative =
                    (-lower.derivative + upper.derivative) / thrust_scale;
                return result;
            };
            const auto thrust_1_penalty =
                thrust_penalty(prediction.thrust_1, thrust_1_raw);
            const auto thrust_2_penalty =
                thrust_penalty(prediction.thrust_2, thrust_2_raw);
            stored.thrust_1 = thrust_1_penalty;
            stored.thrust_2 = thrust_2_penalty;
            evaluation.integral.thrust_1 += quadrature_weight * thrust_1_penalty.value;
            evaluation.integral.thrust_2 += quadrature_weight * thrust_2_penalty.value;

            double flap_1_raw = -std::numeric_limits<double>::infinity();
            double flap_2_raw = -std::numeric_limits<double>::infinity();
            const auto flap_penalty = [&](double flap, double& raw) {
                PenaltyValueDerivative result;
                raw = -std::numeric_limits<double>::infinity();
                if (!std::isfinite(flap))
                {
                    result.value = config.invalid_mapping_penalty;
                    return result;
                }
                const double magnitude_input = (std::abs(flap) - flap_max) / flap_max;
                raw = magnitude_input;
                const auto magnitude =
                    squaredPositivePartWithDerivative(magnitude_input);
                result.value = magnitude.value;
                if (magnitude.derivative != 0.0)
                    result.derivative = magnitude.derivative *
                                        (flap >= 0.0 ? 1.0 : -1.0) / flap_max;
                return result;
            };
            const auto flap_1_penalty = flap_penalty(prediction.flap_1, flap_1_raw);
            const auto flap_2_penalty = flap_penalty(prediction.flap_2, flap_2_raw);
            stored.flap_1 = flap_1_penalty;
            stored.flap_2 = flap_2_penalty;
            evaluation.integral.flap_1 += quadrature_weight * flap_1_penalty.value;
            evaluation.integral.flap_2 += quadrature_weight * flap_2_penalty.value;

            double body_rate_raw = -std::numeric_limits<double>::infinity();
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
                    body_rate_raw = (body_rate - body_rate_max) / body_rate_max;
                    body_rate_penalty =
                        squaredPositivePartWithDerivative(body_rate_raw);
                    if (body_rate_penalty.derivative != 0.0 && body_rate > 1e-12)
                    {
                        body_rate_penalty_gradient =
                            body_rate_penalty.derivative / body_rate_max *
                            prediction.angular_velocity / body_rate;
                    }
                }
                stored.body_rate = body_rate_penalty;
                stored.body_rate_gradient = body_rate_penalty_gradient;
                evaluation.integral.body_rate +=
                    quadrature_weight * body_rate_penalty.value;
            }

            double velocity_raw = -std::numeric_limits<double>::infinity();
            PenaltyValueDerivative velocity_penalty;
            Eigen::Vector3d velocity_penalty_gradient = Eigen::Vector3d::Zero();
            PenaltyValueDerivative collective_thrust_penalty;
            double collective_raw = -std::numeric_limits<double>::infinity();
            if (config.enable_high_level_constraints)
            {
                if (config.enable_velocity_constraint)
                {
                    const double speed = sample.velocity.norm();
                    velocity_raw = (speed - velocity_max) / velocity_max;
                    velocity_penalty =
                        squaredPositivePartWithDerivative(velocity_raw);
                    if (velocity_penalty.derivative != 0.0 && speed > 1e-12)
                    {
                        velocity_penalty_gradient =
                            velocity_penalty.derivative / velocity_max *
                            sample.velocity / speed;
                    }
                    evaluation.integral.velocity +=
                        quadrature_weight * velocity_penalty.value;
                }
                if (std::isfinite(prediction.collective_thrust))
                {
                    collective_raw = std::max(
                        (collective_thrust_min - prediction.collective_thrust) /
                            collective_thrust_scale,
                        (prediction.collective_thrust - collective_thrust_max) /
                            collective_thrust_scale);
                }
                collective_thrust_penalty = boundedSquaredPenalty(
                    prediction.collective_thrust,
                    collective_thrust_min,
                    collective_thrust_max,
                    collective_thrust_scale);
                stored.velocity = velocity_penalty;
                stored.velocity_gradient = velocity_penalty_gradient;
                stored.collective_thrust = collective_thrust_penalty;
                evaluation.integral.collective_thrust +=
                    quadrature_weight * collective_thrust_penalty.value;
            }
            double invalid_penalty = 0.0;
            if (!prediction.valid)
            {
                invalid_penalty = config.invalid_mapping_penalty;
                stored.invalid_penalty = invalid_penalty;
                evaluation.integral.invalid_mapping += quadrature_weight * invalid_penalty;
            }
            if (!prediction.valid)
            {
                ++evaluation.invalid_samples;
                evaluation.max_raw_violation =
                    std::numeric_limits<double>::infinity();
            }
            else
            {
                evaluation.max_raw_violation = std::max(
                    {evaluation.max_raw_violation,
                     thrust_1_raw, thrust_2_raw,
                     flap_1_raw, flap_2_raw,
                     body_rate_raw, velocity_raw, collective_raw});
            }

            if (!compute_gradient)
                continue;

            // Explicit time partials of every type's integral: the quadrature
            // weight (step * trap_weight) scales with the segment duration and
            // the sample itself slides with the segment.
            const double time_value_weight = trap_weight / config.samples_per_segment;
            const auto add_type_time_value = [&](int type, double value) {
                if (value != 0.0)
                    evaluation.type_position_times[type][segment_index] +=
                        time_value_weight * value;
            };
            add_type_time_value(kTypeThrust1, thrust_1_penalty.value);
            add_type_time_value(kTypeThrust2, thrust_2_penalty.value);
            add_type_time_value(kTypeFlap1, flap_1_penalty.value);
            add_type_time_value(kTypeFlap2, flap_2_penalty.value);
            add_type_time_value(kTypeBodyRate, body_rate_penalty.value);
            add_type_time_value(kTypeVelocity, velocity_penalty.value);
            add_type_time_value(kTypeCollective, collective_thrust_penalty.value);
            add_type_time_value(kTypeInvalidMapping, invalid_penalty);

            // Defer the expensive AutoDiff linearization to samples that
            // actually carry gradient mass; an all-inactive sample contributes
            // exactly zero to every type's flat gradient.
            if (thrust_1_penalty.derivative != 0.0 || thrust_2_penalty.derivative != 0.0 ||
                flap_1_penalty.derivative != 0.0 || flap_2_penalty.derivative != 0.0 ||
                collective_thrust_penalty.derivative != 0.0 ||
                body_rate_penalty_gradient.squaredNorm() > 0.0 ||
                velocity_penalty_gradient.squaredNorm() > 0.0)
            {
                need_jacobian[static_cast<std::size_t>(sample_index)] = 1;
            }
        }

        if (config.enable_actuator_dynamics)
        {
            const double rotor_command_min = parameters.omega_min;
            const double rotor_command_max = planning_omega_max;
            const double rotor_command_scale =
                std::max(1e-9, rotor_command_max - rotor_command_min);
            const double flap_rate_max =
                config.actuator_margin * parameters.servo_rate_max;
            // Dynamics gradient slots: 0 = motor dynamics, 1 = servo
            // dynamics, 2 = flap rate.
            constexpr int dynamics_type[3] = {kTypeMotorDynamics,
                                              kTypeServoDynamics,
                                              kTypeFlapRate};
            for (int interval = 0; interval < config.samples_per_segment; ++interval)
            {
                const auto& first = segment_samples[static_cast<std::size_t>(interval)];
                const auto& second = segment_samples[static_cast<std::size_t>(interval + 1)];
                const auto first_state =
                    dynamicActuatorState(first, parameters, false);
                const auto second_state =
                    dynamicActuatorState(second, parameters, false);
                if (!first_state.valid || !second_state.valid)
                    continue;

                Eigen::Vector4d type_first[3] = {Eigen::Vector4d::Zero(),
                                                Eigen::Vector4d::Zero(),
                                                Eigen::Vector4d::Zero()};
                Eigen::Vector4d type_second[3] = {Eigen::Vector4d::Zero(),
                                                  Eigen::Vector4d::Zero(),
                                                  Eigen::Vector4d::Zero()};
                double type_duration_partial[3] = {0.0, 0.0, 0.0};

                const auto add_first_order_dynamics =
                    [&](int slot,
                        int actuator,
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
                        if (std::isfinite(command))
                        {
                            const double raw = symmetric_bound
                                ? std::abs(command) / upper_bound - 1.0
                                : std::max((lower_bound - command) / rotor_command_scale,
                                           (command - upper_bound) / rotor_command_scale);
                            evaluation.max_raw_violation =
                                std::max(evaluation.max_raw_violation, raw);
                        }
                        const double command_first = -decay / response;
                        const double command_second = 1.0 / response;
                        type_first[slot][actuator] +=
                            step * penalty.derivative * command_first;
                        type_second[slot][actuator] +=
                            step * penalty.derivative * command_second;
                        const double command_step =
                            -decay / time_constant *
                            (second_state.value[actuator] - first_state.value[actuator]) /
                            (response * response);
                        type_duration_partial[slot] +=
                            (penalty.value +
                             step * penalty.derivative * command_step) /
                            config.samples_per_segment;
                    };

                add_first_order_dynamics(0, 0,
                                         parameters.motor_time_constant,
                                         false,
                                         rotor_command_min,
                                         rotor_command_max,
                                         evaluation.integral.motor_dynamics);
                add_first_order_dynamics(0, 1,
                                         parameters.motor_time_constant,
                                         false,
                                         rotor_command_min,
                                         rotor_command_max,
                                         evaluation.integral.motor_dynamics);
                add_first_order_dynamics(1, 2,
                                         parameters.servo_time_constant,
                                         true,
                                         -flap_max,
                                         flap_max,
                                         evaluation.integral.servo_dynamics);
                add_first_order_dynamics(1, 3,
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
                    type_first[2][flap] -= penalty.derivative;
                    type_second[2][flap] += penalty.derivative;
                    type_duration_partial[2] +=
                        (penalty.value - penalty.derivative * rate) /
                        config.samples_per_segment;
                    evaluation.max_raw_violation = std::max(
                        evaluation.max_raw_violation,
                        std::abs(rate) / flap_rate_max - 1.0);
                }
                for (int slot = 0; slot < 3; ++slot)
                {
                    evaluation.type_position_times[dynamics_type[slot]][segment_index] +=
                        type_duration_partial[slot];
                }

                // Remember the endpoint slots and mark both samples for the
                // deferred linearization when this interval carries gradient
                // mass.
                if (!compute_gradient)
                    continue;
                bool any_slot = false;
                for (int slot = 0; slot < 3; ++slot)
                {
                    any_slot = any_slot ||
                        type_first[slot].array().abs().maxCoeff() > 0.0 ||
                        type_second[slot].array().abs().maxCoeff() > 0.0;
                }
                if (!any_slot)
                    continue;
                auto& stored_interval = interval_gradients[static_cast<std::size_t>(interval)];
                stored_interval.active = true;
                for (int slot = 0; slot < 3; ++slot)
                {
                    stored_interval.first[slot] = type_first[slot];
                    stored_interval.second[slot] = type_second[slot];
                }
                need_jacobian[static_cast<std::size_t>(interval)] = 1;
                need_jacobian[static_cast<std::size_t>(interval) + 1] = 1;
            }
        }

        // Second pass: linearize only the marked samples, reusing the
        // first-pass prediction and its already-selected attitude branches so
        // no duplicate forward evaluation happens.
        if (compute_gradient)
        {
            for (std::size_t sample_index = 0;
                 sample_index < segment_samples.size();
                 ++sample_index)
            {
                PenaltySample& record = segment_samples[sample_index];
                if (!need_jacobian[sample_index] || !record.prediction.valid)
                    continue;
                record.linearization = flatness.linearize(
                    record.sample,
                    record.prediction,
                    record.prediction.roll_branch,
                    record.prediction.pitch_branch);
            }

            // Third pass, amplitude types: assemble per-type flat gradients
            // through the frozen-branch Jacobians where a hinge is active.
            for (PenaltySample& record : segment_samples)
            {
                if (!record.linearization.differentiable)
                    continue;
                const double quadrature_weight = step * record.trap_weight;
                const auto add_type_flat = [&](int type, const FlatGradient& gradient) {
                    accumulateFlatGradient(
                        evaluation.type_position_coefficients[type],
                        evaluation.type_yaw_coefficients[type],
                        evaluation.type_position_times[type],
                        segment_index,
                        record,
                        quadrature_weight * gradient);
                };
                if (record.thrust_1.derivative != 0.0)
                    add_type_flat(kTypeThrust1,
                                  record.linearization.jacobian.row(0).transpose() *
                                      record.thrust_1.derivative);
                if (record.thrust_2.derivative != 0.0)
                    add_type_flat(kTypeThrust2,
                                  record.linearization.jacobian.row(1).transpose() *
                                      record.thrust_2.derivative);
                if (record.flap_1.derivative != 0.0)
                    add_type_flat(kTypeFlap1,
                                  record.linearization.jacobian.row(2).transpose() *
                                      record.flap_1.derivative);
                if (record.flap_2.derivative != 0.0)
                    add_type_flat(kTypeFlap2,
                                  record.linearization.jacobian.row(3).transpose() *
                                      record.flap_2.derivative);
                if (record.collective_thrust.derivative != 0.0)
                    add_type_flat(kTypeCollective,
                                  (record.linearization.jacobian.row(0) +
                                   record.linearization.jacobian.row(1))
                                      .transpose() *
                                      record.collective_thrust.derivative);
                if (record.body_rate_gradient.squaredNorm() > 0.0)
                    add_type_flat(kTypeBodyRate,
                                  record.linearization.angular_velocity_jacobian
                                          .transpose() *
                                      record.body_rate_gradient);
                if (record.velocity_gradient.squaredNorm() > 0.0)
                {
                    FlatGradient velocity_flat = FlatGradient::Zero();
                    velocity_flat.segment<3>(ActuatorLinearization::velocity_offset) =
                        record.velocity_gradient;
                    add_type_flat(kTypeVelocity, velocity_flat);
                }
            }

            // Third pass, dynamics types: reuse the stored endpoint slots.
            if (config.enable_actuator_dynamics)
            {
                constexpr int dynamics_type[3] = {kTypeMotorDynamics,
                                                  kTypeServoDynamics,
                                                  kTypeFlapRate};
                for (int interval = 0;
                     interval < config.samples_per_segment;
                     ++interval)
                {
                    const DynamicsIntervalGradient& stored_interval =
                        interval_gradients[static_cast<std::size_t>(interval)];
                    if (!stored_interval.active)
                        continue;
                    const PenaltySample& first =
                        segment_samples[static_cast<std::size_t>(interval)];
                    const PenaltySample& second =
                        segment_samples[static_cast<std::size_t>(interval) + 1];
                    const auto first_state =
                        dynamicActuatorState(first, parameters, true);
                    const auto second_state =
                        dynamicActuatorState(second, parameters, true);
                    if (!first_state.differentiable || !second_state.differentiable)
                        continue;
                    for (int slot = 0; slot < 3; ++slot)
                    {
                        if (stored_interval.first[slot].array().abs().maxCoeff() > 0.0)
                        {
                            accumulateFlatGradient(
                                evaluation.type_position_coefficients[dynamics_type[slot]],
                                evaluation.type_yaw_coefficients[dynamics_type[slot]],
                                evaluation.type_position_times[dynamics_type[slot]],
                                segment_index,
                                first,
                                first_state.jacobian.transpose() *
                                    stored_interval.first[slot]);
                        }
                        if (stored_interval.second[slot].array().abs().maxCoeff() > 0.0)
                        {
                            accumulateFlatGradient(
                                evaluation.type_position_coefficients[dynamics_type[slot]],
                                evaluation.type_yaw_coefficients[dynamics_type[slot]],
                                evaluation.type_position_times[dynamics_type[slot]],
                                segment_index,
                                second,
                                second_state.jacobian.transpose() *
                                    stored_interval.second[slot]);
                        }
                    }
                }
            }
        }
        segment_start += duration;
    }

    // The combined partials are the exact sum of the per-type partials; the
    // penalty objective keeps consuming this single aggregated form.
    if (compute_gradient)
    {
        for (int type = 0; type < kPenaltyTypeCount; ++type)
        {
            evaluation.partial_position_coefficients +=
                evaluation.type_position_coefficients[static_cast<std::size_t>(type)];
            evaluation.partial_yaw_coefficients +=
                evaluation.type_yaw_coefficients[static_cast<std::size_t>(type)];
            evaluation.partial_position_times +=
                evaluation.type_position_times[static_cast<std::size_t>(type)];
            evaluation.partial_yaw_times +=
                evaluation.type_yaw_times[static_cast<std::size_t>(type)];
        }
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

// ---------------------------------------------------------------------------
// Stage C: joint optimization with explicit inequality constraints
// ---------------------------------------------------------------------------

std::vector<double> freeDurationsFromVariables(
    const std::vector<double>& variables,
    const std::vector<double>& initial_durations)
{
    std::vector<double> durations(initial_durations.size());
    for (std::size_t segment = 0; segment < durations.size(); ++segment)
        durations[segment] = initial_durations[segment] * std::exp(variables[segment]);
    return durations;
}

FlatOutputTrajectory::PositionMatrix freePositionsFromVariables(
    const std::vector<double>& variables,
    const FlatOutputTrajectory::PositionMatrix& nominal_positions,
    const std::vector<int>& auxiliary_indices,
    const Eigen::Vector3d& half_range,
    std::size_t duration_variable_count)
{
    auto positions = nominal_positions;
    for (std::size_t auxiliary = 0; auxiliary < auxiliary_indices.size(); ++auxiliary)
    {
        const Eigen::Index row = auxiliary_indices[auxiliary];
        for (int axis = 0; axis < 3; ++axis)
        {
            const std::size_t variable =
                duration_variable_count + 3 * auxiliary + static_cast<std::size_t>(axis);
            positions(row, axis) += half_range[axis] * variables[variable];
        }
    }
    return positions;
}

struct ConstrainedEvaluation
{
    StageBObjectiveTerms terms;
    std::vector<double> constraint_values;
    bool objective_gradient_valid{false};
    std::vector<double> objective_gradient;
    bool constraint_jacobian_valid{false};
    std::vector<double> constraint_jacobian;
    double max_raw_violation{-std::numeric_limits<double>::infinity()};
    int invalid_samples{0};
};

struct ConstrainedContext
{
    double smoothness_reference;
    const std::vector<double>* initial_durations;
    const FlatOutputTrajectory::PositionMatrix* nominal_positions;
    const FlatOutputTrajectory::YawMatrix* yaws;
    const std::vector<int>* auxiliary_indices;
    const TailsitterFlatness* flatness;
    const ActuatorPenaltyConfig* config;
    int objective_evaluations{0};
    int constraint_evaluations{0};
    // NLopt evaluates the objective and the constraints at the same point
    // consecutively, so one pass through the expensive flatness integration
    // serves both callbacks.
    bool cache_valid{false};
    std::vector<double> cached_variables;
    ConstrainedEvaluation cache;
};

const ConstrainedEvaluation& constrainedEvaluation(
    const std::vector<double>& variables,
    ConstrainedContext& context,
    bool want_gradient)
{
    if (context.cache_valid && context.cached_variables == variables &&
        (!want_gradient || (context.cache.objective_gradient_valid &&
                            context.cache.constraint_jacobian_valid)))
    {
        return context.cache;
    }

    const auto& config = *context.config;
    const std::size_t duration_variable_count = context.initial_durations->size();
    const std::size_t variable_count = variables.size();
    ConstrainedEvaluation evaluation;
    evaluation.objective_gradient.assign(variable_count, 0.0);
    evaluation.constraint_jacobian.assign(
        kConstraintRowCount * variable_count, 0.0);

    bool usable = true;
    try
    {
        const auto durations =
            freeDurationsFromVariables(variables, *context.initial_durations);
        const auto positions = freePositionsFromVariables(
            variables,
            *context.nominal_positions,
            *context.auxiliary_indices,
            config.auxiliary_half_range,
            duration_variable_count);
        FlatOutputTrajectory trajectory;
        trajectory.update(durations, positions, *context.yaws);

        evaluation.terms.smoothness = trajectory.energy(config.yaw_weight);
        evaluation.terms.normalized_smoothness =
            evaluation.terms.smoothness / context.smoothness_reference;
        evaluation.terms.total_time =
            std::accumulate(durations.begin(), durations.end(), 0.0);
        evaluation.terms.time = config.time_weight * evaluation.terms.total_time;
        for (std::size_t i = duration_variable_count; i < variable_count; ++i)
            evaluation.terms.geometry += variables[i] * variables[i];

        const auto actuator = integrateActuatorPenalty(
            trajectory, *context.flatness, config, want_gradient);
        evaluation.terms.actuator = actuator.integral;
        evaluation.max_raw_violation = actuator.max_raw_violation;
        evaluation.invalid_samples = actuator.invalid_samples;
        evaluation.terms.total =
            config.smoothness_weight * evaluation.terms.normalized_smoothness +
            config.geometry_weight * evaluation.terms.geometry +
            config.time_weight * evaluation.terms.total_time +
            config.actuator_weight * actuator.integral.invalid_mapping;
        evaluation.constraint_values = {
            actuator.integral.thrust_1,
            actuator.integral.thrust_2,
            actuator.integral.flap_1,
            actuator.integral.flap_2,
            actuator.integral.body_rate,
            actuator.integral.velocity,
            actuator.integral.collective_thrust,
            actuator.integral.motor_dynamics,
            actuator.integral.flap_rate,
            actuator.integral.servo_dynamics};

        if (want_gradient)
        {
            const auto smoothness_gradient =
                trajectory.energyGradient(config.yaw_weight);
            const auto invalid_gradient = trajectory.propagateCostGradient(
                actuator.type_position_coefficients[kTypeInvalidMapping],
                actuator.type_position_times[kTypeInvalidMapping],
                actuator.type_yaw_coefficients[kTypeInvalidMapping],
                actuator.type_yaw_times[kTypeInvalidMapping]);

            Eigen::VectorXd duration_gradient =
                config.smoothness_weight / context.smoothness_reference *
                    smoothness_gradient.times +
                config.actuator_weight * invalid_gradient.times;
            duration_gradient.array() += config.time_weight;
            const auto position_inner =
                config.smoothness_weight / context.smoothness_reference *
                    smoothness_gradient.position_inner_points +
                config.actuator_weight * invalid_gradient.position_inner_points;
            for (std::size_t variable = 0;
                 variable < duration_variable_count;
                 ++variable)
            {
                evaluation.objective_gradient[variable] =
                    durations[variable] *
                    duration_gradient[static_cast<Eigen::Index>(variable)];
            }
            for (std::size_t auxiliary = 0;
                 auxiliary < context.auxiliary_indices->size();
                 ++auxiliary)
            {
                const Eigen::Index waypoint = (*context.auxiliary_indices)[auxiliary];
                for (int axis = 0; axis < 3; ++axis)
                {
                    const std::size_t variable = duration_variable_count +
                                                 3 * auxiliary +
                                                 static_cast<std::size_t>(axis);
                    evaluation.objective_gradient[variable] =
                        config.auxiliary_half_range[axis] *
                            position_inner(waypoint - 1, axis) +
                        2.0 * config.geometry_weight * variables[variable];
                }
            }

            for (int type = 0; type < kConstraintRowCount; ++type)
            {
                const auto propagated = trajectory.propagateCostGradient(
                    actuator.type_position_coefficients[static_cast<std::size_t>(type)],
                    actuator.type_position_times[static_cast<std::size_t>(type)],
                    actuator.type_yaw_coefficients[static_cast<std::size_t>(type)],
                    actuator.type_yaw_times[static_cast<std::size_t>(type)]);
                double* row = &evaluation.constraint_jacobian
                    [static_cast<std::size_t>(type) * variable_count];
                for (std::size_t variable = 0;
                     variable < duration_variable_count;
                     ++variable)
                {
                    row[variable] =
                        durations[variable] *
                        propagated.times[static_cast<Eigen::Index>(variable)];
                }
                for (std::size_t auxiliary = 0;
                     auxiliary < context.auxiliary_indices->size();
                     ++auxiliary)
                {
                    const Eigen::Index waypoint =
                        (*context.auxiliary_indices)[auxiliary];
                    for (int axis = 0; axis < 3; ++axis)
                    {
                        const std::size_t variable = duration_variable_count +
                                                     3 * auxiliary +
                                                     static_cast<std::size_t>(axis);
                        row[variable] =
                            config.auxiliary_half_range[axis] *
                            propagated.position_inner_points(waypoint - 1, axis);
                    }
                }
            }
            evaluation.objective_gradient_valid = true;
            evaluation.constraint_jacobian_valid = true;
        }

        const bool gradient_finite =
            !want_gradient ||
            (std::all_of(evaluation.objective_gradient.begin(),
                         evaluation.objective_gradient.end(),
                         [](double value) { return std::isfinite(value); }) &&
             std::all_of(evaluation.constraint_jacobian.begin(),
                         evaluation.constraint_jacobian.end(),
                         [](double value) { return std::isfinite(value); }));
        if (!std::isfinite(evaluation.terms.total) || !gradient_finite)
            usable = false;
    }
    catch (const std::exception&)
    {
        usable = false;
    }
    if (!usable)
    {
        // A candidate that cannot be constructed, or whose gradients are not
        // finite, gets the high objective barrier and every constraint row
        // reported as violated; gradients become zero so the solver still has
        // a descent direction away from the barrier on the objective side.
        evaluation.terms.total = 1e30;
        evaluation.constraint_values.assign(kConstraintRowCount, 1.0);
        std::fill(evaluation.objective_gradient.begin(),
                  evaluation.objective_gradient.end(),
                  0.0);
        std::fill(evaluation.constraint_jacobian.begin(),
                  evaluation.constraint_jacobian.end(),
                  0.0);
        if (want_gradient)
        {
            evaluation.objective_gradient_valid = true;
            evaluation.constraint_jacobian_valid = true;
        }
    }
    context.cache_valid = true;
    context.cached_variables = variables;
    context.cache = std::move(evaluation);
    return context.cache;
}

double constrainedObjective(const std::vector<double>& variables,
                            std::vector<double>& gradient,
                            void* raw_context)
{
    auto& context = *static_cast<ConstrainedContext*>(raw_context);
    ++context.objective_evaluations;
    const auto& evaluation =
        constrainedEvaluation(variables, context, !gradient.empty());
    if (!gradient.empty())
        gradient = evaluation.objective_gradient;
    return evaluation.terms.total;
}

void constrainedConstraintVector(unsigned row_count,
                                 double* result,
                                 unsigned variable_count,
                                 const double* raw_variables,
                                 double* raw_gradient,
                                 void* raw_context)
{
    auto& context = *static_cast<ConstrainedContext*>(raw_context);
    ++context.constraint_evaluations;
    const std::vector<double> variables(
        raw_variables, raw_variables + variable_count);
    const auto& evaluation =
        constrainedEvaluation(variables, context, raw_gradient != nullptr);
    for (unsigned row = 0; row < row_count; ++row)
        result[row] = evaluation.constraint_values[row];
    if (raw_gradient != nullptr)
    {
        std::copy(evaluation.constraint_jacobian.begin(),
                  evaluation.constraint_jacobian.end(),
                  raw_gradient);
    }
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

ConstrainedOptimizationResult ActuatorPenaltyOptimizer::optimizeJointConstrained(
    const std::vector<double>& initial_durations,
    const FlatOutputTrajectory::PositionMatrix& nominal_position_waypoints,
    const FlatOutputTrajectory::YawMatrix& yaw_waypoints,
    const std::vector<int>& auxiliary_waypoint_indices,
    const std::vector<double>& warm_start_durations,
    const FlatOutputTrajectory::PositionMatrix& warm_start_positions) const
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
    if (!(config_.constraint_tolerance >= 0.0) ||
        !(config_.constraint_satisfaction_tolerance >= 0.0) ||
        !(config_.local_solver_max_evaluations > 0))
    {
        throw std::invalid_argument(
            "invalid stage C constrained solver configuration");
    }

    FlatOutputTrajectory reference;
    reference.update(initial_durations, nominal_position_waypoints, yaw_waypoints);
    const double smoothness_reference =
        std::max(1e-12, reference.energy(config_.yaw_weight));
    ConstrainedContext context{smoothness_reference,
                               &initial_durations,
                               &nominal_position_waypoints,
                               &yaw_waypoints,
                               &auxiliary_waypoint_indices,
                               &flatness_,
                               &config_,
                               0,
                               0,
                               false,
                               {},
                               {}};

    const std::size_t duration_variable_count = initial_durations.size();
    std::vector<double> variables(
        duration_variable_count + 3 * auxiliary_waypoint_indices.size(), 0.0);
    if (!warm_start_durations.empty())
    {
        if (warm_start_durations.size() != initial_durations.size())
            throw std::invalid_argument(
                "warm-start durations must match the initial duration count");
        for (std::size_t segment = 0; segment < duration_variable_count; ++segment)
        {
            if (!(warm_start_durations[segment] > 0.0))
                throw std::invalid_argument(
                    "warm-start durations must be finite and positive");
            variables[segment] =
                std::log(warm_start_durations[segment] / initial_durations[segment]);
        }
        if (warm_start_positions.rows() != 0)
        {
            if (warm_start_positions.rows() != nominal_position_waypoints.rows())
                throw std::invalid_argument(
                    "warm-start positions must match the nominal waypoint count");
            for (std::size_t auxiliary = 0;
                 auxiliary < auxiliary_waypoint_indices.size();
                 ++auxiliary)
            {
                const Eigen::Index row = auxiliary_waypoint_indices[auxiliary];
                for (int axis = 0; axis < 3; ++axis)
                {
                    const std::size_t variable = duration_variable_count +
                                                 3 * auxiliary +
                                                 static_cast<std::size_t>(axis);
                    variables[variable] =
                        (warm_start_positions(row, axis) -
                         nominal_position_waypoints(row, axis)) /
                        config_.auxiliary_half_range[axis];
                }
            }
        }
        // Clip the warm start into the variable bounds so a warm point that
        // sits exactly on the box edge still starts strictly feasible.
        for (double& value : variables)
            value = std::clamp(value, -1.0 + 1e-12, 1.0 - 1e-12);
        for (std::size_t segment = 0; segment < duration_variable_count; ++segment)
            variables[segment] =
                std::clamp(variables[segment],
                           -config_.free_time_log_bound + 1e-12,
                           config_.free_time_log_bound - 1e-12);
    }
    std::vector<double> lower(variables.size(), -1.0);
    std::vector<double> upper(variables.size(), 1.0);
    std::fill(lower.begin(),
              lower.begin() + static_cast<std::ptrdiff_t>(duration_variable_count),
              -config_.free_time_log_bound);
    std::fill(upper.begin(),
              upper.begin() + static_cast<std::ptrdiff_t>(duration_variable_count),
              config_.free_time_log_bound);

    ConstrainedOptimizationResult result;
    const ConstrainedEvaluation initial =
        constrainedEvaluation(variables, context, false);
    result.initial_terms = initial.terms;
    result.initial_constraint_values = initial.constraint_values;
    result.initial_max_sample_violation = initial.max_raw_violation;

    nlopt::algorithm algorithm = nlopt::LD_AUGLAG;
    nlopt::algorithm subsolver = nlopt::LD_MMA;
    bool auglag = true;
    switch (config_.constrained_solver)
    {
        case ConstrainedSolver::AuglagMma:
            algorithm = nlopt::LD_AUGLAG;
            subsolver = nlopt::LD_MMA;
            break;
        case ConstrainedSolver::AuglagCcsaq:
            algorithm = nlopt::LD_AUGLAG;
            subsolver = nlopt::LD_CCSAQ;
            break;
        case ConstrainedSolver::AuglagSlsqp:
            algorithm = nlopt::LD_AUGLAG;
            subsolver = nlopt::LD_SLSQP;
            break;
        case ConstrainedSolver::Mma:
            algorithm = nlopt::LD_MMA;
            auglag = false;
            break;
        case ConstrainedSolver::Ccsaq:
            algorithm = nlopt::LD_CCSAQ;
            auglag = false;
            break;
        case ConstrainedSolver::Slsqp:
            algorithm = nlopt::LD_SLSQP;
            auglag = false;
            break;
    }
    nlopt::opt optimizer(algorithm, static_cast<unsigned>(variables.size()));
    if (auglag)
    {
        nlopt::opt local(subsolver, static_cast<unsigned>(variables.size()));
        local.set_lower_bounds(lower);
        local.set_upper_bounds(upper);
        local.set_ftol_rel(1e-10);
        local.set_xtol_rel(1e-10);
        local.set_maxeval(config_.local_solver_max_evaluations);
        optimizer.set_local_optimizer(local);
    }
    optimizer.set_min_objective(constrainedObjective, &context);
    optimizer.add_inequality_mconstraint(
        constrainedConstraintVector,
        &context,
        std::vector<double>(
            static_cast<std::size_t>(kConstraintRowCount),
            config_.constraint_tolerance));
    optimizer.set_lower_bounds(lower);
    optimizer.set_upper_bounds(upper);
    optimizer.set_ftol_rel(config_.relative_tolerance);
    optimizer.set_xtol_rel(config_.relative_tolerance);
    optimizer.set_maxeval(config_.max_evaluations);

    double minimum = initial.terms.total;
    try
    {
        optimizer.optimize(variables, minimum);
        result.optimized = true;
    }
    catch (const nlopt::roundoff_limited&)
    {
        // Keep the best iterate; the independent dense feasibility check
        // decides whether the returned trajectory is actually usable.
        result.optimized = true;
    }
    catch (const std::exception& error)
    {
        throw std::runtime_error(
            std::string("NLopt stage C constrained optimization failed: ") +
            error.what());
    }

    result.objective_evaluations = context.objective_evaluations;
    result.constraint_evaluations = context.constraint_evaluations;
    result.durations = freeDurationsFromVariables(variables, initial_durations);
    result.position_waypoints = freePositionsFromVariables(
        variables,
        nominal_position_waypoints,
        auxiliary_waypoint_indices,
        config_.auxiliary_half_range,
        duration_variable_count);
    const ConstrainedEvaluation final_evaluation =
        constrainedEvaluation(variables, context, false);
    result.final_terms = final_evaluation.terms;
    result.final_constraint_values = final_evaluation.constraint_values;
    result.final_max_sample_violation = final_evaluation.max_raw_violation;
    result.constraints_satisfied = std::all_of(
        result.final_constraint_values.begin(),
        result.final_constraint_values.end(),
        [this](double value) {
            return value <= config_.constraint_satisfaction_tolerance;
        });
    return result;
}

ConstraintGradientCheckResult
ActuatorPenaltyOptimizer::checkInitialJointConstraintGradient(
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
    ConstrainedContext context{smoothness_reference,
                               &initial_durations,
                               &nominal_position_waypoints,
                               &yaw_waypoints,
                               &auxiliary_waypoint_indices,
                               &flatness_,
                               &config_,
                               0,
                               0,
                               false,
                               {},
                               {}};
    const std::size_t duration_variable_count = initial_durations.size();
    std::vector<double> variables(
        duration_variable_count + 3 * auxiliary_waypoint_indices.size(), 0.0);

    ConstraintGradientCheckResult result;
    result.constraint_rows = kConstraintRowCount;
    const ConstrainedEvaluation analytic_evaluation =
        constrainedEvaluation(variables, context, true);
    result.analytic = analytic_evaluation.constraint_jacobian;
    bool values_valid = analytic_evaluation.terms.total < 1e29 &&
                        analytic_evaluation.constraint_jacobian_valid;
    result.finite_difference.assign(
        static_cast<std::size_t>(kConstraintRowCount) * variables.size(), 0.0);
    result.maximum_relative_error = 0.0;
    for (std::size_t variable = 0; variable < variables.size(); ++variable)
    {
        const double step =
            relative_step * std::max(1.0, std::abs(variables[variable]));
        auto plus = variables;
        auto minus = variables;
        plus[variable] += step;
        minus[variable] -= step;
        // Copy by value: the returned reference is a cache member that the
        // next evaluation overwrites in place.
        const ConstrainedEvaluation plus_evaluation =
            constrainedEvaluation(plus, context, false);
        const ConstrainedEvaluation minus_evaluation =
            constrainedEvaluation(minus, context, false);
        values_valid =
            values_valid && plus_evaluation.terms.total < 1e29 &&
            minus_evaluation.terms.total < 1e29;
        for (int row = 0; row < kConstraintRowCount; ++row)
        {
            const std::size_t index =
                static_cast<std::size_t>(row) * variables.size() + variable;
            result.finite_difference[index] =
                (plus_evaluation.constraint_values[static_cast<std::size_t>(row)] -
                 minus_evaluation.constraint_values[static_cast<std::size_t>(row)]) /
                (2.0 * step);
            const double scale = std::max(
                // 1e-3 floor: hinge-activation boundaries make entries below
                // this magnitude indistinguishable from finite-difference
                // noise through the long flatness chain.
                {1e-3,
                 std::abs(result.analytic[index]),
                 std::abs(result.finite_difference[index])});
            result.maximum_relative_error =
                std::max(result.maximum_relative_error,
                         std::abs(result.analytic[index] -
                                  result.finite_difference[index]) / scale);
        }
    }
    result.valid = values_valid &&
                   std::isfinite(result.maximum_relative_error) &&
                   std::all_of(result.analytic.begin(), result.analytic.end(),
                               [](double value) { return std::isfinite(value); }) &&
                   std::all_of(result.finite_difference.begin(),
                               result.finite_difference.end(),
                               [](double value) { return std::isfinite(value); });
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
