#include "planning/feasibility_checker.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace planning
{

FeasibilityChecker::FeasibilityChecker(TailsitterFlatness flatness,
                                       double sample_interval,
                                       double actuator_margin,
                                       bool enforce_velocity_constraint)
    : flatness_(std::move(flatness)),
      sample_interval_(sample_interval),
      actuator_margin_(actuator_margin),
      enforce_velocity_constraint_(enforce_velocity_constraint)
{
    if (!(sample_interval > 0.0) || !(actuator_margin > 0.0 && actuator_margin <= 1.0))
        throw std::invalid_argument("invalid feasibility sampling interval or actuator margin");
    const auto& parameters = flatness_.parameters();
    if (!(parameters.body_rate_max > 0.0) ||
        (enforce_velocity_constraint_ && !(parameters.velocity_max > 0.0)) ||
        !(parameters.motor_time_constant > 0.0) ||
        !(parameters.servo_time_constant > 0.0) ||
        !(parameters.servo_rate_max > 0.0))
    {
        throw std::invalid_argument("invalid body-rate or actuator dynamics parameters");
    }
}

FeasibilityReport FeasibilityChecker::check(const FlatOutputTrajectory& trajectory,
                                            bool keep_predictions) const
{
    std::vector<double> times;
    const int uniform_count = std::max(1, static_cast<int>(std::ceil(trajectory.duration() / sample_interval_)));
    times.reserve(static_cast<std::size_t>(uniform_count + trajectory.durations().size() + 1));
    for (int i = 0; i <= uniform_count; ++i)
        times.push_back(trajectory.duration() * static_cast<double>(i) / uniform_count);
    double knot = 0.0;
    for (double duration : trajectory.durations())
    {
        knot += duration;
        times.push_back(knot);
    }
    std::sort(times.begin(), times.end());
    times.erase(std::unique(times.begin(), times.end(), [](double a, double b) {
                    return std::abs(a - b) < 1e-12;
                }),
                times.end());

    FeasibilityReport report;
    report.feasible = true;
    BranchState branch;
    const auto& parameters = flatness_.parameters();
    const double omega_upper = parameters.omega_max * actuator_margin_;
    const double delta_upper = parameters.delta_max * actuator_margin_;
    const double body_rate_upper = parameters.body_rate_max * actuator_margin_;
    const double velocity_upper = parameters.velocity_max * actuator_margin_;
    const double collective_thrust_lower =
        2.0 * parameters.c_T * parameters.omega_min * parameters.omega_min;
    const double collective_thrust_upper =
        2.0 * parameters.c_T * omega_upper * omega_upper;
    const double collective_thrust_scale = std::max(
        1e-9, collective_thrust_upper - collective_thrust_lower);
    const double flap_rate_upper = parameters.servo_rate_max * actuator_margin_;
    if (keep_predictions)
        report.predictions.reserve(times.size());

    bool previous_valid = false;
    double previous_time = 0.0;
    ActuatorPrediction previous_prediction;
    for (double time : times)
    {
        const FlatOutputSample sample = trajectory.sample(time);
        ActuatorPrediction prediction = flatness_.forward(sample, branch);
        if (prediction.valid)
        {
            bool branch_continuous = true;
            if (branch.initialized)
            {
                const double quaternion_dot = std::clamp(
                    std::abs(branch.attitude.dot(prediction.attitude)), 0.0, 1.0);
                const double attitude_step = 2.0 * std::acos(quaternion_dot);
                // With millisecond-level sampling, a half-radian step is a branch
                // jump, not a physically continuous attitude trajectory.
                branch_continuous = attitude_step <= 0.5;
            }
            branch.initialized = true;
            branch.attitude = prediction.attitude;
            report.peak_rotor_speed = std::max({report.peak_rotor_speed,
                                                prediction.rotor_speed_1,
                                                prediction.rotor_speed_2});
            report.peak_flap_deflection = std::max({report.peak_flap_deflection,
                                                    std::abs(prediction.flap_1),
                                                    std::abs(prediction.flap_2)});
            report.peak_body_rate = std::max(
                report.peak_body_rate, prediction.angular_velocity.norm());
            report.peak_velocity = std::max(report.peak_velocity, sample.velocity.norm());
            report.peak_collective_thrust = std::max(
                report.peak_collective_thrust, prediction.collective_thrust);
            double violation = std::max({
                (parameters.omega_min - prediction.rotor_speed_1) /
                    std::max(1.0, parameters.omega_max - parameters.omega_min),
                (parameters.omega_min - prediction.rotor_speed_2) /
                    std::max(1.0, parameters.omega_max - parameters.omega_min),
                prediction.rotor_speed_1 / omega_upper - 1.0,
                prediction.rotor_speed_2 / omega_upper - 1.0,
                std::abs(prediction.flap_1) / delta_upper - 1.0,
                std::abs(prediction.flap_2) / delta_upper - 1.0,
                prediction.angular_velocity.norm() / body_rate_upper - 1.0,
                enforce_velocity_constraint_
                    ? sample.velocity.norm() / velocity_upper - 1.0
                    : -std::numeric_limits<double>::infinity(),
                (collective_thrust_lower - prediction.collective_thrust) /
                    collective_thrust_scale,
                (prediction.collective_thrust - collective_thrust_upper) /
                    collective_thrust_scale});

            if (previous_valid)
            {
                const double interval = time - previous_time;
                if (interval > 0.0)
                {
                    const Eigen::Vector2d previous_rotor(
                        previous_prediction.rotor_speed_1,
                        previous_prediction.rotor_speed_2);
                    const Eigen::Vector2d current_rotor(
                        prediction.rotor_speed_1,
                        prediction.rotor_speed_2);
                    const Eigen::Vector2d previous_flap(
                        previous_prediction.flap_1,
                        previous_prediction.flap_2);
                    const Eigen::Vector2d current_flap(
                        prediction.flap_1,
                        prediction.flap_2);
                    const Eigen::Vector2d flap_rate =
                        (current_flap - previous_flap) / interval;
                    const double motor_decay =
                        std::exp(-interval / parameters.motor_time_constant);
                    const double servo_decay =
                        std::exp(-interval / parameters.servo_time_constant);
                    const Eigen::Vector2d required_rotor_command =
                        (current_rotor - motor_decay * previous_rotor) /
                        (1.0 - motor_decay);
                    const Eigen::Vector2d required_flap_command =
                        (current_flap - servo_decay * previous_flap) /
                        (1.0 - servo_decay);

                    report.peak_flap_rate = std::max(
                        report.peak_flap_rate, flap_rate.cwiseAbs().maxCoeff());
                    report.peak_required_rotor_command = std::max(
                        report.peak_required_rotor_command,
                        required_rotor_command.cwiseAbs().maxCoeff());
                    report.peak_required_flap_command = std::max(
                        report.peak_required_flap_command,
                        required_flap_command.cwiseAbs().maxCoeff());
                    violation = std::max({
                        violation,
                        (parameters.omega_min - required_rotor_command[0]) /
                            std::max(1.0, parameters.omega_max - parameters.omega_min),
                        (parameters.omega_min - required_rotor_command[1]) /
                            std::max(1.0, parameters.omega_max - parameters.omega_min),
                        required_rotor_command[0] / omega_upper - 1.0,
                        required_rotor_command[1] / omega_upper - 1.0,
                        std::abs(flap_rate[0]) / flap_rate_upper - 1.0,
                        std::abs(flap_rate[1]) / flap_rate_upper - 1.0,
                        std::abs(required_flap_command[0]) / delta_upper - 1.0,
                        std::abs(required_flap_command[1]) / delta_upper - 1.0});
                }
            }
            report.max_normalized_violation = std::max(report.max_normalized_violation, violation);
            if (!branch_continuous && report.feasible)
            {
                report.feasible = false;
                report.max_normalized_violation = std::numeric_limits<double>::infinity();
                report.first_failure_time = time;
                report.first_failure_reason = "attitude branch discontinuity";
            }
            else if (violation > 0.0 && report.feasible)
            {
                report.feasible = false;
                report.first_failure_time = time;
                report.first_failure_reason =
                    "high-level or actuator amplitude/rate/dynamic limit exceeded";
            }
            previous_valid = true;
            previous_time = time;
            previous_prediction = prediction;
        }
        else
        {
            previous_valid = false;
            if (report.feasible)
            {
                report.feasible = false;
                report.max_normalized_violation = std::numeric_limits<double>::infinity();
                report.first_failure_time = time;
                report.first_failure_reason = prediction.failure_reason;
            }
        }
        if (keep_predictions)
            report.predictions.push_back(std::move(prediction));
    }
    return report;
}

}  // namespace planning
