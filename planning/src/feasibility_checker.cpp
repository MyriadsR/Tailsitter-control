#include "planning/feasibility_checker.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace planning
{

FeasibilityChecker::FeasibilityChecker(TailsitterFlatness flatness,
                                       double sample_interval,
                                       double actuator_margin)
    : flatness_(std::move(flatness)),
      sample_interval_(sample_interval),
      actuator_margin_(actuator_margin)
{
    if (!(sample_interval > 0.0) || !(actuator_margin > 0.0 && actuator_margin <= 1.0))
        throw std::invalid_argument("invalid feasibility sampling interval or actuator margin");
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
    if (keep_predictions)
        report.predictions.reserve(times.size());

    for (double time : times)
    {
        ActuatorPrediction prediction = flatness_.forward(trajectory.sample(time), branch);
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
            const double violation = std::max({
                (parameters.omega_min - prediction.rotor_speed_1) /
                    std::max(1.0, parameters.omega_max - parameters.omega_min),
                (parameters.omega_min - prediction.rotor_speed_2) /
                    std::max(1.0, parameters.omega_max - parameters.omega_min),
                prediction.rotor_speed_1 / omega_upper - 1.0,
                prediction.rotor_speed_2 / omega_upper - 1.0,
                std::abs(prediction.flap_1) / delta_upper - 1.0,
                std::abs(prediction.flap_2) / delta_upper - 1.0});
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
                report.first_failure_reason = "raw actuator limit exceeded";
            }
        }
        else if (report.feasible)
        {
            report.feasible = false;
            report.max_normalized_violation = std::numeric_limits<double>::infinity();
            report.first_failure_time = time;
            report.first_failure_reason = prediction.failure_reason;
        }
        if (keep_predictions)
            report.predictions.push_back(std::move(prediction));
    }
    return report;
}

}  // namespace planning
