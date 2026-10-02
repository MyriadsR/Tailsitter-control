#include "planning/time_allocation.hpp"
#include "planning/time_scaler.hpp"

#include <cmath>
#include <iostream>
#include <numeric>

namespace
{
bool near(double a, double b, double tolerance) { return std::abs(a - b) <= tolerance; }

int fail(const char* message)
{
    std::cerr << "FAIL: " << message << '\n';
    return 1;
}
}  // namespace

int main()
{
    planning::FlatOutputTrajectory::PositionMatrix positions(3, 3);
    positions << 0.0, 0.0, -10.0,
                 1.5, 0.0, -10.0,
                 6.0, 0.0, -10.0;
    planning::FlatOutputTrajectory::YawMatrix yaws(3, 1);
    yaws.setZero();

    planning::FixedTotalTimeAllocator allocator;

    // Verify the combined position/yaw analytic duration gradient after the
    // fixed-total-time softmax chain rule.
    planning::FlatOutputTrajectory::YawMatrix gradient_yaws(3, 1);
    gradient_yaws << 0.0, 0.2, 0.5;
    const std::vector<double> tau{0.2};
    const auto tau_durations = planning::FixedTotalTimeAllocator::durationsFromTau(4.0, tau);
    planning::FlatOutputTrajectory gradient_trajectory;
    gradient_trajectory.update(tau_durations, positions, gradient_yaws);
    const Eigen::VectorXd grad_t = gradient_trajectory.energyTimeGradient(1.0);
    const double weighted_mean = (tau_durations[0] * grad_t[0] + tau_durations[1] * grad_t[1]) / 4.0;
    const double analytic_tau_gradient = tau_durations[0] * (grad_t[0] - weighted_mean);
    constexpr double finite_difference_step = 1e-6;
    auto energy_at_tau = [&](double value) {
        planning::FlatOutputTrajectory candidate;
        candidate.update(planning::FixedTotalTimeAllocator::durationsFromTau(4.0, {value}),
                         positions, gradient_yaws);
        return candidate.energy(1.0);
    };
    const double numeric_tau_gradient =
        (energy_at_tau(tau[0] + finite_difference_step) -
         energy_at_tau(tau[0] - finite_difference_step)) /
        (2.0 * finite_difference_step);
    if (std::abs(analytic_tau_gradient - numeric_tau_gradient) >
        2e-5 * std::max(1.0, std::abs(numeric_tau_gradient)))
        return fail("combined energy time gradient failed finite difference");

    const auto allocation = allocator.optimize(4.0, positions, yaws);
    if (!near(std::accumulate(allocation.durations.begin(), allocation.durations.end(), 0.0), 4.0, 1e-12))
        return fail("softmax durations do not preserve total time");
    if (allocation.final_cost > allocation.initial_cost * (1.0 + 1e-8))
        return fail("time allocation increased smoothness energy");

    // Test 3 control: fixed 15 m straight line, five fixed waypoints and four
    // softmax durations. Only the time ratio is optimized; no waypoint or
    // actuator-penalty variable enters this Stage A objective.
    planning::FlatOutputTrajectory::PositionMatrix straight_positions(5, 3);
    planning::FlatOutputTrajectory::YawMatrix straight_yaws(5, 1);
    for (int waypoint = 0; waypoint < 5; ++waypoint)
    {
        const double fraction = static_cast<double>(waypoint) / 4.0;
        straight_positions.row(waypoint) << 15.0 * fraction, 0.0, -10.0;
        straight_yaws(waypoint, 0) = fraction * 3.14159265358979323846;
    }
    const auto straight_allocation = allocator.optimize(
        4.0, straight_positions, straight_yaws);
    if (straight_allocation.durations.size() != 4 ||
        !near(std::accumulate(straight_allocation.durations.begin(),
                              straight_allocation.durations.end(), 0.0),
              4.0, 1e-12))
        return fail("five-waypoint softmax durations do not preserve total time");
    if (straight_allocation.final_cost >
        straight_allocation.initial_cost * (1.0 + 1e-8))
        return fail("five-waypoint time allocation increased smoothness energy");

    planning::FlatOutputTrajectory trajectory;
    trajectory.update(allocation.durations, positions, yaws);
    const auto start = trajectory.sample(0.0);
    const auto end = trajectory.sample(trajectory.duration());
    if ((start.position - positions.row(0).transpose()).norm() > 1e-9 ||
        (end.position - positions.row(2).transpose()).norm() > 1e-8)
        return fail("position endpoint mismatch");
    if (start.velocity.norm() > 1e-8 || end.velocity.norm() > 1e-8 ||
        start.acceleration.norm() > 1e-8 || end.acceleration.norm() > 1e-8 ||
        start.jerk.norm() > 1e-8 || end.jerk.norm() > 1e-7)
        return fail("septic endpoint derivatives are not clamped");

    planning::FlatOutputSample hover;
    hover.position = Eigen::Vector3d(0.0, 0.0, -10.0);
    const auto hover_prediction = planning::TailsitterFlatness(
        planning::VehicleParameters::paperVehicle()).forward(hover);
    if (!hover_prediction.valid)
        return fail("open-loop flatness is invalid at regularized hover");
    if (!(hover_prediction.thrust_1 >= 0.0 && hover_prediction.thrust_2 >= 0.0))
        return fail("hover produced negative motor thrust");
    const auto parameters = planning::VehicleParameters::paperVehicle();
    const double alpha_bar = parameters.alpha_0 + parameters.alpha_T;
    const double ca0 = std::cos(parameters.alpha_0);
    const double nu_1 = -parameters.c_LT_delta * std::cos(alpha_bar) * hover_prediction.thrust_1;
    const double nu_2 = -parameters.c_LT_delta * std::cos(alpha_bar) * hover_prediction.thrust_2;
    const Eigen::Vector2d reconstructed_flap_moment(
        parameters.l_delta_y * ca0 *
            (nu_2 * hover_prediction.flap_2 - nu_1 * hover_prediction.flap_1),
        parameters.l_delta_x *
            (nu_1 * hover_prediction.flap_1 + nu_2 * hover_prediction.flap_2));
    const double cab = std::cos(alpha_bar), sab = std::sin(alpha_bar);
    const double thrust_b_z = std::sin(parameters.alpha_0) * cab * (1.0 - parameters.c_DT) +
                              ca0 * sab * (parameters.c_LT - 1.0);
    const Eigen::Vector2d non_flap_moment(
        parameters.l_Ty * thrust_b_z *
            (hover_prediction.thrust_2 - hover_prediction.thrust_1) +
            std::cos(parameters.alpha_T) * (parameters.c_mu / parameters.c_T) *
                (hover_prediction.thrust_1 - hover_prediction.thrust_2),
        parameters.c_mu_T * hover_prediction.collective_thrust);
    if ((non_flap_moment + reconstructed_flap_moment - hover_prediction.moment.head<2>()).norm() > 1e-9)
        return fail("raw actuator outputs do not reconstruct commanded roll/pitch moment");

    const auto mid_sample = trajectory.sample(0.5 * trajectory.duration());
    const auto mid_prediction = planning::TailsitterFlatness(parameters).forward(mid_sample);
    if (!mid_prediction.valid)
        return fail("open-loop flatness failed at trajectory midpoint");
    const Eigen::Matrix3d body_to_alpha = (Eigen::Matrix3d() <<
        ca0, 0.0, std::sin(parameters.alpha_0),
        0.0, 1.0, 0.0,
        -std::sin(parameters.alpha_0), 0.0, ca0).finished();
    const Eigen::Vector3d velocity_body = mid_prediction.attitude.inverse() * mid_sample.velocity;
    const Eigen::Vector3d velocity_alpha = body_to_alpha * velocity_body;
    const Eigen::Vector3d force_alpha(
        cab * (1.0 - parameters.c_DT) * mid_prediction.collective_thrust -
            parameters.c_DV * mid_sample.velocity.norm() * velocity_alpha.x(),
        0.0,
        sab * (parameters.c_LT - 1.0) * mid_prediction.collective_thrust -
            parameters.c_LV * mid_sample.velocity.norm() * velocity_alpha.z());
    const Eigen::Vector3d reconstructed_world_force =
        mid_prediction.attitude * body_to_alpha.transpose() * force_alpha;
    const Eigen::Vector3d required_world_force =
        parameters.mass * (mid_sample.acceleration - Eigen::Vector3d(0.0, 0.0, parameters.gravity));
    if ((reconstructed_world_force - required_world_force).norm() > 1e-6)
        return fail("open-loop attitude/thrust does not reconstruct required force");

    planning::FeasibilityChecker checker(
        planning::TailsitterFlatness(planning::VehicleParameters::paperVehicle()), 0.005);
    planning::TimeScaler scaler(checker);
    const auto scaled = scaler.search(positions, yaws, allocation.durations, 0.25, 3.0, 17, 2e-3);
    if (!scaled.found || !scaled.report.feasible)
        return fail("outer time scaling found no feasible simple trajectory");
    if (scaled.report.predictions.empty())
        return fail("final feasibility report has no dense predictions");
    double minimum_adjacent_attitude_dot = 1.0;
    for (std::size_t i = 1; i < scaled.report.predictions.size(); ++i)
    {
        const double adjacent_dot = std::abs(scaled.report.predictions[i - 1].attitude.dot(
            scaled.report.predictions[i].attitude));
        minimum_adjacent_attitude_dot = std::min(minimum_adjacent_attitude_dot, adjacent_dot);
        if (!scaled.report.predictions[i].valid)
            return fail("attitude branch is invalid or discontinuous");
    }
    if (minimum_adjacent_attitude_dot < 0.95)
        return fail("attitude branch is invalid or discontinuous");
    std::vector<double> shortened_durations = allocation.durations;
    for (double& duration : shortened_durations)
        duration *= 0.98 * scaled.minimum_feasible_scale;
    planning::FlatOutputTrajectory shortened;
    shortened.update(shortened_durations, positions, yaws);
    if (checker.check(shortened).feasible)
        return fail("trajectory just below the actuator boundary remained feasible");

    std::cout << "stage A test passed; scale=" << scaled.minimum_feasible_scale
              << ", peak omega=" << scaled.report.peak_rotor_speed
              << ", peak flap=" << scaled.report.peak_flap_deflection << '\n';
    return 0;
}
