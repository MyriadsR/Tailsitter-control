#include "planning/actuator_penalty_optimizer.hpp"
#include "planning/feasibility_checker.hpp"
#include "planning/time_allocation.hpp"

#include <filesystem>
#include <fstream>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>

namespace
{

void writeCsv(const std::filesystem::path& path,
              const planning::FlatOutputTrajectory& trajectory,
              const planning::FeasibilityReport& report,
              const planning::VehicleParameters& parameters)
{
    if (!path.parent_path().empty())
        std::filesystem::create_directories(path.parent_path());
    std::ofstream stream(path);
    if (!stream)
        throw std::runtime_error("cannot open output CSV: " + path.string());
    stream << "t,px,py,pz,vx,vy,vz,ax,ay,az,jx,jy,jz,snap_x,snap_y,snap_z,"
              "yaw,yaw_rate,yaw_acceleration,omega1_pred,omega2_pred,delta1_pred,delta2_pred,"
              "collective_thrust,body_rate_x,body_rate_y,body_rate_z,body_rate_norm,"
              "rotor_rate_1,rotor_rate_2,"
              "flap_rate_1,flap_rate_2,omega_cmd_req_1,omega_cmd_req_2,"
              "delta_cmd_req_1,delta_cmd_req_2\n";
    stream << std::setprecision(12);
    bool previous_valid = false;
    planning::ActuatorPrediction previous_prediction;
    for (const auto& prediction : report.predictions)
    {
        const auto sample = trajectory.sample(prediction.time);
        stream << prediction.time;
        for (int i = 0; i < 3; ++i) stream << ',' << sample.position[i];
        for (int i = 0; i < 3; ++i) stream << ',' << sample.velocity[i];
        for (int i = 0; i < 3; ++i) stream << ',' << sample.acceleration[i];
        for (int i = 0; i < 3; ++i) stream << ',' << sample.jerk[i];
        for (int i = 0; i < 3; ++i) stream << ',' << sample.snap[i];
        stream << ',' << sample.yaw << ',' << sample.yaw_rate << ',' << sample.yaw_acceleration
               << ',' << prediction.rotor_speed_1 << ',' << prediction.rotor_speed_2
               << ',' << prediction.flap_1 << ',' << prediction.flap_2
               << ',' << prediction.collective_thrust;
        for (int axis = 0; axis < 3; ++axis)
            stream << ',' << prediction.angular_velocity[axis];
        stream << ',' << prediction.angular_velocity.norm();

        const double not_available = std::numeric_limits<double>::quiet_NaN();
        Eigen::Vector2d rotor_rate = Eigen::Vector2d::Constant(not_available);
        Eigen::Vector2d flap_rate = Eigen::Vector2d::Constant(not_available);
        Eigen::Vector2d rotor_command = Eigen::Vector2d::Constant(not_available);
        Eigen::Vector2d flap_command = Eigen::Vector2d::Constant(not_available);
        if (previous_valid && prediction.valid &&
            prediction.time > previous_prediction.time)
        {
            const double interval = prediction.time - previous_prediction.time;
            const Eigen::Vector2d previous_rotor(previous_prediction.rotor_speed_1,
                                                  previous_prediction.rotor_speed_2);
            const Eigen::Vector2d current_rotor(prediction.rotor_speed_1,
                                                 prediction.rotor_speed_2);
            const Eigen::Vector2d previous_flap(previous_prediction.flap_1,
                                                 previous_prediction.flap_2);
            const Eigen::Vector2d current_flap(prediction.flap_1, prediction.flap_2);
            rotor_rate = (current_rotor - previous_rotor) / interval;
            flap_rate = (current_flap - previous_flap) / interval;
            const double motor_decay =
                std::exp(-interval / parameters.motor_time_constant);
            const double servo_decay =
                std::exp(-interval / parameters.servo_time_constant);
            rotor_command =
                (current_rotor - motor_decay * previous_rotor) / (1.0 - motor_decay);
            flap_command =
                (current_flap - servo_decay * previous_flap) / (1.0 - servo_decay);
        }
        for (int actuator = 0; actuator < 2; ++actuator)
            stream << ',' << rotor_rate[actuator];
        for (int actuator = 0; actuator < 2; ++actuator)
            stream << ',' << flap_rate[actuator];
        for (int actuator = 0; actuator < 2; ++actuator)
            stream << ',' << rotor_command[actuator];
        for (int actuator = 0; actuator < 2; ++actuator)
            stream << ',' << flap_command[actuator];
        stream << '\n';
        previous_valid = prediction.valid;
        previous_prediction = prediction;
    }
}

double parseDouble(const char* text, const char* name)
{
    try
    {
        const double value = std::stod(text);
        if (!std::isfinite(value)) throw std::invalid_argument("non-finite");
        return value;
    }
    catch (const std::exception&)
    {
        throw std::invalid_argument(std::string("invalid value for ") + name);
    }
}

int parseInt(const char* text, const char* name)
{
    try
    {
        return std::stoi(text);
    }
    catch (const std::exception&)
    {
        throw std::invalid_argument(std::string("invalid value for ") + name);
    }
}

void printTerms(const char* label, const planning::StageBObjectiveTerms& terms)
{
    std::cout << label << " objective: " << terms.total
              << ", smoothness: " << terms.smoothness
              << ", normalized smoothness: " << terms.normalized_smoothness
              << ", geometry: " << terms.geometry << '\n'
              << label << " actuator integrals [T1,T2,d1,d2,velocity,collective,"
                 "body-rate,motor-dynamics,"
                 "flap-rate,servo-dynamics,invalid]: ["
              << terms.actuator.thrust_1 << ", " << terms.actuator.thrust_2 << ", "
              << terms.actuator.flap_1 << ", " << terms.actuator.flap_2 << ", "
              << terms.actuator.velocity << ", "
              << terms.actuator.collective_thrust << ", "
              << terms.actuator.body_rate << ", "
              << terms.actuator.motor_dynamics << ", "
              << terms.actuator.flap_rate << ", "
              << terms.actuator.servo_dynamics << ", "
              << terms.actuator.invalid_mapping << "]\n";
}

void printDurations(const char* label, const std::vector<double>& durations)
{
    std::cout << label << " durations: [";
    for (std::size_t i = 0; i < durations.size(); ++i)
        std::cout << (i == 0 ? "" : ", ") << durations[i];
    std::cout << "] s\n";
}

void printWaypoint(const char* label,
                   const planning::FlatOutputTrajectory::PositionMatrix& initial,
                   const planning::FlatOutputTrajectory::PositionMatrix& optimized,
                   int row)
{
    std::cout << label << ": [" << initial(row, 0) << ", " << initial(row, 1) << ", "
              << initial(row, 2) << "] -> [" << optimized(row, 0) << ", "
              << optimized(row, 1) << ", " << optimized(row, 2) << "]\n";
}

}  // namespace

int main(int argc, char** argv)
{
    try
    {
        bool yaw_pi = false;
        bool simulation_vehicle = false;
        double total_time = 1.8;
        double body_rate_max_override = std::numeric_limits<double>::quiet_NaN();
        double motor_time_constant_override = std::numeric_limits<double>::quiet_NaN();
        double servo_time_constant_override = std::numeric_limits<double>::quiet_NaN();
        double servo_rate_max_override = std::numeric_limits<double>::quiet_NaN();
        planning::ActuatorPenaltyConfig config;
        std::filesystem::path output = "output/stage_b_optimized.csv";
        std::filesystem::path baseline_output = "output/stage_b_baseline.csv";
        for (int i = 1; i < argc; ++i)
        {
            const std::string argument = argv[i];
            const auto next = [&](const char* name) {
                if (i + 1 >= argc)
                    throw std::invalid_argument(std::string("missing value for ") + name);
                return argv[++i];
            };
            if (argument == "--yaw-pi") yaw_pi = true;
            else if (argument == "--simulation-vehicle") simulation_vehicle = true;
            else if (argument == "--total-time") total_time = parseDouble(next("--total-time"), "--total-time");
            else if (argument == "--actuator-margin") config.actuator_margin = parseDouble(next("--actuator-margin"), "--actuator-margin");
            else if (argument == "--actuator-weight") config.actuator_weight = parseDouble(next("--actuator-weight"), "--actuator-weight");
            else if (argument == "--geometry-weight") config.geometry_weight = parseDouble(next("--geometry-weight"), "--geometry-weight");
            else if (argument == "--samples-per-segment") config.samples_per_segment = parseInt(next("--samples-per-segment"), "--samples-per-segment");
            else if (argument == "--maxeval") config.max_evaluations = parseInt(next("--maxeval"), "--maxeval");
            else if (argument == "--body-rate-max") body_rate_max_override = parseDouble(next("--body-rate-max"), "--body-rate-max");
            else if (argument == "--motor-time-constant") motor_time_constant_override = parseDouble(next("--motor-time-constant"), "--motor-time-constant");
            else if (argument == "--servo-time-constant") servo_time_constant_override = parseDouble(next("--servo-time-constant"), "--servo-time-constant");
            else if (argument == "--servo-rate-max") servo_rate_max_override = parseDouble(next("--servo-rate-max"), "--servo-rate-max");
            else if (argument == "--no-body-rate-constraint") config.enable_body_rate_constraint = false;
            else if (argument == "--no-actuator-dynamics") config.enable_actuator_dynamics = false;
            else if (argument == "--output") output = next("--output");
            else if (argument == "--baseline-output") baseline_output = next("--baseline-output");
            else
            {
                std::cerr << "usage: generate_stage_b_trajectory [--yaw-pi] [--simulation-vehicle] "
                             "[--total-time S] [--actuator-margin M] [--actuator-weight W] "
                             "[--geometry-weight W] [--samples-per-segment N] [--maxeval N] "
                             "[--body-rate-max RAD_S] [--motor-time-constant S] "
                             "[--servo-time-constant S] [--servo-rate-max RAD_S] "
                             "[--no-body-rate-constraint] [--no-actuator-dynamics] "
                             "[--output FILE] [--baseline-output FILE]\n";
                return 2;
            }
        }

        // Rows 0, 1 and 3 are fixed key waypoints. Row 2 is a newly inserted
        // support point and is the only geometry variable in this simple test.
        planning::FlatOutputTrajectory::PositionMatrix nominal_positions(4, 3);
        nominal_positions << 0.0, 0.0, -10.0,
                             1.5, 0.0, -10.0,
                             3.75, 0.0, -10.0,
                             6.0, 0.0, -10.0;
        planning::FlatOutputTrajectory::YawMatrix yaws(4, 1);
        yaws << 0.0,
                (yaw_pi ? 0.5 * 3.14159265358979323846 : 0.0),
                (yaw_pi ? 0.75 * 3.14159265358979323846 : 0.0),
                (yaw_pi ? 3.14159265358979323846 : 0.0);

        planning::FixedTotalTimeAllocator stage_a_allocator(config.yaw_weight);
        const auto stage_a = stage_a_allocator.optimize(total_time, nominal_positions, yaws);
        auto parameters = simulation_vehicle
                              ? planning::VehicleParameters::simulationVehicle()
                              : planning::VehicleParameters::paperVehicle();
        if (std::isfinite(body_rate_max_override))
            parameters.body_rate_max = body_rate_max_override;
        if (std::isfinite(motor_time_constant_override))
            parameters.motor_time_constant = motor_time_constant_override;
        if (std::isfinite(servo_time_constant_override))
            parameters.servo_time_constant = servo_time_constant_override;
        if (std::isfinite(servo_rate_max_override))
            parameters.servo_rate_max = servo_rate_max_override;
        planning::ActuatorPenaltyOptimizer optimizer(
            planning::TailsitterFlatness(parameters), config);
        const auto stage_b = optimizer.optimize(
            total_time, stage_a.durations, nominal_positions, yaws, {2});

        planning::FlatOutputTrajectory baseline_trajectory;
        baseline_trajectory.update(stage_a.durations, nominal_positions, yaws);
        planning::FlatOutputTrajectory optimized_trajectory;
        optimized_trajectory.update(stage_b.durations, stage_b.position_waypoints, yaws);
        planning::FeasibilityChecker checker(
            planning::TailsitterFlatness(parameters), 0.002);
        const auto baseline_report = checker.check(baseline_trajectory, true);
        const auto optimized_report = checker.check(optimized_trajectory, true);
        writeCsv(baseline_output, baseline_trajectory, baseline_report, parameters);
        writeCsv(output, optimized_trajectory, optimized_report, parameters);

        std::cout << std::fixed << std::setprecision(6)
                  << "stage B fixed total time: " << total_time << " s\n"
                  << "yaw case: " << (yaw_pi ? "0 -> pi" : "0 -> 0") << '\n'
                  << "actuator planning margin: " << config.actuator_margin << '\n'
                  << "actuator penalty weight: " << config.actuator_weight << '\n'
                  << "body-rate constraint: "
                  << (config.enable_body_rate_constraint ? "enabled" : "disabled")
                  << ", physical/reference limit: " << parameters.body_rate_max << " rad/s\n"
                  << "actuator dynamics: "
                  << (config.enable_actuator_dynamics ? "enabled" : "disabled")
                  << ", tau_motor/tau_servo: " << parameters.motor_time_constant << "/"
                  << parameters.servo_time_constant << " s, flap-rate limit: "
                  << parameters.servo_rate_max << " rad/s\n"
                  << "evaluations: " << stage_b.evaluations << '\n';
        printDurations("stage A", stage_a.durations);
        printDurations("stage B", stage_b.durations);
        printWaypoint("fixed key waypoint", nominal_positions, stage_b.position_waypoints, 1);
        printWaypoint("auxiliary waypoint", nominal_positions, stage_b.position_waypoints, 2);
        printTerms("initial", stage_b.initial_terms);
        printTerms("final", stage_b.final_terms);
        std::cout << "baseline feasible at physical limits: "
                  << (baseline_report.feasible ? "yes" : "no")
                  << ", peak omega: " << baseline_report.peak_rotor_speed
                  << ", peak flap: " << baseline_report.peak_flap_deflection
                  << ", peak speed/collective: " << baseline_report.peak_velocity
                  << "/" << baseline_report.peak_collective_thrust
                  << ", peak body rate: " << baseline_report.peak_body_rate
                  << ", peak flap rate: " << baseline_report.peak_flap_rate
                  << ", peak required rotor/flap command: "
                  << baseline_report.peak_required_rotor_command << "/"
                  << baseline_report.peak_required_flap_command << '\n'
                  << "stage B feasible at physical limits: "
                  << (optimized_report.feasible ? "yes" : "no")
                  << ", peak omega: " << optimized_report.peak_rotor_speed
                  << ", peak flap: " << optimized_report.peak_flap_deflection
                  << ", peak speed/collective: " << optimized_report.peak_velocity
                  << "/" << optimized_report.peak_collective_thrust
                  << ", peak body rate: " << optimized_report.peak_body_rate
                  << ", peak flap rate: " << optimized_report.peak_flap_rate
                  << ", peak required rotor/flap command: "
                  << optimized_report.peak_required_rotor_command << "/"
                  << optimized_report.peak_required_flap_command << '\n'
                  << "baseline CSV: " << std::filesystem::absolute(baseline_output) << '\n'
                  << "stage B CSV: " << std::filesystem::absolute(output) << '\n';
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}
