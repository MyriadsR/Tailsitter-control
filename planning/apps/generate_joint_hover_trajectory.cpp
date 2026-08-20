#include "planning/actuator_penalty_optimizer.hpp"
#include "planning/feasibility_checker.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>

namespace
{

double parseDouble(const char* text, const char* name)
{
    try
    {
        const double value = std::stod(text);
        if (!std::isfinite(value))
            throw std::invalid_argument("non-finite");
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
              "rotor_rate_1,rotor_rate_2,flap_rate_1,flap_rate_2,"
              "omega_cmd_req_1,omega_cmd_req_2,delta_cmd_req_1,delta_cmd_req_2\n";
    stream << std::setprecision(12);

    bool previous_valid = false;
    planning::ActuatorPrediction previous_prediction;
    for (const auto& prediction : report.predictions)
    {
        const auto sample = trajectory.sample(prediction.time);
        stream << prediction.time;
        for (int axis = 0; axis < 3; ++axis) stream << ',' << sample.position[axis];
        for (int axis = 0; axis < 3; ++axis) stream << ',' << sample.velocity[axis];
        for (int axis = 0; axis < 3; ++axis) stream << ',' << sample.acceleration[axis];
        for (int axis = 0; axis < 3; ++axis) stream << ',' << sample.jerk[axis];
        for (int axis = 0; axis < 3; ++axis) stream << ',' << sample.snap[axis];
        stream << ',' << sample.yaw << ',' << sample.yaw_rate << ','
               << sample.yaw_acceleration << ',' << prediction.rotor_speed_1 << ','
               << prediction.rotor_speed_2 << ',' << prediction.flap_1 << ','
               << prediction.flap_2 << ',' << prediction.collective_thrust;
        for (int axis = 0; axis < 3; ++axis)
            stream << ',' << prediction.angular_velocity[axis];
        stream << ',' << prediction.angular_velocity.norm();

        const double nan = std::numeric_limits<double>::quiet_NaN();
        Eigen::Vector2d rotor_rate = Eigen::Vector2d::Constant(nan);
        Eigen::Vector2d flap_rate = Eigen::Vector2d::Constant(nan);
        Eigen::Vector2d rotor_command = Eigen::Vector2d::Constant(nan);
        Eigen::Vector2d flap_command = Eigen::Vector2d::Constant(nan);
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

void printDurations(const char* label, const std::vector<double>& durations)
{
    std::cout << label << " durations: [";
    for (std::size_t segment = 0; segment < durations.size(); ++segment)
        std::cout << (segment == 0 ? "" : ", ") << durations[segment];
    std::cout << "] s, total="
              << std::accumulate(durations.begin(), durations.end(), 0.0) << " s\n";
}

void printTerms(const char* label, const planning::StageBObjectiveTerms& terms)
{
    std::cout << label << " objective=" << terms.total
              << ", smoothness(normalized)=" << terms.normalized_smoothness
              << ", time=" << terms.time << ", geometry=" << terms.geometry << '\n'
              << label << " penalties [velocity,collective,body-rate,T1,T2,d1,d2,"
                 "motor-dyn,flap-rate,servo-dyn,invalid]=["
              << terms.actuator.velocity << ", "
              << terms.actuator.collective_thrust << ", "
              << terms.actuator.body_rate << ", "
              << terms.actuator.thrust_1 << ", "
              << terms.actuator.thrust_2 << ", "
              << terms.actuator.flap_1 << ", "
              << terms.actuator.flap_2 << ", "
              << terms.actuator.motor_dynamics << ", "
              << terms.actuator.flap_rate << ", "
              << terms.actuator.servo_dynamics << ", "
              << terms.actuator.invalid_mapping << "]\n";
}

}  // namespace

int main(int argc, char** argv)
{
    try
    {
        bool yaw_pi = false;
        bool simulation_vehicle = false;
        double distance = 15.0;
        double initial_total_time = 4.0;
        planning::ActuatorPenaltyConfig config;
        config.enable_high_level_constraints = true;
        config.auxiliary_half_range = Eigen::Vector3d(2.0, 3.0, 2.0);
        config.geometry_weight = 0.02;
        config.max_evaluations = 600;
        std::filesystem::path output = "output/joint_hover_optimized.csv";
        std::filesystem::path baseline_output = "output/joint_hover_initial.csv";

        for (int index = 1; index < argc; ++index)
        {
            const std::string argument = argv[index];
            const auto next = [&](const char* name) {
                if (index + 1 >= argc)
                    throw std::invalid_argument(std::string("missing value for ") + name);
                return argv[++index];
            };
            if (argument == "--yaw-pi") yaw_pi = true;
            else if (argument == "--simulation-vehicle") simulation_vehicle = true;
            else if (argument == "--distance") distance = parseDouble(next("--distance"), "--distance");
            else if (argument == "--initial-total-time") initial_total_time = parseDouble(next("--initial-total-time"), "--initial-total-time");
            else if (argument == "--time-weight") config.time_weight = parseDouble(next("--time-weight"), "--time-weight");
            else if (argument == "--actuator-margin") config.actuator_margin = parseDouble(next("--actuator-margin"), "--actuator-margin");
            else if (argument == "--actuator-weight") config.actuator_weight = parseDouble(next("--actuator-weight"), "--actuator-weight");
            else if (argument == "--geometry-weight") config.geometry_weight = parseDouble(next("--geometry-weight"), "--geometry-weight");
            else if (argument == "--free-time-log-bound") config.free_time_log_bound = parseDouble(next("--free-time-log-bound"), "--free-time-log-bound");
            else if (argument == "--samples-per-segment") config.samples_per_segment = parseInt(next("--samples-per-segment"), "--samples-per-segment");
            else if (argument == "--maxeval") config.max_evaluations = parseInt(next("--maxeval"), "--maxeval");
            else if (argument == "--output") output = next("--output");
            else if (argument == "--baseline-output") baseline_output = next("--baseline-output");
            else
            {
                std::cerr << "usage: generate_joint_hover_trajectory [--yaw-pi] "
                             "[--simulation-vehicle] [--distance M] "
                             "[--initial-total-time S] [--time-weight W] "
                             "[--actuator-margin M] [--actuator-weight W] "
                             "[--geometry-weight W] [--free-time-log-bound B] "
                             "[--samples-per-segment N] [--maxeval N] "
                             "[--output FILE] [--baseline-output FILE]\n";
                return 2;
            }
        }
        if (!(distance > 0.0) || !(initial_total_time > 0.0))
            throw std::invalid_argument("distance and initial total time must be positive");

        constexpr int waypoint_count = 5;
        constexpr int segment_count = waypoint_count - 1;
        planning::FlatOutputTrajectory::PositionMatrix nominal_positions(
            waypoint_count, 3);
        planning::FlatOutputTrajectory::YawMatrix yaws(waypoint_count, 1);
        for (int waypoint = 0; waypoint < waypoint_count; ++waypoint)
        {
            const double fraction =
                static_cast<double>(waypoint) / static_cast<double>(segment_count);
            nominal_positions.row(waypoint) << distance * fraction, 0.0, -10.0;
            yaws(waypoint, 0) = yaw_pi ? fraction * 3.14159265358979323846 : 0.0;
        }
        const std::vector<double> initial_durations(
            segment_count, initial_total_time / segment_count);
        const std::vector<int> free_waypoints{1, 2, 3};
        auto parameters = simulation_vehicle
                              ? planning::VehicleParameters::simulationVehicle()
                              : planning::VehicleParameters::paperVehicle();
        planning::ActuatorPenaltyOptimizer optimizer(
            planning::TailsitterFlatness(parameters), config);
        const auto result = optimizer.optimizeJoint(
            initial_durations, nominal_positions, yaws, free_waypoints);

        planning::FlatOutputTrajectory initial_trajectory;
        initial_trajectory.update(initial_durations, nominal_positions, yaws);
        planning::FlatOutputTrajectory optimized_trajectory;
        optimized_trajectory.update(result.durations, result.position_waypoints, yaws);
        planning::FeasibilityChecker checker(
            planning::TailsitterFlatness(parameters), 0.002);
        const auto initial_report = checker.check(initial_trajectory, true);
        const auto optimized_report = checker.check(optimized_trajectory, true);
        writeCsv(baseline_output, initial_trajectory, initial_report, parameters);
        writeCsv(output, optimized_trajectory, optimized_report, parameters);

        std::cout << std::fixed << std::setprecision(6)
                  << "test 3 joint hover-to-hover optimization\n"
                  << "yaw case: " << (yaw_pi ? "0 -> pi" : "0 -> 0") << '\n'
                  << "distance: " << distance << " m, variables: 13 "
                     "(4 log-durations + 3 free 3D waypoints)\n"
                  << "fixed total time: no, safety corridor: no\n"
                  << "time weight: " << config.time_weight
                  << ", actuator weight: " << config.actuator_weight
                  << ", margin: " << config.actuator_margin << '\n'
                  << "evaluations: " << result.evaluations << '\n';
        printDurations("initial", initial_durations);
        printDurations("optimized", result.durations);
        for (int waypoint : free_waypoints)
        {
            std::cout << "waypoint " << waypoint << ": ["
                      << nominal_positions(waypoint, 0) << ", "
                      << nominal_positions(waypoint, 1) << ", "
                      << nominal_positions(waypoint, 2) << "] -> ["
                      << result.position_waypoints(waypoint, 0) << ", "
                      << result.position_waypoints(waypoint, 1) << ", "
                      << result.position_waypoints(waypoint, 2) << "]\n";
        }
        printTerms("initial", result.initial_terms);
        printTerms("optimized", result.final_terms);
        const auto print_report = [](const char* label,
                                     const planning::FeasibilityReport& report) {
            std::cout << label << " full-envelope feasible: "
                      << (report.feasible ? "yes" : "no")
                      << ", speed=" << report.peak_velocity
                      << ", collective thrust=" << report.peak_collective_thrust
                      << ", body rate=" << report.peak_body_rate
                      << ", rotor=" << report.peak_rotor_speed
                      << ", flap=" << report.peak_flap_deflection
                      << ", flap rate=" << report.peak_flap_rate
                      << ", required rotor/flap command="
                      << report.peak_required_rotor_command << "/"
                      << report.peak_required_flap_command << '\n';
        };
        print_report("initial", initial_report);
        print_report("optimized", optimized_report);
        std::cout << "initial CSV: " << std::filesystem::absolute(baseline_output) << '\n'
                  << "optimized CSV: " << std::filesystem::absolute(output) << '\n';
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}
