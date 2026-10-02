#include "planning/time_allocation.hpp"
#include "planning/time_scaler.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>
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
    stream << std::setprecision(17);

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
            const Eigen::Vector2d current_flap(prediction.flap_1,
                                                prediction.flap_2);
            rotor_rate = (current_rotor - previous_rotor) / interval;
            flap_rate = (current_flap - previous_flap) / interval;
            const double motor_decay =
                std::exp(-interval / parameters.motor_time_constant);
            const double servo_decay =
                std::exp(-interval / parameters.servo_time_constant);
            rotor_command =
                (current_rotor - motor_decay * previous_rotor) /
                (1.0 - motor_decay);
            flap_command =
                (current_flap - servo_decay * previous_flap) /
                (1.0 - servo_decay);
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
}  // namespace

int main(int argc, char** argv)
{
    try
    {
        bool yaw_pi = false;
        bool simulation_vehicle = false;
        bool optimize_time_ratio = true;
        bool scale_time = true;
        bool enforce_velocity_constraint = true;
        bool custom_straight_geometry = false;
        double distance = 6.0;
        double fixed_total_time = 4.0;
        int waypoint_count = 3;
        std::filesystem::path output = "output/simple_trajectory.csv";
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
            else if (argument == "--no-optimize-time-ratio") optimize_time_ratio = false;
            else if (argument == "--no-time-scaling") scale_time = false;
            else if (argument == "--no-velocity-constraint")
                enforce_velocity_constraint = false;
            else if (argument == "--distance")
            {
                distance = parseDouble(next("--distance"), "--distance");
                custom_straight_geometry = true;
            }
            else if (argument == "--waypoint-count")
            {
                waypoint_count = parseInt(next("--waypoint-count"), "--waypoint-count");
                custom_straight_geometry = true;
            }
            else if (argument == "--total-time")
                fixed_total_time = parseDouble(next("--total-time"), "--total-time");
            else if (argument == "--output") output = next("--output");
            else
            {
                std::cerr << "usage: generate_simple_trajectory [--yaw-pi] [--simulation-vehicle] "
                             "[--no-optimize-time-ratio] [--no-time-scaling] "
                             "[--no-velocity-constraint] "
                             "[--distance M] [--waypoint-count N] [--total-time S] "
                             "[--output FILE]\n";
                return 2;
            }
        }
        if (!(distance > 0.0) || !(fixed_total_time > 0.0) || waypoint_count < 2)
            throw std::invalid_argument(
                "distance and total time must be positive; waypoint count must be >= 2");

        planning::FlatOutputTrajectory::PositionMatrix positions(waypoint_count, 3);
        planning::FlatOutputTrajectory::YawMatrix yaws(waypoint_count, 1);
        if (custom_straight_geometry)
        {
            for (int waypoint = 0; waypoint < waypoint_count; ++waypoint)
            {
                const double fraction = static_cast<double>(waypoint) /
                                        static_cast<double>(waypoint_count - 1);
                positions.row(waypoint) << distance * fraction, 0.0, -10.0;
                yaws(waypoint, 0) =
                    yaw_pi ? fraction * 3.14159265358979323846 : 0.0;
            }
        }
        else
        {
            // Preserve the original three-waypoint Stage A experiment.
            positions << 0.0, 0.0, -10.0,
                         1.5, 0.0, -10.0,
                         6.0, 0.0, -10.0;
            yaws << 0.0, (yaw_pi ? 0.5 * 3.14159265358979323846 : 0.0),
                    (yaw_pi ? 3.14159265358979323846 : 0.0);
        }

        planning::FixedTotalTimeAllocator allocator(1.0);
        planning::TimeAllocationResult allocation;
        if (optimize_time_ratio)
        {
            allocation = allocator.optimize(fixed_total_time, positions, yaws);
        }
        else
        {
            allocation.durations = planning::FixedTotalTimeAllocator::distanceInitialGuess(
                fixed_total_time, positions);
            planning::FlatOutputTrajectory baseline;
            baseline.update(allocation.durations, positions, yaws);
            allocation.initial_cost = baseline.energy(1.0);
            allocation.final_cost = allocation.initial_cost;
        }

        const auto parameters = simulation_vehicle
                                    ? planning::VehicleParameters::simulationVehicle()
                                    : planning::VehicleParameters::paperVehicle();
        planning::FeasibilityChecker checker(
            planning::TailsitterFlatness(parameters), 0.002, 1.0,
            enforce_velocity_constraint);
        planning::TimeScaleResult scaling;
        if (scale_time)
        {
            planning::TimeScaler scaler(checker);
            scaling = scaler.search(
                positions, yaws, allocation.durations, 0.25, 3.0, 29, 5e-4);
            if (!scaling.found)
            {
                std::cerr << "no feasible time scale found in [0.25, 3.0]\n";
                return 1;
            }
        }
        else
        {
            scaling.minimum_feasible_scale = 1.0;
            scaling.durations = allocation.durations;
            planning::FlatOutputTrajectory fixed_time_trajectory;
            fixed_time_trajectory.update(scaling.durations, positions, yaws);
            scaling.report = checker.check(fixed_time_trajectory, true);
            scaling.found = scaling.report.feasible;
        }

        planning::FlatOutputTrajectory final_trajectory;
        final_trajectory.update(scaling.durations, positions, yaws);
        writeCsv(output, final_trajectory, scaling.report, parameters);

        const double final_time = std::accumulate(scaling.durations.begin(), scaling.durations.end(), 0.0);
        std::cout << std::fixed << std::setprecision(6)
                  << "vehicle: " << (simulation_vehicle ? "simulation" : "paper") << '\n'
                  << "yaw case: " << (yaw_pi ? "0 -> pi" : "0 -> 0") << '\n'
                  << "geometry: fixed straight line, distance=" << positions(positions.rows() - 1, 0)
                  << " m, waypoints=" << positions.rows() << '\n'
                  << "time-ratio optimization: " << (optimize_time_ratio ? "enabled" : "disabled") << '\n'
                  << "outer time scaling: " << (scale_time ? "enabled" : "disabled") << '\n'
                  << "velocity feasibility constraint: "
                  << (enforce_velocity_constraint ? "enabled" : "disabled") << '\n'
                  << "fixed-time durations: [";
        for (std::size_t segment = 0; segment < allocation.durations.size(); ++segment)
            std::cout << (segment == 0 ? "" : ", ") << allocation.durations[segment];
        std::cout << "] s\n"
                  << "energy: " << allocation.initial_cost << " -> " << allocation.final_cost << '\n'
                  << (scale_time ? "minimum feasible scale: " : "selected time scale: ")
                  << scaling.minimum_feasible_scale << '\n'
                  << (scale_time ? "minimum feasible total time: " : "selected total time: ")
                  << final_time << " s\n"
                  << "selected trajectory feasible: " << (scaling.report.feasible ? "yes" : "no") << '\n'
                  << "peak speed: " << scaling.report.peak_velocity << " m/s\n"
                  << "peak collective thrust: " << scaling.report.peak_collective_thrust << " N\n"
                  << "peak body rate: " << scaling.report.peak_body_rate << " rad/s\n"
                  << "peak rotor speed: " << scaling.report.peak_rotor_speed << " rad/s\n"
                  << "peak flap deflection: " << scaling.report.peak_flap_deflection << " rad\n"
                  << "peak flap rate: " << scaling.report.peak_flap_rate << " rad/s\n"
                  << "peak required rotor/flap command: "
                  << scaling.report.peak_required_rotor_command << " rad/s, "
                  << scaling.report.peak_required_flap_command << " rad\n"
                  << "CSV: " << std::filesystem::absolute(output) << '\n';
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}
