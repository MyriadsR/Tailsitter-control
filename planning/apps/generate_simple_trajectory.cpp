#include "planning/time_allocation.hpp"
#include "planning/time_scaler.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>

namespace
{
void writeCsv(const std::filesystem::path& path,
              const planning::FlatOutputTrajectory& trajectory,
              const planning::FeasibilityReport& report)
{
    if (!path.parent_path().empty())
        std::filesystem::create_directories(path.parent_path());
    std::ofstream stream(path);
    if (!stream)
        throw std::runtime_error("cannot open output CSV: " + path.string());
    stream << "t,px,py,pz,vx,vy,vz,ax,ay,az,jx,jy,jz,snap_x,snap_y,snap_z,"
              "yaw,yaw_rate,yaw_acceleration,omega1_pred,omega2_pred,delta1_pred,delta2_pred\n";
    stream << std::setprecision(12);
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
               << ',' << prediction.flap_1 << ',' << prediction.flap_2 << '\n';
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
        std::filesystem::path output = "output/simple_trajectory.csv";
        for (int i = 1; i < argc; ++i)
        {
            const std::string argument = argv[i];
            if (argument == "--yaw-pi") yaw_pi = true;
            else if (argument == "--simulation-vehicle") simulation_vehicle = true;
            else if (argument == "--no-optimize-time-ratio") optimize_time_ratio = false;
            else if (argument == "--no-time-scaling") scale_time = false;
            else if (argument == "--output" && i + 1 < argc) output = argv[++i];
            else
            {
                std::cerr << "usage: generate_simple_trajectory [--yaw-pi] [--simulation-vehicle] "
                             "[--no-optimize-time-ratio] [--no-time-scaling] [--output FILE]\n";
                return 2;
            }
        }

        planning::FlatOutputTrajectory::PositionMatrix positions(3, 3);
        positions << 0.0, 0.0, -10.0,
                     1.5, 0.0, -10.0,
                     6.0, 0.0, -10.0;
        planning::FlatOutputTrajectory::YawMatrix yaws(3, 1);
        yaws << 0.0, (yaw_pi ? 0.5 * 3.14159265358979323846 : 0.0),
                (yaw_pi ? 3.14159265358979323846 : 0.0);

        constexpr double fixed_total_time = 4.0;
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
        planning::FeasibilityChecker checker(planning::TailsitterFlatness(parameters), 0.002);
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
        writeCsv(output, final_trajectory, scaling.report);

        const double final_time = std::accumulate(scaling.durations.begin(), scaling.durations.end(), 0.0);
        std::cout << std::fixed << std::setprecision(6)
                  << "vehicle: " << (simulation_vehicle ? "simulation" : "paper") << '\n'
                  << "yaw case: " << (yaw_pi ? "0 -> pi" : "0 -> 0") << '\n'
                  << "time-ratio optimization: " << (optimize_time_ratio ? "enabled" : "disabled") << '\n'
                  << "outer time scaling: " << (scale_time ? "enabled" : "disabled") << '\n'
                  << "fixed-time durations: [" << allocation.durations[0] << ", "
                  << allocation.durations[1] << "] s\n"
                  << "energy: " << allocation.initial_cost << " -> " << allocation.final_cost << '\n'
                  << (scale_time ? "minimum feasible scale: " : "selected time scale: ")
                  << scaling.minimum_feasible_scale << '\n'
                  << (scale_time ? "minimum feasible total time: " : "selected total time: ")
                  << final_time << " s\n"
                  << "selected trajectory feasible: " << (scaling.report.feasible ? "yes" : "no") << '\n'
                  << "peak rotor speed: " << scaling.report.peak_rotor_speed << " rad/s\n"
                  << "peak flap deflection: " << scaling.report.peak_flap_deflection << " rad\n"
                  << "CSV: " << std::filesystem::absolute(output) << '\n';
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}
