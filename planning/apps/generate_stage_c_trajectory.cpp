#include "planning/actuator_penalty_optimizer.hpp"
#include "planning/feasibility_checker.hpp"
#include "planning/trajectory_csv.hpp"

#include <cmath>
#include <filesystem>
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

planning::ConstrainedSolver parseSolver(const std::string& text)
{
    if (text == "auglag-mma") return planning::ConstrainedSolver::AuglagMma;
    if (text == "auglag-ccsaq") return planning::ConstrainedSolver::AuglagCcsaq;
    if (text == "auglag-slsqp") return planning::ConstrainedSolver::AuglagSlsqp;
    if (text == "slsqp") return planning::ConstrainedSolver::Slsqp;
    if (text == "mma") return planning::ConstrainedSolver::Mma;
    if (text == "ccsaq") return planning::ConstrainedSolver::Ccsaq;
    throw std::invalid_argument(
        "unknown solver '" + text +
        "' (expected auglag-mma|auglag-ccsaq|auglag-slsqp|slsqp|mma|ccsaq)");
}

const char* solverName(planning::ConstrainedSolver solver)
{
    switch (solver)
    {
        case planning::ConstrainedSolver::AuglagMma: return "auglag-mma";
        case planning::ConstrainedSolver::AuglagCcsaq: return "auglag-ccsaq";
        case planning::ConstrainedSolver::AuglagSlsqp: return "auglag-slsqp";
        case planning::ConstrainedSolver::Slsqp: return "slsqp";
        case planning::ConstrainedSolver::Mma: return "mma";
        case planning::ConstrainedSolver::Ccsaq: return "ccsaq";
    }
    return "unknown";
}

void printDurations(const char* label, const std::vector<double>& durations)
{
    std::cout << label << " durations: [";
    for (std::size_t segment = 0; segment < durations.size(); ++segment)
        std::cout << (segment == 0 ? "" : ", ") << durations[segment];
    std::cout << "] s, total="
              << std::accumulate(durations.begin(), durations.end(), 0.0) << " s\n";
}

void printConstraintRows(const char* label, const std::vector<double>& values)
{
    static const char* names[] = {"thrust_1", "thrust_2", "flap_1", "flap_2",
                                  "body_rate", "velocity", "collective",
                                  "motor_dynamics", "flap_rate", "servo_dynamics"};
    std::cout << label << " constraint rows [name: value]:\n";
    for (std::size_t row = 0; row < values.size(); ++row)
        std::cout << "  " << names[row] << ": " << values[row]
                  << (values[row] > 0.0 ? "  (violated)" : "") << '\n';
}

}  // namespace

int main(int argc, char** argv)
{
    try
    {
        bool yaw_pi = false;
        bool simulation_vehicle = false;
        bool enforce_velocity_constraint = true;
        bool warm_start_penalty = true;
        // Warm-start sampling resolution. Empirically 20 samples/segment is
        // the sweet spot: coarse enough for ~3x cheaper preoptimization
        // evaluations, fine enough that L-BFGS converges without extra
        // iterations; 0 falls back to the fine soft-penalty warm start.
        int coarse_samples = 20;
        double distance = 15.0;
        double initial_total_time = 4.0;
        planning::ActuatorPenaltyConfig config;
        config.enable_high_level_constraints = true;
        config.auxiliary_half_range = Eigen::Vector3d(2.0, 3.0, 2.0);
        config.geometry_weight = 0.02;
        config.max_evaluations = 600;
        config.constrained_solver = planning::ConstrainedSolver::Slsqp;
        std::filesystem::path output = "output/stage_c_optimized.csv";
        std::filesystem::path baseline_output = "output/stage_c_initial.csv";

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
            else if (argument == "--no-velocity-constraint")
                enforce_velocity_constraint = false;
            else if (argument == "--cold-start")
                warm_start_penalty = false;
            else if (argument == "--coarse-samples")
                coarse_samples = parseInt(next("--coarse-samples"), "--coarse-samples");
            else if (argument == "--distance")
                distance = parseDouble(next("--distance"), "--distance");
            else if (argument == "--initial-total-time")
                initial_total_time =
                    parseDouble(next("--initial-total-time"), "--initial-total-time");
            else if (argument == "--solver")
                config.constrained_solver = parseSolver(next("--solver"));
            else if (argument == "--time-weight")
                config.time_weight = parseDouble(next("--time-weight"), "--time-weight");
            else if (argument == "--actuator-margin")
                config.actuator_margin =
                    parseDouble(next("--actuator-margin"), "--actuator-margin");
            else if (argument == "--geometry-weight")
                config.geometry_weight =
                    parseDouble(next("--geometry-weight"), "--geometry-weight");
            else if (argument == "--free-time-log-bound")
                config.free_time_log_bound =
                    parseDouble(next("--free-time-log-bound"), "--free-time-log-bound");
            else if (argument == "--samples-per-segment")
                config.samples_per_segment =
                    parseInt(next("--samples-per-segment"), "--samples-per-segment");
            else if (argument == "--maxeval")
                config.max_evaluations = parseInt(next("--maxeval"), "--maxeval");
            else if (argument == "--local-maxeval")
                config.local_solver_max_evaluations =
                    parseInt(next("--local-maxeval"), "--local-maxeval");
            else if (argument == "--constraint-tol")
                config.constraint_tolerance =
                    parseDouble(next("--constraint-tol"), "--constraint-tol");
            else if (argument == "--output") output = next("--output");
            else if (argument == "--baseline-output") baseline_output = next("--baseline-output");
            else
            {
                std::cerr << "usage: generate_stage_c_trajectory [--yaw-pi] "
                             "[--simulation-vehicle] [--no-velocity-constraint] "
                             "[--cold-start] [--coarse-samples N] [--distance M] "
                             "[--initial-total-time S] "
                             "[--solver NAME] [--time-weight W] "
                             "[--actuator-margin M] [--geometry-weight W] "
                             "[--free-time-log-bound B] [--samples-per-segment N] "
                             "[--maxeval N] [--local-maxeval N] "
                             "[--constraint-tol T] [--output FILE] "
                             "[--baseline-output FILE]\n";
                return 2;
            }
        }
        if (!(distance > 0.0) || !(initial_total_time > 0.0))
            throw std::invalid_argument("distance and initial total time must be positive");
        config.enable_velocity_constraint = enforce_velocity_constraint;

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
        std::vector<double> warm_start_durations;
        planning::FlatOutputTrajectory::PositionMatrix warm_start_positions;
        if (warm_start_penalty)
        {
            // Standard pipeline: a soft-penalty joint solution is nearly
            // feasible; Stage C only has to polish it onto the hard boundary.
            // The coarse preoptimization resolves the trajectory shape cheaply;
            // the fine SLSQP stage then enforces the per-sample envelopes.
            if (coarse_samples > 0 && coarse_samples != config.samples_per_segment)
            {
                if (!(coarse_samples > 0))
                    throw std::invalid_argument("--coarse-samples must be positive");
                planning::ActuatorPenaltyConfig coarse_config = config;
                coarse_config.samples_per_segment = coarse_samples;
                planning::ActuatorPenaltyOptimizer coarse_optimizer(
                    planning::TailsitterFlatness(parameters), coarse_config);
                const auto coarse = coarse_optimizer.optimizeJoint(
                    initial_durations, nominal_positions, yaws, free_waypoints);
                warm_start_durations = coarse.durations;
                warm_start_positions = coarse.position_waypoints;
                std::cout << std::fixed << std::setprecision(6)
                          << "coarse penalty warm start ("
                          << coarse_samples << " samples/segment): evaluations "
                          << coarse.evaluations << ", total time "
                          << std::accumulate(coarse.durations.begin(),
                                             coarse.durations.end(), 0.0)
                          << " s\n";
            }
            else
            {
                const auto penalty = optimizer.optimizeJoint(
                    initial_durations, nominal_positions, yaws, free_waypoints);
                warm_start_durations = penalty.durations;
                warm_start_positions = penalty.position_waypoints;
                std::cout << std::fixed << std::setprecision(6)
                          << "penalty warm start: evaluations "
                          << penalty.evaluations
                          << ", total time "
                          << std::accumulate(penalty.durations.begin(),
                                             penalty.durations.end(), 0.0)
                          << " s\n";
            }
        }
        const auto result = optimizer.optimizeJointConstrained(
            initial_durations, nominal_positions, yaws, free_waypoints,
            warm_start_durations, warm_start_positions);

        planning::FlatOutputTrajectory initial_trajectory;
        initial_trajectory.update(initial_durations, nominal_positions, yaws);
        planning::FlatOutputTrajectory optimized_trajectory;
        optimized_trajectory.update(result.durations, result.position_waypoints, yaws);
        planning::FeasibilityChecker checker(
            planning::TailsitterFlatness(parameters), 0.002, 1.0,
            enforce_velocity_constraint);
        const auto initial_report = checker.check(initial_trajectory, true);
        const auto optimized_report = checker.check(optimized_trajectory, true);
        planning::writeTrajectoryCsv(
            baseline_output, initial_trajectory, initial_report, parameters);
        planning::writeTrajectoryCsv(
            output, optimized_trajectory, optimized_report, parameters);

        const auto print_terms = [](const char* label,
                                    const planning::StageBObjectiveTerms& terms) {
            std::cout << label
                      << " objective=" << terms.total
                      << " (smoothness=" << terms.smoothness
                      << ", normalized=" << terms.normalized_smoothness
                      << ", time=" << terms.time
                      << ", geometry=" << terms.geometry << ")\n"
                      << label << " actuator integrals [T1,T2,d1,d2,body-rate,"
                         "velocity,collective,motor-dyn,flap-rate,servo-dyn,invalid]: ["
                      << terms.actuator.thrust_1 << ", " << terms.actuator.thrust_2
                      << ", " << terms.actuator.flap_1 << ", "
                      << terms.actuator.flap_2 << ", " << terms.actuator.body_rate
                      << ", " << terms.actuator.velocity << ", "
                      << terms.actuator.collective_thrust << ", "
                      << terms.actuator.motor_dynamics << ", "
                      << terms.actuator.flap_rate << ", "
                      << terms.actuator.servo_dynamics << ", "
                      << terms.actuator.invalid_mapping << "]\n";
        };
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

        std::cout << std::fixed << std::setprecision(6)
                  << "stage C constrained joint hover-to-hover optimization\n"
                  << "yaw case: " << (yaw_pi ? "0 -> pi" : "0 -> 0") << '\n'
                  << "distance: " << distance << " m, variables: "
                  << segment_count + 9
                  << " (" << segment_count << " log-durations + 3 free 3D waypoints)\n"
                  << "solver: " << solverName(config.constrained_solver)
                  << ", local maxeval: " << config.local_solver_max_evaluations << '\n'
                  << "velocity constraint: "
                  << (enforce_velocity_constraint ? "enabled" : "disabled") << '\n'
                  << "time weight: " << config.time_weight
                  << ", planning margin: " << config.actuator_margin << '\n'
                  << "objective evaluations: " << result.objective_evaluations
                  << ", constraint evaluations: " << result.constraint_evaluations
                  << '\n';
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
        print_terms("initial", result.initial_terms);
        print_terms("final", result.final_terms);
        printConstraintRows("initial", result.initial_constraint_values);
        printConstraintRows("final", result.final_constraint_values);
        std::cout << "initial max raw sample violation (planning envelope): "
                  << result.initial_max_sample_violation << '\n'
                  << "final max raw sample violation (planning envelope): "
                  << result.final_max_sample_violation << '\n'
                  << "constraints satisfied within tolerance "
                  << config.constraint_satisfaction_tolerance << ": "
                  << (result.constraints_satisfied ? "yes" : "no") << '\n';
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
