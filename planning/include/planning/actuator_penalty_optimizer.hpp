#pragma once

#include "planning/flat_output_trajectory.hpp"
#include "planning/tailsitter_flatness.hpp"

#include <vector>

namespace planning
{

// Stage C constrained solvers. Every entry keeps the same decision variables,
// objective and explicit inequality constraints; only the NLopt algorithm and
// the AUGLAG subsolver change.
enum class ConstrainedSolver
{
    AuglagMma,
    AuglagCcsaq,
    AuglagSlsqp,
    Mma,
    Ccsaq,
    Slsqp
};

struct ActuatorPenaltyConfig
{
    double yaw_weight{1.0};
    double smoothness_weight{1.0};
    double actuator_weight{200.0};
    double geometry_weight{0.05};
    double actuator_margin{0.75};
    int samples_per_segment{50};
    double invalid_mapping_penalty{10.0};
    bool enable_body_rate_constraint{true};
    bool enable_high_level_constraints{false};
    bool enable_velocity_constraint{true};
    bool enable_actuator_dynamics{true};
    Eigen::Vector3d auxiliary_half_range{0.5, 0.5, 0.5};
    double tau_bound{5.0};
    double free_time_log_bound{1.5};
    double time_weight{0.5};
    int max_evaluations{400};
    double relative_tolerance{1e-7};
    // Stage C explicit-constraint settings. The ten actuator/high-level
    // integrals become inequality rows; invalid-mapping stays a barrier in the
    // objective because it has no usable gradient. Direct SLSQP converges to
    // feasibility from deeply infeasible starts within a modest evaluation
    // budget; the AUGLAG/MMA/CCSAQ variants remain available for experiments
    // but need larger budgets on this problem.
    ConstrainedSolver constrained_solver{ConstrainedSolver::Slsqp};
    double constraint_tolerance{1e-8};
    double constraint_satisfaction_tolerance{1e-6};
    int local_solver_max_evaluations{4000};
};

struct ActuatorPenaltyIntegral
{
    double thrust_1{0.0};
    double thrust_2{0.0};
    double flap_1{0.0};
    double flap_2{0.0};
    double body_rate{0.0};
    double velocity{0.0};
    double collective_thrust{0.0};
    double motor_dynamics{0.0};
    double flap_rate{0.0};
    double servo_dynamics{0.0};
    double invalid_mapping{0.0};

    double total() const
    {
        return thrust_1 + thrust_2 + flap_1 + flap_2 + body_rate + velocity +
               collective_thrust +
               motor_dynamics + flap_rate + servo_dynamics + invalid_mapping;
    }
};

struct StageBObjectiveTerms
{
    double smoothness{0.0};
    double normalized_smoothness{0.0};
    double geometry{0.0};
    double total_time{0.0};
    double time{0.0};
    ActuatorPenaltyIntegral actuator;
    double total{0.0};
};

struct StageBOptimizationResult
{
    bool optimized{false};
    int evaluations{0};
    std::vector<double> durations;
    FlatOutputTrajectory::PositionMatrix position_waypoints;
    StageBObjectiveTerms initial_terms;
    StageBObjectiveTerms final_terms;
};

struct StageBGradientCheckResult
{
    bool valid{false};
    double maximum_relative_error{0.0};
    std::vector<double> analytic;
    std::vector<double> finite_difference;
};

// Result of the Stage C joint optimization with explicit inequality
// constraints. `terms.actuator` still reports the full penalty decomposition
// for diagnostics, but `terms.total` only weights the invalid-mapping barrier
// from the actuator side; the ten other integrals are constraint rows.
struct ConstrainedOptimizationResult
{
    bool optimized{false};
    int objective_evaluations{0};
    int constraint_evaluations{0};
    std::vector<double> durations;
    FlatOutputTrajectory::PositionMatrix position_waypoints;
    StageBObjectiveTerms initial_terms;
    StageBObjectiveTerms final_terms;
    std::vector<double> initial_constraint_values;
    std::vector<double> final_constraint_values;
    double initial_max_sample_violation{0.0};
    double final_max_sample_violation{0.0};
    bool constraints_satisfied{false};
};

struct ConstraintGradientCheckResult
{
    bool valid{false};
    double maximum_relative_error{0.0};
    int constraint_rows{0};
    // Row-major constraint_rows x variable_count Jacobians, flattened.
    std::vector<double> analytic;
    std::vector<double> finite_difference;
};

class ActuatorPenaltyOptimizer
{
public:
    ActuatorPenaltyOptimizer(TailsitterFlatness flatness,
                             ActuatorPenaltyConfig config = ActuatorPenaltyConfig{});

    StageBOptimizationResult optimize(
        double total_time,
        const std::vector<double>& initial_durations,
        const FlatOutputTrajectory::PositionMatrix& nominal_position_waypoints,
        const FlatOutputTrajectory::YawMatrix& yaw_waypoints,
        const std::vector<int>& auxiliary_waypoint_indices) const;

    StageBOptimizationResult optimizeJoint(
        const std::vector<double>& initial_durations,
        const FlatOutputTrajectory::PositionMatrix& nominal_position_waypoints,
        const FlatOutputTrajectory::YawMatrix& yaw_waypoints,
        const std::vector<int>& auxiliary_waypoint_indices) const;

    // Stage C: same variables and parameterization as optimizeJoint, but the
    // ten per-type actuator/high-level integrals enter as explicit
    // inequality constraints instead of weighted penalties in the objective.
    //
    // The optional warm-start arguments decode into the initial variable
    // vector (log-duration ratios and normalized waypoint offsets). Passing
    // the result of the penalty-based optimizeJoint is the recommended
    // pipeline: the soft solution is nearly feasible, and SLSQP then only
    // has to polish it onto the hard constraint boundary.
    ConstrainedOptimizationResult optimizeJointConstrained(
        const std::vector<double>& initial_durations,
        const FlatOutputTrajectory::PositionMatrix& nominal_position_waypoints,
        const FlatOutputTrajectory::YawMatrix& yaw_waypoints,
        const std::vector<int>& auxiliary_waypoint_indices,
        const std::vector<double>& warm_start_durations = {},
        const FlatOutputTrajectory::PositionMatrix& warm_start_positions =
            FlatOutputTrajectory::PositionMatrix{}) const;

    StageBGradientCheckResult checkInitialGradient(
        double total_time,
        const std::vector<double>& initial_durations,
        const FlatOutputTrajectory::PositionMatrix& nominal_position_waypoints,
        const FlatOutputTrajectory::YawMatrix& yaw_waypoints,
        const std::vector<int>& auxiliary_waypoint_indices,
        double relative_step = 1e-6) const;

    StageBGradientCheckResult checkInitialJointGradient(
        const std::vector<double>& initial_durations,
        const FlatOutputTrajectory::PositionMatrix& nominal_position_waypoints,
        const FlatOutputTrajectory::YawMatrix& yaw_waypoints,
        const std::vector<int>& auxiliary_waypoint_indices,
        double relative_step = 1e-6) const;

    // Centered finite-difference validation of the explicit constraint
    // Jacobian at the initial point of optimizeJointConstrained.
    //
    // Known non-smoothness: the yaw spline is C2, so yaw jerk jumps at
    // segment knots while rotor speed depends on yaw acceleration. At knot
    // samples the moving-sample term of the dynamics rows has a one-sided
    // derivative, and centered differences see the average of both sides.
    // The resulting mismatch is below roughly one percent of the affected
    // row and is inherent to sampling penalties exactly at knots.
    ConstraintGradientCheckResult checkInitialJointConstraintGradient(
        const std::vector<double>& initial_durations,
        const FlatOutputTrajectory::PositionMatrix& nominal_position_waypoints,
        const FlatOutputTrajectory::YawMatrix& yaw_waypoints,
        const std::vector<int>& auxiliary_waypoint_indices,
        double relative_step = 1e-6) const;

private:
    TailsitterFlatness flatness_;
    ActuatorPenaltyConfig config_;
};

}  // namespace planning
