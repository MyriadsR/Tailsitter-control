#pragma once

#include "planning/flat_output_trajectory.hpp"
#include "planning/tailsitter_flatness.hpp"

#include <vector>

namespace planning
{

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
    bool enable_actuator_dynamics{true};
    Eigen::Vector3d auxiliary_half_range{0.5, 0.5, 0.5};
    double tau_bound{5.0};
    double free_time_log_bound{1.5};
    double time_weight{0.5};
    int max_evaluations{400};
    double relative_tolerance{1e-7};
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

private:
    TailsitterFlatness flatness_;
    ActuatorPenaltyConfig config_;
};

}  // namespace planning
