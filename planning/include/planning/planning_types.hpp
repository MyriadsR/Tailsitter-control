#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>
#include <string>
#include <vector>

namespace planning
{

struct FlatOutputSample
{
    double time{0.0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
    Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
    Eigen::Vector3d snap{Eigen::Vector3d::Zero()};
    Eigen::Vector3d crackle{Eigen::Vector3d::Zero()};
    double yaw{0.0};
    double yaw_rate{0.0};
    double yaw_acceleration{0.0};
    double yaw_jerk{0.0};
};

struct VehicleParameters
{
    double mass{0.68};
    double gravity{9.81};
    double alpha_0{-2.0 * 3.14159265358979323846 / 180.0};
    double alpha_T{-5.0 * 3.14159265358979323846 / 180.0};
    double l_Ty{0.15};
    double l_delta_y{0.12};
    double l_delta_x{0.075};
    double c_T{1.62e-6};
    double c_mu{1.78e-8};
    double c_mu_T{-0.025};
    double c_LV{0.29};
    double c_DV{0.0};
    double c_LT{2.23};
    double c_DT{0.0};
    double c_LV_delta{0.18};
    double c_LT_delta{1.25};
    Eigen::Matrix3d inertia{(Eigen::Vector3d(0.0095, 0.0030, 0.0115)).asDiagonal()};
    double omega_min{0.0};
    double omega_max{2500.0};
    double delta_max{1.0};
    // Planning/reference envelope from the reported 650 deg/s maneuver peak.
    // This is not a manufacturer-certified hardware rate limit.
    double body_rate_max{650.0 * 3.14159265358979323846 / 180.0};
    // Reference translational-speed envelope for the high-level planner.
    double velocity_max{8.0};
    // Engineering estimates shared with config.py/main_sim.py.
    double motor_time_constant{0.04};
    double servo_time_constant{0.03};
    double servo_rate_max{25.0};
    double velocity_regularization{1e-3};
    double singularity_tolerance{1e-9};

    static VehicleParameters paperVehicle();
    static VehicleParameters simulationVehicle();
};

struct BranchState
{
    bool initialized{false};
    Eigen::Quaterniond attitude{Eigen::Quaterniond::Identity()};
};

struct ActuatorPrediction
{
    double time{0.0};
    Eigen::Quaterniond attitude{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d angular_acceleration{Eigen::Vector3d::Zero()};
    Eigen::Vector3d moment{Eigen::Vector3d::Zero()};
    double collective_thrust{0.0};
    double thrust_1{0.0};
    double thrust_2{0.0};
    double rotor_speed_1{std::numeric_limits<double>::quiet_NaN()};
    double rotor_speed_2{std::numeric_limits<double>::quiet_NaN()};
    double flap_1{std::numeric_limits<double>::quiet_NaN()};
    double flap_2{std::numeric_limits<double>::quiet_NaN()};
    double flap_matrix_min_singular_value{0.0};
    int roll_branch{-1};
    int pitch_branch{-1};
    bool valid{false};
    std::string failure_reason;
};

struct ActuatorLinearization
{
    static constexpr int flat_derivative_dimension = 15;
    static constexpr int actuator_dimension = 4;
    static constexpr int velocity_offset = 0;
    static constexpr int acceleration_offset = 3;
    static constexpr int jerk_offset = 6;
    static constexpr int snap_offset = 9;
    static constexpr int yaw_index = 12;
    static constexpr int yaw_rate_index = 13;
    static constexpr int yaw_acceleration_index = 14;

    ActuatorPrediction prediction;
    Eigen::Matrix<double, actuator_dimension, flat_derivative_dimension> jacobian{
        Eigen::Matrix<double, actuator_dimension, flat_derivative_dimension>::Zero()};
    Eigen::Matrix<double, 3, flat_derivative_dimension> angular_velocity_jacobian{
        Eigen::Matrix<double, 3, flat_derivative_dimension>::Zero()};
    bool differentiable{false};
    std::string failure_reason;
};

struct FeasibilityReport
{
    bool feasible{false};
    double max_normalized_violation{-std::numeric_limits<double>::infinity()};
    double peak_rotor_speed{0.0};
    double peak_flap_deflection{0.0};
    double peak_body_rate{0.0};
    double peak_velocity{0.0};
    double peak_collective_thrust{0.0};
    double peak_flap_rate{0.0};
    double peak_required_rotor_command{0.0};
    double peak_required_flap_command{0.0};
    double first_failure_time{std::numeric_limits<double>::quiet_NaN()};
    std::string first_failure_reason;
    std::vector<ActuatorPrediction> predictions;
};

}  // namespace planning
