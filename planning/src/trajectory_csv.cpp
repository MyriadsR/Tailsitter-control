#include "planning/trajectory_csv.hpp"

#include <Eigen/Core>

#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

namespace planning
{

void writeTrajectoryCsv(const std::filesystem::path& path,
                        const FlatOutputTrajectory& trajectory,
                        const FeasibilityReport& report,
                        const VehicleParameters& parameters)
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
    ActuatorPrediction previous_prediction;
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

}  // namespace planning
