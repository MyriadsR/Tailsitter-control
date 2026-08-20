#include "planning/tailsitter_flatness.hpp"

#include <Eigen/LU>
#include <Eigen/SVD>
#include <unsupported/Eigen/AutoDiff>

#include <array>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>
#include <vector>

namespace planning
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

struct Jet2
{
    double value{0.0};
    double first{0.0};
    double second{0.0};
};

Jet2 operator+(const Jet2& a, const Jet2& b)
{
    return {a.value + b.value, a.first + b.first, a.second + b.second};
}
Jet2 operator-(const Jet2& a, const Jet2& b)
{
    return {a.value - b.value, a.first - b.first, a.second - b.second};
}
Jet2 operator-(const Jet2& a) { return {-a.value, -a.first, -a.second}; }
Jet2 operator*(const Jet2& a, const Jet2& b)
{
    return {a.value * b.value,
            a.first * b.value + a.value * b.first,
            a.second * b.value + 2.0 * a.first * b.first + a.value * b.second};
}
Jet2 inverse(const Jet2& x)
{
    const double inv = 1.0 / x.value;
    return {inv,
            -x.first * inv * inv,
            2.0 * x.first * x.first * inv * inv * inv - x.second * inv * inv};
}
Jet2 operator/(const Jet2& a, const Jet2& b) { return a * inverse(b); }
Jet2 operator+(const Jet2& a, double b) { return a + Jet2{b, 0.0, 0.0}; }
Jet2 operator*(double a, const Jet2& b) { return Jet2{a, 0.0, 0.0} * b; }
Jet2 operator/(const Jet2& a, double b) { return a / Jet2{b, 0.0, 0.0}; }

Jet2 sin(const Jet2& x)
{
    const double s = std::sin(x.value);
    const double c = std::cos(x.value);
    return {s, c * x.first, c * x.second - s * x.first * x.first};
}
Jet2 cos(const Jet2& x)
{
    const double s = std::sin(x.value);
    const double c = std::cos(x.value);
    return {c, -s * x.first, -s * x.second - c * x.first * x.first};
}
Jet2 sqrt(const Jet2& x)
{
    const double root = std::sqrt(x.value);
    const double first = x.first / (2.0 * root);
    return {root, first, x.second / (2.0 * root) - x.first * x.first / (4.0 * root * root * root)};
}
Jet2 atan2(const Jet2& y, const Jet2& x, double tolerance)
{
    const double radius_squared = x.value * x.value + y.value * y.value;
    if (radius_squared < tolerance * tolerance)
        return {std::atan2(y.value, x.value), 0.0, 0.0};
    const double numerator = x.value * y.first - y.value * x.first;
    const double numerator_dot = x.value * y.second - y.value * x.second;
    const double radius_dot = 2.0 * (x.value * x.first + y.value * y.first);
    return {std::atan2(y.value, x.value),
            numerator / radius_squared,
            (numerator_dot * radius_squared - numerator * radius_dot) /
                (radius_squared * radius_squared)};
}

using JetVector = std::array<Jet2, 3>;

JetVector add(const JetVector& a, const JetVector& b)
{
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}
Jet2 dot(const JetVector& a, const JetVector& b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
JetVector rotateZ(const Jet2& angle, const JetVector& vector)
{
    const Jet2 c = cos(angle);
    const Jet2 s = sin(angle);
    return {c * vector[0] + s * vector[1], -s * vector[0] + c * vector[1], vector[2]};
}
JetVector rotateX(const Jet2& angle, const JetVector& vector)
{
    const Jet2 c = cos(angle);
    const Jet2 s = sin(angle);
    return {vector[0], c * vector[1] + s * vector[2], -s * vector[1] + c * vector[2]};
}
JetVector rotateY(const Jet2& angle, const JetVector& vector)
{
    const Jet2 c = cos(angle);
    const Jet2 s = sin(angle);
    return {c * vector[0] - s * vector[2], vector[1], s * vector[0] + c * vector[2]};
}
Eigen::Vector3d values(const JetVector& vector)
{
    return {vector[0].value, vector[1].value, vector[2].value};
}
Eigen::Vector3d firsts(const JetVector& vector)
{
    return {vector[0].first, vector[1].first, vector[2].first};
}

Eigen::Matrix3d worldToBodyMatrix(double yaw, double roll, double pitch)
{
    const double cp = std::cos(yaw), sp = std::sin(yaw);
    const double cr = std::cos(roll), sr = std::sin(roll);
    const double ct = std::cos(pitch), st = std::sin(pitch);
    Eigen::Matrix3d rz;
    rz << cp, sp, 0.0, -sp, cp, 0.0, 0.0, 0.0, 1.0;
    Eigen::Matrix3d rx;
    rx << 1.0, 0.0, 0.0, 0.0, cr, sr, 0.0, -sr, cr;
    Eigen::Matrix3d ry;
    ry << ct, 0.0, -st, 0.0, 1.0, 0.0, st, 0.0, ct;
    return ry * rx * rz;
}

bool allFinite(const ActuatorPrediction& output)
{
    return output.attitude.coeffs().allFinite() && output.angular_velocity.allFinite() &&
           output.angular_acceleration.allFinite() && output.moment.allFinite() &&
           std::isfinite(output.collective_thrust) && std::isfinite(output.thrust_1) &&
           std::isfinite(output.thrust_2) && std::isfinite(output.rotor_speed_1) &&
           std::isfinite(output.rotor_speed_2) && std::isfinite(output.flap_1) &&
           std::isfinite(output.flap_2);
}

struct AttitudeCandidate
{
    Jet2 phi;
    Jet2 theta_bar;
    Jet2 thrust;
    JetVector velocity_phi;
    Eigen::Quaterniond attitude{Eigen::Quaterniond::Identity()};
    double distance{0.0};
    int roll_branch{-1};
    int pitch_branch{-1};
};

using FlatDerivativeVector =
    Eigen::Matrix<double, ActuatorLinearization::flat_derivative_dimension, 1>;
using ADScalar = Eigen::AutoDiffScalar<FlatDerivativeVector>;

template <typename Scalar>
double scalarValue(const Scalar& value)
{
    return static_cast<double>(value);
}

template <typename DerivativeType>
double scalarValue(const Eigen::AutoDiffScalar<DerivativeType>& value)
{
    return value.value();
}

template <typename Scalar>
Scalar scalarSin(const Scalar& value)
{
    using std::sin;
    return sin(value);
}

template <typename Scalar>
Scalar scalarCos(const Scalar& value)
{
    using std::cos;
    return cos(value);
}

template <typename Scalar>
Scalar scalarSqrt(const Scalar& value)
{
    using std::sqrt;
    return sqrt(value);
}

template <typename Scalar>
Scalar scalarAtan2(const Scalar& y, const Scalar& x)
{
    using std::atan2;
    return atan2(y, x);
}

template <typename Scalar>
struct DifferentiableJet2
{
    Scalar value{0.0};
    Scalar first{0.0};
    Scalar second{0.0};
};

template <typename Scalar>
DifferentiableJet2<Scalar> operator+(const DifferentiableJet2<Scalar>& a,
                                     const DifferentiableJet2<Scalar>& b)
{
    return {a.value + b.value, a.first + b.first, a.second + b.second};
}

template <typename Scalar>
DifferentiableJet2<Scalar> operator-(const DifferentiableJet2<Scalar>& a,
                                     const DifferentiableJet2<Scalar>& b)
{
    return {a.value - b.value, a.first - b.first, a.second - b.second};
}

template <typename Scalar>
DifferentiableJet2<Scalar> operator-(const DifferentiableJet2<Scalar>& a)
{
    return {-a.value, -a.first, -a.second};
}

template <typename Scalar>
DifferentiableJet2<Scalar> operator*(const DifferentiableJet2<Scalar>& a,
                                     const DifferentiableJet2<Scalar>& b)
{
    return {a.value * b.value,
            a.first * b.value + a.value * b.first,
            a.second * b.value + Scalar(2.0) * a.first * b.first + a.value * b.second};
}

template <typename Scalar>
DifferentiableJet2<Scalar> differentiableInverse(const DifferentiableJet2<Scalar>& x)
{
    const Scalar inverse = Scalar(1.0) / x.value;
    return {inverse,
            -x.first * inverse * inverse,
            Scalar(2.0) * x.first * x.first * inverse * inverse * inverse -
                x.second * inverse * inverse};
}

template <typename Scalar>
DifferentiableJet2<Scalar> operator/(const DifferentiableJet2<Scalar>& a,
                                     const DifferentiableJet2<Scalar>& b)
{
    return a * differentiableInverse(b);
}

template <typename Scalar>
DifferentiableJet2<Scalar> operator+(const DifferentiableJet2<Scalar>& a, double b)
{
    return a + DifferentiableJet2<Scalar>{Scalar(b), Scalar(0.0), Scalar(0.0)};
}

template <typename Scalar>
DifferentiableJet2<Scalar> operator*(double a, const DifferentiableJet2<Scalar>& b)
{
    return DifferentiableJet2<Scalar>{Scalar(a), Scalar(0.0), Scalar(0.0)} * b;
}

template <typename Scalar>
DifferentiableJet2<Scalar> operator/(const DifferentiableJet2<Scalar>& a, double b)
{
    return a / DifferentiableJet2<Scalar>{Scalar(b), Scalar(0.0), Scalar(0.0)};
}

template <typename Scalar>
DifferentiableJet2<Scalar> differentiableSin(const DifferentiableJet2<Scalar>& x)
{
    const Scalar sine = scalarSin(x.value);
    const Scalar cosine = scalarCos(x.value);
    return {sine,
            cosine * x.first,
            cosine * x.second - sine * x.first * x.first};
}

template <typename Scalar>
DifferentiableJet2<Scalar> differentiableCos(const DifferentiableJet2<Scalar>& x)
{
    const Scalar sine = scalarSin(x.value);
    const Scalar cosine = scalarCos(x.value);
    return {cosine,
            -sine * x.first,
            -sine * x.second - cosine * x.first * x.first};
}

template <typename Scalar>
DifferentiableJet2<Scalar> differentiableSqrt(const DifferentiableJet2<Scalar>& x)
{
    const Scalar root = scalarSqrt(x.value);
    const Scalar first = x.first / (Scalar(2.0) * root);
    return {root,
            first,
            x.second / (Scalar(2.0) * root) -
                x.first * x.first / (Scalar(4.0) * root * root * root)};
}

template <typename Scalar>
DifferentiableJet2<Scalar> differentiableAtan2(const DifferentiableJet2<Scalar>& y,
                                               const DifferentiableJet2<Scalar>& x,
                                               double tolerance)
{
    const Scalar radius_squared = x.value * x.value + y.value * y.value;
    if (scalarValue(radius_squared) < tolerance * tolerance)
    {
        return {scalarAtan2(y.value, x.value), Scalar(0.0), Scalar(0.0)};
    }
    const Scalar numerator = x.value * y.first - y.value * x.first;
    const Scalar numerator_dot = x.value * y.second - y.value * x.second;
    const Scalar radius_dot = Scalar(2.0) * (x.value * x.first + y.value * y.first);
    return {scalarAtan2(y.value, x.value),
            numerator / radius_squared,
            (numerator_dot * radius_squared - numerator * radius_dot) /
                (radius_squared * radius_squared)};
}

template <typename Scalar>
using DifferentiableJetVector = std::array<DifferentiableJet2<Scalar>, 3>;

template <typename Scalar>
DifferentiableJetVector<Scalar> differentiableAdd(const DifferentiableJetVector<Scalar>& a,
                                                  const DifferentiableJetVector<Scalar>& b)
{
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

template <typename Scalar>
DifferentiableJet2<Scalar> differentiableDot(const DifferentiableJetVector<Scalar>& a,
                                             const DifferentiableJetVector<Scalar>& b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

template <typename Scalar>
DifferentiableJetVector<Scalar> differentiableRotateZ(
    const DifferentiableJet2<Scalar>& angle,
    const DifferentiableJetVector<Scalar>& vector)
{
    const auto cosine = differentiableCos(angle);
    const auto sine = differentiableSin(angle);
    return {cosine * vector[0] + sine * vector[1],
            -sine * vector[0] + cosine * vector[1],
            vector[2]};
}

template <typename Scalar>
DifferentiableJetVector<Scalar> differentiableRotateX(
    const DifferentiableJet2<Scalar>& angle,
    const DifferentiableJetVector<Scalar>& vector)
{
    const auto cosine = differentiableCos(angle);
    const auto sine = differentiableSin(angle);
    return {vector[0],
            cosine * vector[1] + sine * vector[2],
            -sine * vector[1] + cosine * vector[2]};
}

template <typename Scalar>
DifferentiableJetVector<Scalar> differentiableRotateY(
    const DifferentiableJet2<Scalar>& angle,
    const DifferentiableJetVector<Scalar>& vector)
{
    const auto cosine = differentiableCos(angle);
    const auto sine = differentiableSin(angle);
    return {cosine * vector[0] - sine * vector[2],
            vector[1],
            sine * vector[0] + cosine * vector[2]};
}

template <typename Scalar>
Eigen::Matrix<Scalar, 3, 1> differentiableValues(
    const DifferentiableJetVector<Scalar>& vector)
{
    return {vector[0].value, vector[1].value, vector[2].value};
}

template <typename Scalar>
Eigen::Matrix<Scalar, 3, 1> differentiableFirsts(
    const DifferentiableJetVector<Scalar>& vector)
{
    return {vector[0].first, vector[1].first, vector[2].first};
}

template <typename Scalar>
Eigen::Matrix<Scalar, 3, 3> differentiableWorldToBodyMatrix(
    const Scalar& yaw,
    const Scalar& roll,
    const Scalar& pitch)
{
    const Scalar cp = scalarCos(yaw), sp = scalarSin(yaw);
    const Scalar cr = scalarCos(roll), sr = scalarSin(roll);
    const Scalar ct = scalarCos(pitch), st = scalarSin(pitch);
    Eigen::Matrix<Scalar, 3, 3> rz;
    rz << cp, sp, Scalar(0.0),
          -sp, cp, Scalar(0.0),
          Scalar(0.0), Scalar(0.0), Scalar(1.0);
    Eigen::Matrix<Scalar, 3, 3> rx;
    rx << Scalar(1.0), Scalar(0.0), Scalar(0.0),
          Scalar(0.0), cr, sr,
          Scalar(0.0), -sr, cr;
    Eigen::Matrix<Scalar, 3, 3> ry;
    ry << ct, Scalar(0.0), -st,
          Scalar(0.0), Scalar(1.0), Scalar(0.0),
          st, Scalar(0.0), ct;
    return ry * rx * rz;
}

template <typename Scalar>
struct FlatDerivativeInput
{
    Eigen::Matrix<Scalar, 3, 1> velocity;
    Eigen::Matrix<Scalar, 3, 1> acceleration;
    Eigen::Matrix<Scalar, 3, 1> jerk;
    Eigen::Matrix<Scalar, 3, 1> snap;
    Scalar yaw;
    Scalar yaw_rate;
    Scalar yaw_acceleration;
};

template <typename Scalar>
struct FixedBranchActuators
{
    Scalar thrust_1{0.0};
    Scalar thrust_2{0.0};
    Scalar flap_1{0.0};
    Scalar flap_2{0.0};
    Eigen::Matrix<Scalar, 3, 1> angular_velocity{
        Eigen::Matrix<Scalar, 3, 1>::Zero()};
    bool valid{false};
    std::string failure_reason;
};

template <typename Scalar>
FixedBranchActuators<Scalar> evaluateFixedBranchActuators(
    const FlatDerivativeInput<Scalar>& input,
    int roll_branch,
    int pitch_branch,
    const VehicleParameters& p)
{
    FixedBranchActuators<Scalar> result;
    using DJet = DifferentiableJet2<Scalar>;
    using DVector = DifferentiableJetVector<Scalar>;

    const DVector velocity{{{input.velocity.x(), input.acceleration.x(), input.jerk.x()},
                            {input.velocity.y(), input.acceleration.y(), input.jerk.y()},
                            {input.velocity.z(), input.acceleration.z(), input.jerk.z()}}};
    const DVector force{{{p.mass * input.acceleration.x(),
                          p.mass * input.jerk.x(),
                          p.mass * input.snap.x()},
                         {p.mass * input.acceleration.y(),
                          p.mass * input.jerk.y(),
                          p.mass * input.snap.y()},
                         {p.mass * (input.acceleration.z() - p.gravity),
                          p.mass * input.jerk.z(),
                          p.mass * input.snap.z()}}};
    const DJet yaw{input.yaw, input.yaw_rate, input.yaw_acceleration};
    const DVector force_psi = differentiableRotateZ(yaw, force);
    const DJet phi = -differentiableAtan2(force_psi[1], force[2],
                                          p.singularity_tolerance) +
                     roll_branch * kPi;
    const DJet speed = differentiableSqrt(
        differentiableDot(velocity, velocity) +
        p.velocity_regularization * p.velocity_regularization);

    const double alpha_bar = p.alpha_0 + p.alpha_T;
    const double eta_denominator = std::cos(alpha_bar) * (1.0 - p.c_DT);
    if (std::abs(eta_denominator) < p.singularity_tolerance)
    {
        result.failure_reason = "collective-thrust denominator is singular";
        return result;
    }
    const double eta = std::sin(alpha_bar) * (p.c_LT - 1.0) / eta_denominator;
    const DVector velocity_phi =
        differentiableRotateX(phi, differentiableRotateZ(yaw, velocity));
    const DVector force_phi = differentiableRotateX(phi, force_psi);
    const DJet sigma_y = eta * (force_phi[0] + p.c_DV * speed * velocity_phi[0]) -
                         p.c_LV * speed * velocity_phi[2] - force_phi[2];
    const DJet sigma_x = eta * (force_phi[2] + p.c_DV * speed * velocity_phi[2]) +
                         p.c_LV * speed * velocity_phi[0] + force_phi[0];
    const DJet theta_bar = differentiableAtan2(sigma_y, sigma_x,
                                               p.singularity_tolerance) +
                           pitch_branch * kPi;
    const DJet thrust =
        (differentiableCos(theta_bar) * force_phi[0] -
         differentiableSin(theta_bar) * force_phi[2] +
         p.c_DV * speed *
             (differentiableCos(theta_bar) * velocity_phi[0] -
              differentiableSin(theta_bar) * velocity_phi[2])) /
        eta_denominator;
    if (!(scalarValue(thrust.value) >= 0.0) || !std::isfinite(scalarValue(thrust.value)))
    {
        result.failure_reason = "fixed attitude branch requires negative collective thrust";
        return result;
    }

    const DJet theta = theta_bar + p.alpha_0;
    const DJet yaw_rate{yaw.first, yaw.second, Scalar(0.0)};
    const DJet phi_rate{phi.first, phi.second, Scalar(0.0)};
    const DJet theta_rate{theta.first, theta.second, Scalar(0.0)};
    const DJet zero{Scalar(0.0), Scalar(0.0), Scalar(0.0)};
    const DVector yaw_rate_world{{zero, zero, yaw_rate}};
    const DVector roll_rate_phi{{phi_rate, zero, zero}};
    const DVector pitch_rate_body{{zero, theta_rate, zero}};
    const DVector yaw_contribution = differentiableRotateY(
        theta, differentiableRotateX(phi, differentiableRotateZ(yaw, yaw_rate_world)));
    const DVector roll_contribution = differentiableRotateY(theta, roll_rate_phi);
    const DVector omega = differentiableAdd(
        differentiableAdd(yaw_contribution, roll_contribution), pitch_rate_body);
    const Eigen::Matrix<Scalar, 3, 1> angular_velocity = differentiableValues(omega);
    result.angular_velocity = angular_velocity;
    const Eigen::Matrix<Scalar, 3, 1> angular_acceleration = differentiableFirsts(omega);
    const Eigen::Matrix<Scalar, 3, 3> inertia = p.inertia.template cast<Scalar>();
    const Eigen::Matrix<Scalar, 3, 1> moment =
        inertia * angular_acceleration +
        angular_velocity.cross(inertia * angular_velocity);

    const double ca0 = std::cos(p.alpha_0), sa0 = std::sin(p.alpha_0);
    const double caT = std::cos(p.alpha_T), saT = std::sin(p.alpha_T);
    const double cab = std::cos(alpha_bar), sab = std::sin(alpha_bar);
    const double thrust_force_b_x =
        ca0 * cab * (1.0 - p.c_DT) - sa0 * sab * (p.c_LT - 1.0);
    const double thrust_force_b_z =
        sa0 * cab * (1.0 - p.c_DT) + ca0 * sab * (p.c_LT - 1.0);
    const double differential_denominator =
        p.l_Ty * thrust_force_b_x - saT * (p.c_mu / p.c_T);
    if (std::abs(differential_denominator) < p.singularity_tolerance)
    {
        result.failure_reason = "differential-thrust denominator is singular";
        return result;
    }
    const Scalar differential_thrust = moment.z() / differential_denominator;
    result.thrust_1 = Scalar(0.5) * (thrust.value + differential_thrust);
    result.thrust_2 = Scalar(0.5) * (thrust.value - differential_thrust);
    if (scalarValue(result.thrust_1) < 0.0 || scalarValue(result.thrust_2) < 0.0)
    {
        result.failure_reason = "negative individual motor thrust";
        return result;
    }

    const Eigen::Matrix<Scalar, 3, 1> moment_thrust(
        p.l_Ty * thrust_force_b_z * (result.thrust_2 - result.thrust_1),
        p.c_mu_T * thrust.value,
        p.l_Ty * thrust_force_b_x * differential_thrust);
    const Eigen::Matrix<Scalar, 3, 1> moment_torque(
        caT * (p.c_mu / p.c_T) * differential_thrust,
        Scalar(0.0),
        -saT * (p.c_mu / p.c_T) * differential_thrust);
    const Eigen::Matrix<Scalar, 3, 1> flap_moment =
        moment - moment_thrust - moment_torque;

    const auto world_to_body = differentiableWorldToBodyMatrix(
        yaw.value, phi.value, theta.value);
    const Eigen::Matrix<Scalar, 3, 1> velocity_body = world_to_body * input.velocity;
    const Scalar velocity_alpha_x =
        ca0 * velocity_body.x() + sa0 * velocity_body.z();
    const Scalar speed_squared = input.velocity.squaredNorm();
    const Scalar speed_value =
        scalarValue(speed_squared) <= p.singularity_tolerance * p.singularity_tolerance
            ? Scalar(0.0)
            : scalarSqrt(speed_squared);
    const Scalar nu_1 = -p.c_LT_delta * cab * result.thrust_1 -
                        p.c_LV_delta * speed_value * velocity_alpha_x;
    const Scalar nu_2 = -p.c_LT_delta * cab * result.thrust_2 -
                        p.c_LV_delta * speed_value * velocity_alpha_x;
    const Scalar a00 = -p.l_delta_y * ca0 * nu_1;
    const Scalar a01 = p.l_delta_y * ca0 * nu_2;
    const Scalar a10 = p.l_delta_x * nu_1;
    const Scalar a11 = p.l_delta_x * nu_2;
    const Scalar determinant = a00 * a11 - a01 * a10;
    if (std::abs(scalarValue(determinant)) < p.singularity_tolerance)
    {
        result.failure_reason = "flap allocation matrix is singular";
        return result;
    }
    result.flap_1 = (a11 * flap_moment.x() - a01 * flap_moment.y()) / determinant;
    result.flap_2 = (-a10 * flap_moment.x() + a00 * flap_moment.y()) / determinant;
    result.valid = std::isfinite(scalarValue(result.thrust_1)) &&
                   std::isfinite(scalarValue(result.thrust_2)) &&
                   std::isfinite(scalarValue(result.flap_1)) &&
                   std::isfinite(scalarValue(result.flap_2));
    if (!result.valid)
        result.failure_reason = "non-finite fixed-branch actuator output";
    return result;
}

FlatDerivativeInput<double> flatDerivativeInput(const FlatOutputSample& sample)
{
    return {sample.velocity,
            sample.acceleration,
            sample.jerk,
            sample.snap,
            sample.yaw,
            sample.yaw_rate,
            sample.yaw_acceleration};
}

ADScalar independentAD(double value, int derivative_index)
{
    ADScalar result(value);
    result.derivatives().setZero();
    result.derivatives()[derivative_index] = 1.0;
    return result;
}

FlatDerivativeInput<ADScalar> differentiableFlatDerivativeInput(
    const FlatOutputSample& sample)
{
    FlatDerivativeInput<ADScalar> result;
    for (int axis = 0; axis < 3; ++axis)
    {
        result.velocity[axis] = independentAD(
            sample.velocity[axis], ActuatorLinearization::velocity_offset + axis);
        result.acceleration[axis] = independentAD(
            sample.acceleration[axis], ActuatorLinearization::acceleration_offset + axis);
        result.jerk[axis] = independentAD(
            sample.jerk[axis], ActuatorLinearization::jerk_offset + axis);
        result.snap[axis] = independentAD(
            sample.snap[axis], ActuatorLinearization::snap_offset + axis);
    }
    result.yaw = independentAD(sample.yaw, ActuatorLinearization::yaw_index);
    result.yaw_rate = independentAD(sample.yaw_rate, ActuatorLinearization::yaw_rate_index);
    result.yaw_acceleration = independentAD(
        sample.yaw_acceleration, ActuatorLinearization::yaw_acceleration_index);
    return result;
}

}  // namespace

VehicleParameters VehicleParameters::paperVehicle() { return VehicleParameters{}; }

VehicleParameters VehicleParameters::simulationVehicle()
{
    VehicleParameters parameters;
    parameters.mass = 0.7;
    parameters.c_T = 1.8e-6;
    parameters.c_mu = 2.2e-8;
    return parameters;
}

TailsitterFlatness::TailsitterFlatness(VehicleParameters parameters)
    : parameters_(std::move(parameters))
{
}

ActuatorPrediction TailsitterFlatness::forward(const FlatOutputSample& sample,
                                               const BranchState& previous_branch) const
{
    ActuatorPrediction output;
    output.time = sample.time;
    const auto& p = parameters_;
    const JetVector velocity{{{sample.velocity.x(), sample.acceleration.x(), sample.jerk.x()},
                              {sample.velocity.y(), sample.acceleration.y(), sample.jerk.y()},
                              {sample.velocity.z(), sample.acceleration.z(), sample.jerk.z()}}};
    const JetVector force{{{p.mass * (sample.acceleration.x()), p.mass * sample.jerk.x(), p.mass * sample.snap.x()},
                           {p.mass * (sample.acceleration.y()), p.mass * sample.jerk.y(), p.mass * sample.snap.y()},
                           {p.mass * (sample.acceleration.z() - p.gravity), p.mass * sample.jerk.z(), p.mass * sample.snap.z()}}};
    const Jet2 yaw{sample.yaw, sample.yaw_rate, sample.yaw_acceleration};
    const JetVector force_psi = rotateZ(yaw, force);
    const Jet2 phi_base = -atan2(force_psi[1], force[2], p.singularity_tolerance);
    const Jet2 speed = sqrt(dot(velocity, velocity) + p.velocity_regularization * p.velocity_regularization);

    const double alpha_bar = p.alpha_0 + p.alpha_T;
    const double eta_denominator = std::cos(alpha_bar) * (1.0 - p.c_DT);
    if (std::abs(eta_denominator) < p.singularity_tolerance)
    {
        output.failure_reason = "collective-thrust denominator is singular";
        return output;
    }
    const double eta = std::sin(alpha_bar) * (p.c_LT - 1.0) / eta_denominator;

    std::vector<AttitudeCandidate> candidates;
    candidates.reserve(4);
    for (int roll_branch = 0; roll_branch < 2; ++roll_branch)
    {
        const Jet2 phi = phi_base + roll_branch * kPi;
        const JetVector velocity_phi = rotateX(phi, rotateZ(yaw, velocity));
        const JetVector force_phi = rotateX(phi, force_psi);
        const Jet2 sigma_y = eta * (force_phi[0] + p.c_DV * speed * velocity_phi[0]) -
                             p.c_LV * speed * velocity_phi[2] - force_phi[2];
        const Jet2 sigma_x = eta * (force_phi[2] + p.c_DV * speed * velocity_phi[2]) +
                             p.c_LV * speed * velocity_phi[0] + force_phi[0];
        const Jet2 theta_base = atan2(sigma_y, sigma_x, p.singularity_tolerance);

        for (int pitch_branch = 0; pitch_branch < 2; ++pitch_branch)
        {
            const Jet2 theta_bar = theta_base + pitch_branch * kPi;
            const Jet2 thrust = (cos(theta_bar) * force_phi[0] - sin(theta_bar) * force_phi[2] +
                                 p.c_DV * speed *
                                     (cos(theta_bar) * velocity_phi[0] - sin(theta_bar) * velocity_phi[2])) /
                                eta_denominator;
            if (!(thrust.value >= 0.0) || !std::isfinite(thrust.value))
                continue;

            const double theta = theta_bar.value + p.alpha_0;
            const Eigen::Matrix3d world_to_body = worldToBodyMatrix(yaw.value, phi.value, theta);
            Eigen::Quaterniond attitude(world_to_body.transpose());
            attitude.normalize();
            double distance = 0.0;
            if (previous_branch.initialized)
                distance = 1.0 - std::abs(previous_branch.attitude.dot(attitude));
            candidates.push_back(
                {phi, theta_bar, thrust, velocity_phi, attitude, distance,
                 roll_branch, pitch_branch});
        }
    }

    if (candidates.empty())
    {
        output.failure_reason = "no nonnegative-thrust attitude branch";
        return output;
    }
    const auto selected = std::min_element(candidates.begin(), candidates.end(),
                                           [](const auto& a, const auto& b) { return a.distance < b.distance; });
    const Jet2 theta = selected->theta_bar + p.alpha_0;
    output.attitude = selected->attitude;
    output.collective_thrust = selected->thrust.value;
    output.roll_branch = selected->roll_branch;
    output.pitch_branch = selected->pitch_branch;

    const Jet2 yaw_rate{yaw.first, yaw.second, 0.0};
    const Jet2 phi_rate{selected->phi.first, selected->phi.second, 0.0};
    const Jet2 theta_rate{theta.first, theta.second, 0.0};
    const Jet2 zero{0.0, 0.0, 0.0};
    const JetVector yaw_rate_world{{zero, zero, yaw_rate}};
    const JetVector roll_rate_phi{{phi_rate, zero, zero}};
    const JetVector pitch_rate_body{{zero, theta_rate, zero}};
    const JetVector yaw_contribution =
        rotateY(theta, rotateX(selected->phi, rotateZ(yaw, yaw_rate_world)));
    const JetVector roll_contribution = rotateY(theta, roll_rate_phi);
    const JetVector omega = add(add(yaw_contribution, roll_contribution), pitch_rate_body);
    output.angular_velocity = values(omega);
    output.angular_acceleration = firsts(omega);
    output.moment = p.inertia * output.angular_acceleration +
                    output.angular_velocity.cross(p.inertia * output.angular_velocity);

    const double ca0 = std::cos(p.alpha_0), sa0 = std::sin(p.alpha_0);
    const double caT = std::cos(p.alpha_T), saT = std::sin(p.alpha_T);
    const double cab = std::cos(alpha_bar), sab = std::sin(alpha_bar);
    const double thrust_force_b_x = ca0 * cab * (1.0 - p.c_DT) - sa0 * sab * (p.c_LT - 1.0);
    const double thrust_force_b_z = sa0 * cab * (1.0 - p.c_DT) + ca0 * sab * (p.c_LT - 1.0);
    const double differential_denominator = p.l_Ty * thrust_force_b_x - saT * (p.c_mu / p.c_T);
    if (std::abs(differential_denominator) < p.singularity_tolerance)
    {
        output.failure_reason = "differential-thrust denominator is singular";
        return output;
    }
    const double differential_thrust = output.moment.z() / differential_denominator;
    output.thrust_1 = 0.5 * (output.collective_thrust + differential_thrust);
    output.thrust_2 = 0.5 * (output.collective_thrust - differential_thrust);
    if (output.thrust_1 < 0.0 || output.thrust_2 < 0.0)
    {
        output.failure_reason = "negative individual motor thrust";
        return output;
    }
    output.rotor_speed_1 = std::sqrt(output.thrust_1 / p.c_T);
    output.rotor_speed_2 = std::sqrt(output.thrust_2 / p.c_T);

    const Eigen::Vector3d moment_thrust(
        p.l_Ty * thrust_force_b_z * (output.thrust_2 - output.thrust_1),
        p.c_mu_T * output.collective_thrust,
        p.l_Ty * thrust_force_b_x * differential_thrust);
    const Eigen::Vector3d moment_torque(
        caT * (p.c_mu / p.c_T) * differential_thrust,
        0.0,
        -saT * (p.c_mu / p.c_T) * differential_thrust);
    const Eigen::Vector3d flap_moment = output.moment - moment_thrust - moment_torque;

    const Eigen::Matrix3d world_to_body = worldToBodyMatrix(yaw.value, selected->phi.value, theta.value);
    const Eigen::Vector3d velocity_body = world_to_body * sample.velocity;
    const double velocity_alpha_x = ca0 * velocity_body.x() + sa0 * velocity_body.z();
    const double speed_value = sample.velocity.norm();
    const double nu_1 = -p.c_LT_delta * cab * output.thrust_1 -
                        p.c_LV_delta * speed_value * velocity_alpha_x;
    const double nu_2 = -p.c_LT_delta * cab * output.thrust_2 -
                        p.c_LV_delta * speed_value * velocity_alpha_x;
    Eigen::Matrix2d allocation;
    allocation << -p.l_delta_y * ca0 * nu_1, p.l_delta_y * ca0 * nu_2,
                   p.l_delta_x * nu_1, p.l_delta_x * nu_2;
    const Eigen::JacobiSVD<Eigen::Matrix2d> svd(allocation);
    output.flap_matrix_min_singular_value = svd.singularValues().minCoeff();
    if (output.flap_matrix_min_singular_value < p.singularity_tolerance)
    {
        output.failure_reason = "flap allocation matrix is singular";
        return output;
    }
    const Eigen::Vector2d flaps = allocation.fullPivLu().solve(flap_moment.head<2>());
    output.flap_1 = flaps.x();
    output.flap_2 = flaps.y();
    output.valid = allFinite(output);
    if (!output.valid)
        output.failure_reason = "non-finite open-loop flatness output";
    return output;
}

ActuatorLinearization TailsitterFlatness::linearize(
    const FlatOutputSample& sample,
    const BranchState& previous_branch) const
{
    ActuatorLinearization output;
    output.prediction = forward(sample, previous_branch);
    if (!output.prediction.valid)
    {
        output.failure_reason = output.prediction.failure_reason;
        return output;
    }
    if (output.prediction.roll_branch < 0 || output.prediction.pitch_branch < 0)
    {
        output.failure_reason = "nominal flatness evaluation did not report an attitude branch";
        return output;
    }

    const auto nominal = evaluateFixedBranchActuators(
        flatDerivativeInput(sample),
        output.prediction.roll_branch,
        output.prediction.pitch_branch,
        parameters_);
    if (!nominal.valid)
    {
        output.failure_reason = nominal.failure_reason;
        return output;
    }

    const auto differentiated = evaluateFixedBranchActuators(
        differentiableFlatDerivativeInput(sample),
        output.prediction.roll_branch,
        output.prediction.pitch_branch,
        parameters_);
    if (!differentiated.valid)
    {
        output.failure_reason = differentiated.failure_reason;
        return output;
    }

    const std::array<ADScalar, ActuatorLinearization::actuator_dimension> actuators{{
        differentiated.thrust_1,
        differentiated.thrust_2,
        differentiated.flap_1,
        differentiated.flap_2}};
    for (int actuator = 0; actuator < ActuatorLinearization::actuator_dimension; ++actuator)
    {
        output.jacobian.row(actuator) = actuators[static_cast<std::size_t>(actuator)]
                                            .derivatives()
                                            .transpose();
    }
    for (int axis = 0; axis < 3; ++axis)
    {
        output.angular_velocity_jacobian.row(axis) =
            differentiated.angular_velocity[axis].derivatives().transpose();
    }
    output.differentiable = output.jacobian.allFinite() &&
                            output.angular_velocity_jacobian.allFinite();
    if (!output.differentiable)
        output.failure_reason = "non-finite fixed-branch actuator Jacobian";
    return output;
}

}  // namespace planning
