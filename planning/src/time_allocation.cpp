#include "planning/time_allocation.hpp"

#include <nlopt.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace planning
{
namespace
{
struct ObjectiveData
{
    double total_time;
    double yaw_weight;
    const FlatOutputTrajectory::PositionMatrix* position_waypoints;
    const FlatOutputTrajectory::YawMatrix* yaw_waypoints;
    int evaluations{0};
};

double objective(const std::vector<double>& tau, std::vector<double>& gradient, void* raw_data)
{
    auto& data = *static_cast<ObjectiveData*>(raw_data);
    ++data.evaluations;
    const auto durations = FixedTotalTimeAllocator::durationsFromTau(data.total_time, tau);
    FlatOutputTrajectory trajectory;
    trajectory.update(durations, *data.position_waypoints, *data.yaw_waypoints);
    const double cost = trajectory.energy(data.yaw_weight);

    if (!gradient.empty())
    {
        const Eigen::VectorXd grad_t = trajectory.energyTimeGradient(data.yaw_weight);
        double weighted_mean = 0.0;
        for (std::size_t j = 0; j < durations.size(); ++j)
            weighted_mean += durations[j] * grad_t(static_cast<Eigen::Index>(j));
        weighted_mean /= data.total_time;

        for (std::size_t i = 0; i < tau.size(); ++i)
            gradient[i] = durations[i] * (grad_t(static_cast<Eigen::Index>(i)) - weighted_mean);
    }
    return cost;
}
}  // namespace

FixedTotalTimeAllocator::FixedTotalTimeAllocator(double yaw_weight) : yaw_weight_(yaw_weight)
{
    if (yaw_weight < 0.0)
        throw std::invalid_argument("yaw weight must be nonnegative");
}

std::vector<double> FixedTotalTimeAllocator::distanceInitialGuess(
    double total_time, const FlatOutputTrajectory::PositionMatrix& points)
{
    if (!(total_time > 0.0) || points.rows() < 2)
        throw std::invalid_argument("invalid total time or waypoint count");
    std::vector<double> weights(static_cast<std::size_t>(points.rows() - 1));
    constexpr double epsilon = 1e-3;
    for (Eigen::Index i = 0; i + 1 < points.rows(); ++i)
        weights[static_cast<std::size_t>(i)] = (points.row(i + 1) - points.row(i)).norm() + epsilon;
    const double sum = std::accumulate(weights.begin(), weights.end(), 0.0);
    for (double& value : weights)
        value *= total_time / sum;
    return weights;
}

std::vector<double> FixedTotalTimeAllocator::durationsFromTau(
    double total_time, const std::vector<double>& tau_free)
{
    const std::size_t segment_count = tau_free.size() + 1;
    std::vector<double> logits(segment_count, 0.0);
    std::copy(tau_free.begin(), tau_free.end(), logits.begin());
    const double maximum = *std::max_element(logits.begin(), logits.end());
    double sum = 0.0;
    for (double& value : logits)
    {
        value = std::exp(value - maximum);
        sum += value;
    }
    for (double& value : logits)
        value *= total_time / sum;
    return logits;
}

TimeAllocationResult FixedTotalTimeAllocator::optimize(
    double total_time,
    const FlatOutputTrajectory::PositionMatrix& position_waypoints,
    const FlatOutputTrajectory::YawMatrix& yaw_waypoints) const
{
    TimeAllocationResult result;
    result.durations = distanceInitialGuess(total_time, position_waypoints);
    FlatOutputTrajectory initial;
    initial.update(result.durations, position_waypoints, yaw_waypoints);
    result.initial_cost = initial.energy(yaw_weight_);
    result.final_cost = result.initial_cost;

    if (result.durations.size() == 1)
        return result;

    std::vector<double> tau(result.durations.size() - 1);
    const double last_duration = result.durations.back();
    for (std::size_t i = 0; i < tau.size(); ++i)
        tau[i] = std::log(result.durations[i] / last_duration);

    ObjectiveData data{total_time, yaw_weight_, &position_waypoints, &yaw_waypoints};
    nlopt::opt optimizer(nlopt::LD_LBFGS, static_cast<unsigned>(tau.size()));
    optimizer.set_min_objective(objective, &data);
    optimizer.set_lower_bounds(std::vector<double>(tau.size(), -12.0));
    optimizer.set_upper_bounds(std::vector<double>(tau.size(), 12.0));
    optimizer.set_ftol_rel(1e-10);
    optimizer.set_xtol_rel(1e-9);
    optimizer.set_maxeval(250);

    double minimum = result.initial_cost;
    try
    {
        optimizer.optimize(tau, minimum);
    }
    catch (const std::exception& error)
    {
        throw std::runtime_error(std::string("NLopt time allocation failed: ") + error.what());
    }

    result.durations = durationsFromTau(total_time, tau);
    result.final_cost = minimum;
    result.evaluations = data.evaluations;
    result.optimized = true;
    return result;
}

}  // namespace planning
