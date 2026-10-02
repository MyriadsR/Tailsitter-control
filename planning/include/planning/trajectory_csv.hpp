#pragma once

#include "planning/flat_output_trajectory.hpp"
#include "planning/planning_types.hpp"

#include <filesystem>

namespace planning
{

// Write a dense feasibility-checked trajectory plus its open-loop actuator
// predictions to CSV. The finite-difference rate and first-order-command
// columns are recomputed from adjacent predictions exactly as
// FeasibilityChecker evaluates them internally.
void writeTrajectoryCsv(const std::filesystem::path& path,
                        const FlatOutputTrajectory& trajectory,
                        const FeasibilityReport& report,
                        const VehicleParameters& parameters);

}  // namespace planning
