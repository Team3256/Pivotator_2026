// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include "mayhem/sim/field.hpp"

/**
 * The 2025 REEFSCAPE field for the physics engine: what a robot can drive into. Blue-origin
 * field coordinates in meters, from Mayhem's field model (app/src/suite/core/reefscape-2025.json).
 */
namespace reefscape {

inline constexpr double kFieldLength = 17.548;
inline constexpr double kFieldWidth = 8.052;

/** Blue reef centre and the distance from it to each flat face. */
inline constexpr double kBlueReefX = 4.489, kBlueReefY = 4.026;
inline constexpr double kReefApothem = 0.832;

/** Adds the walls, both reefs and the four coral-station corners. */
void BuildField(mayhem::sim::Obstacles& obstacles);

}  // namespace reefscape
