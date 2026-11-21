// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include "commands/MayhemProject.h"
#include "subsystems/elevator/ElevatorConstants.h"

/**
 * Where each Superstructure goal puts the mechanisms. The goals themselves (and which button
 * picks which) are the "Superstructure" machine in robot.mayhem.json / tools/mayhem/machines.json.
 *
 * PLACEHOLDERS: the pivot and wrist angles are first guesses. Tune them on the robot together
 * with the elevator heights in ElevatorConstants.
 */
namespace SuperstructureConstants {

using Goal = mayhem_generated::Superstructure::State;

struct Targets {
  double elevatorMeters;
  /** Degrees above horizontal (90 = elevator upright). */
  double pivotDeg;
  /** Degrees relative to the elevator (0 = in line with it). */
  double wristDeg;
};

constexpr Targets For(Goal goal) {
  using H = ElevatorConstants::Height;
  switch (goal) {
    case Goal::STOW:
      return {ElevatorConstants::Meters(H::kStow), 90.0, 0.0};
    case Goal::CORAL_INTAKE:
      return {ElevatorConstants::Meters(H::kCoralIntake), 60.0, -45.0};  // TODO: tune
    case Goal::L1:
      return {ElevatorConstants::Meters(H::kL1), 75.0, 30.0};  // TODO: tune
    case Goal::L2:
      return {ElevatorConstants::Meters(H::kL2), 80.0, 30.0};  // TODO: tune
    case Goal::L3:
      return {ElevatorConstants::Meters(H::kL3), 85.0, 30.0};  // TODO: tune
    case Goal::L4:
      return {ElevatorConstants::Meters(H::kL4), 88.0, 45.0};  // TODO: tune
  }
  return {0.0, 90.0, 0.0};
}

// ---------------- Motion sequencing ----------------
/** A pivot move bigger than this pulls the elevator in first (Motion input pivotFar). */
inline constexpr double kPivotMoveDeg = 5.0;
/** ...and the pivot counts as there (pivotFar false again) once it's within this. */
inline constexpr double kPivotSettledDeg = 2.0;

/**
 * Where the elevator waits while the pivot swings: a little under the safe height, so it settles
 * clearly inside it.
 */
inline constexpr double kElevatorHoldMeters = ElevatorConstants::kPivotSafeMaxHeightMeters - 0.03;

/** Everything is "at goal" inside these, and stops being at goal outside the looser ones. */
inline constexpr double kAtGoalElevatorMeters = 0.015;
inline constexpr double kAtGoalPivotDeg = 1.5;
inline constexpr double kAtGoalWristDeg = 2.0;
inline constexpr double kLeaveGoalElevatorMeters = 0.03;
inline constexpr double kLeaveGoalPivotDeg = 3.0;
inline constexpr double kLeaveGoalWristDeg = 4.0;

}  // namespace SuperstructureConstants
