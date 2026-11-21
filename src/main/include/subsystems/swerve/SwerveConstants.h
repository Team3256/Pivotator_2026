// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <numbers>

#include "subsystems/swerve/generated/TunerConstants.h"

/** Driver and drivetrain limits. Module hardware lives in generated/TunerConstants.h. */
namespace SwerveConstants {

/** Stick deadband, as a fraction of full speed. */
inline constexpr double kDeadband = 0.1;
inline constexpr double kMaxSpeed = TunerConstants::kSpeedAt12Volts.value();  // m/s
inline constexpr double kMaxAngularRate = 1.5 * std::numbers::pi;              // rad/s

/**
 * With the elevator up the robot is tall and top-heavy, so the driver's speed is scaled down while
 * the elevator is extended: full speed below kSlowFromHeight, kExtendedSpeedScale at full height.
 */
inline constexpr double kSlowFromHeightMeters = 0.15;
inline constexpr double kExtendedSpeedScale = 0.35;

// ---------------- Autonomous PID (drive-to-pose) ----------------
inline constexpr double kTranslationKP = 8.0;
inline constexpr double kHeadingKP = 5.0;

}  // namespace SwerveConstants
