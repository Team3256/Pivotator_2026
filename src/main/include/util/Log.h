// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <span>
#include <string_view>

#include <frc/geometry/Pose3d.h>
#include <wpi/DataLog.h>

/**
 * Stand-in for AdvantageKit's Logger.recordOutput (AdvantageKit is Java-only), using the same
 * names AdvantageKit wrote so Mayhem and AdvantageScope read our logs like any 3256 robot's:
 *
 *  - the .wpilog file gets "/RealOutputs/<key>" (the 3D replay in Mayhem reads
 *    /RealOutputs/Sim/RobotPose and /RealOutputs/Sim/Components, sim proof reads
 *    /RealOutputs/Mayhem/..., log analysis reads the rest), and
 *  - NetworkTables gets "/AdvantageKit/RealOutputs/<key>" for live viewing (Mayhem's live view,
 *    AdvantageScope).
 *
 * Call Start() once before anything is recorded. Values recorded before Start() only go to
 * NetworkTables.
 */
namespace Log {

/**
 * Starts the .wpilog: {@code dir} empty = WPILib's default (USB stick, else /home/lvuser/logs on
 * the robot; ./logs in sim). Also records the Driver Station, joysticks, SmartDashboard and
 * Mayhem's NetworkTables telemetry. Safe to call more than once (later calls do nothing).
 */
void Start(std::string_view dir = "");

/** The data log, or nullptr before Start(). */
wpi::log::DataLog* GetLog();

void Record(std::string_view key, double value);
void Record(std::string_view key, bool value);
void Record(std::string_view key, std::string_view value);
// Without this, Record("key", "text") would pick the bool overload.
inline void Record(std::string_view key, const char* value) {
  Record(key, std::string_view{value});
}
void Record(std::string_view key, std::span<const double> values);
void Record(std::string_view key, const frc::Pose3d& value);
void Record(std::string_view key, std::span<const frc::Pose3d> values);

}  // namespace Log
