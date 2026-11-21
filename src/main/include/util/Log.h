// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <span>
#include <string_view>

#include <frc/geometry/Pose3d.h>

/**
 * Small stand-in for AdvantageKit's Logger.recordOutput (AdvantageKit is Java-only).
 *
 * Every value is published to NetworkTables under "/Robot/<key>". Robot starts DataLogManager,
 * which records all of NetworkTables into a .wpilog file. So you can watch values live in
 * AdvantageScope (connect to the robot / localhost) or open the .wpilog afterwards, just like
 * before.
 */
namespace Log {

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
