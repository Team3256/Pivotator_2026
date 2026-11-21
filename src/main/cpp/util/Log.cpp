// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "util/Log.h"

#include <filesystem>
#include <optional>
#include <string>
#include <system_error>
#include <unordered_map>

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/geometry/struct/Pose2dStruct.h>
#include <frc/geometry/struct/Pose3dStruct.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>

namespace {

wpi::log::DataLog* g_log = nullptr;

nt::NetworkTableInstance Inst() {
  return nt::NetworkTableInstance::GetDefault();
}

/** One logged key: a .wpilog entry (once logging started) and a NetworkTables publisher. */
template <typename Entry, typename Publisher>
struct Channel {
  std::optional<Entry> entry;
  Publisher pub;
};

/**
 * Finds the channel for {@code key}, creating it the first time. Later calls reuse it, so a loop
 * does no allocation for keys it has logged before.
 */
template <typename Entry, typename Publisher, typename MakePublisher>
Channel<Entry, Publisher>& Get(std::unordered_map<std::string, Channel<Entry, Publisher>>& map,
                               std::string_view key, MakePublisher make) {
  std::string k{key};
  auto it = map.find(k);
  if (it == map.end()) {
    it = map.emplace(k, Channel<Entry, Publisher>{std::nullopt,
                                                  make("/AdvantageKit/RealOutputs/" + k)})
             .first;
  }
  auto& channel = it->second;
  if (!channel.entry && g_log != nullptr) {
    channel.entry.emplace(*g_log, "/RealOutputs/" + k);
  }
  return channel;
}

}  // namespace

void Log::Start(std::string_view dir) {
  if (g_log != nullptr) {
    return;
  }
  if (!dir.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(std::string{dir}, ec);
  }
  frc::DataLogManager::Start(dir);
  // Our own values go straight into the log under /RealOutputs, and Mayhem's state machines write
  // their own Mayhem/... entries (MayhemObserver). Only mirror the dashboard from NetworkTables.
  // (Mirroring /Mayhem/ too would add empty NT: copies of machines that haven't run yet, which
  // Mayhem's log_states tool trips over.)
  frc::DataLogManager::LogNetworkTables(false);
  g_log = &frc::DataLogManager::GetLog();
  auto inst = Inst();
  inst.StartEntryDataLog(*g_log, "/SmartDashboard/", "NT:/SmartDashboard/");
  inst.StartConnectionDataLog(*g_log, "NTConnection");
  // Driver Station state and joysticks (Mayhem uses them to split logs into auto/teleop).
  frc::DriverStation::StartDataLog(*g_log, true);
}

wpi::log::DataLog* Log::GetLog() {
  return g_log;
}

void Log::Record(std::string_view key, double value) {
  static std::unordered_map<std::string, Channel<wpi::log::DoubleLogEntry, nt::DoublePublisher>>
      channels;
  auto& c = Get(channels, key,
                [](const std::string& t) { return Inst().GetDoubleTopic(t).Publish(); });
  if (c.entry) {
    c.entry->Update(value);
  }
  c.pub.Set(value);
}

void Log::Record(std::string_view key, bool value) {
  static std::unordered_map<std::string, Channel<wpi::log::BooleanLogEntry, nt::BooleanPublisher>>
      channels;
  auto& c = Get(channels, key,
                [](const std::string& t) { return Inst().GetBooleanTopic(t).Publish(); });
  if (c.entry) {
    c.entry->Update(value);
  }
  c.pub.Set(value);
}

void Log::Record(std::string_view key, std::string_view value) {
  static std::unordered_map<std::string, Channel<wpi::log::StringLogEntry, nt::StringPublisher>>
      channels;
  auto& c = Get(channels, key,
                [](const std::string& t) { return Inst().GetStringTopic(t).Publish(); });
  if (c.entry) {
    c.entry->Update(value);
  }
  c.pub.Set(value);
}

void Log::Record(std::string_view key, std::span<const double> values) {
  static std::unordered_map<std::string,
                            Channel<wpi::log::DoubleArrayLogEntry, nt::DoubleArrayPublisher>>
      channels;
  auto& c = Get(channels, key,
                [](const std::string& t) { return Inst().GetDoubleArrayTopic(t).Publish(); });
  if (c.entry) {
    c.entry->Update(values);
  }
  c.pub.Set(values);
}

void Log::Record(std::string_view key, const frc::Pose2d& value) {
  static std::unordered_map<std::string, Channel<wpi::log::StructLogEntry<frc::Pose2d>,
                                                  nt::StructPublisher<frc::Pose2d>>>
      channels;
  auto& c = Get(channels, key, [](const std::string& t) {
    return Inst().GetStructTopic<frc::Pose2d>(t).Publish();
  });
  if (c.entry) {
    c.entry->Update(value);
  }
  c.pub.Set(value);
}

void Log::Record(std::string_view key, const frc::Pose3d& value) {
  static std::unordered_map<std::string, Channel<wpi::log::StructLogEntry<frc::Pose3d>,
                                                  nt::StructPublisher<frc::Pose3d>>>
      channels;
  auto& c = Get(channels, key, [](const std::string& t) {
    return Inst().GetStructTopic<frc::Pose3d>(t).Publish();
  });
  if (c.entry) {
    c.entry->Update(value);
  }
  c.pub.Set(value);
}

void Log::Record(std::string_view key, std::span<const frc::Pose3d> values) {
  static std::unordered_map<std::string, Channel<wpi::log::StructArrayLogEntry<frc::Pose3d>,
                                                  nt::StructArrayPublisher<frc::Pose3d>>>
      channels;
  auto& c = Get(channels, key, [](const std::string& t) {
    return Inst().GetStructArrayTopic<frc::Pose3d>(t).Publish();
  });
  if (c.entry) {
    c.entry->Update(values);
  }
  c.pub.Set(values);
}
