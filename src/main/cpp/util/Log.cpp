// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "util/Log.h"

#include <string>
#include <unordered_map>

#include <frc/geometry/struct/Pose3dStruct.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>

namespace {

constexpr std::string_view kRoot = "/Robot/";

nt::NetworkTableInstance Inst() {
  return nt::NetworkTableInstance::GetDefault();
}

/** Creates a publisher the first time a key is used, then reuses it every loop after that. */
template <typename Publisher, typename MakePublisher>
Publisher& GetPublisher(std::unordered_map<std::string, Publisher>& publishers,
                        std::string_view key, MakePublisher make) {
  std::string k{key};
  auto it = publishers.find(k);
  if (it == publishers.end()) {
    it = publishers.emplace(k, make(std::string{kRoot} + k)).first;
  }
  return it->second;
}

}  // namespace

void Log::Record(std::string_view key, double value) {
  static std::unordered_map<std::string, nt::DoublePublisher> pubs;
  GetPublisher(pubs, key, [](const std::string& topic) {
    return Inst().GetDoubleTopic(topic).Publish();
  }).Set(value);
}

void Log::Record(std::string_view key, bool value) {
  static std::unordered_map<std::string, nt::BooleanPublisher> pubs;
  GetPublisher(pubs, key, [](const std::string& topic) {
    return Inst().GetBooleanTopic(topic).Publish();
  }).Set(value);
}

void Log::Record(std::string_view key, std::string_view value) {
  static std::unordered_map<std::string, nt::StringPublisher> pubs;
  GetPublisher(pubs, key, [](const std::string& topic) {
    return Inst().GetStringTopic(topic).Publish();
  }).Set(value);
}

void Log::Record(std::string_view key, std::span<const double> values) {
  static std::unordered_map<std::string, nt::DoubleArrayPublisher> pubs;
  GetPublisher(pubs, key, [](const std::string& topic) {
    return Inst().GetDoubleArrayTopic(topic).Publish();
  }).Set(values);
}

void Log::Record(std::string_view key, const frc::Pose3d& value) {
  static std::unordered_map<std::string, nt::StructPublisher<frc::Pose3d>> pubs;
  GetPublisher(pubs, key, [](const std::string& topic) {
    return Inst().GetStructTopic<frc::Pose3d>(topic).Publish();
  }).Set(value);
}

void Log::Record(std::string_view key, std::span<const frc::Pose3d> values) {
  static std::unordered_map<std::string, nt::StructArrayPublisher<frc::Pose3d>> pubs;
  GetPublisher(pubs, key, [](const std::string& topic) {
    return Inst().GetStructArrayTopic<frc::Pose3d>(topic).Publish();
  }).Set(values);
}
