// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "sim/SimBattery.h"

#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <units/current.h>

namespace {
std::map<std::string, double>& Currents() {
  static std::map<std::string, double> currents;
  return currents;
}
}  // namespace

void SimBattery::Report(std::string_view mechanism, double amps) {
  Currents()[std::string{mechanism}] = std::abs(amps);
}

void SimBattery::Update() {
  std::vector<units::ampere_t> amps;
  for (const auto& [name, current] : Currents()) {
    amps.emplace_back(current);
  }
  frc::sim::RoboRioSim::SetVInVoltage(frc::sim::BatterySim::Calculate(amps));
}
