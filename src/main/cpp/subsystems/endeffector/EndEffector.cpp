// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "subsystems/endeffector/EndEffector.h"

#include <string>
#include <utility>

#include "subsystems/endeffector/EndEffectorConstants.h"
#include "util/Log.h"

namespace {

void LogMotor(const std::string& prefix, const RollerMotorInputs& in) {
  Log::Record(prefix + "/Connected", in.connected);
  Log::Record(prefix + "/VelocityRps", in.velocityRps);
  Log::Record(prefix + "/Voltage", in.voltage);
  Log::Record(prefix + "/StatorCurrent", in.statorCurrent);
  Log::Record(prefix + "/SupplyCurrent", in.supplyCurrent);
  Log::Record(prefix + "/TempCelsius", in.tempCelsius);
}

}  // namespace

EndEffector::EndEffector(bool enabled, std::unique_ptr<EndEffectorIO> io)
    : DisableSubsystem(enabled, "EndEffector"), m_io{std::move(io)} {}

void EndEffector::Periodic() {
  DisableSubsystem::Periodic();
  m_io->UpdateInputs(m_inputs);
  LogInputs();
  Log::Record("EndEffector/ReqVoltage", m_reqVolts);
  Log::Record("EndEffector/HasCoral", m_inputs.hasCoral);
}

void EndEffector::LogInputs() const {
  const EndEffectorIOInputs& in = m_inputs;
  LogMotor("EndEffector/Inputs/Top", in.top);
  LogMotor("EndEffector/Inputs/SideLeader", in.sideLeader);
  LogMotor("EndEffector/Inputs/SideFollower", in.sideFollower);
  Log::Record("EndEffector/Inputs/SensorConnected", in.sensorConnected);
  Log::Record("EndEffector/Inputs/SensorDistanceMeters", in.sensorDistanceMeters);
  Log::Record("EndEffector/Inputs/HasCoral", in.hasCoral);
}

frc2::CommandPtr EndEffector::SetVoltage(std::function<double()> volts) {
  return Run([this, volts = std::move(volts)] {
           Apply(volts());
         })
      .WithName("EndEffector.setVoltage");
}

void EndEffector::Apply(double volts) {
  m_reqVolts = volts;
  m_io->SetVoltage(volts, volts * EndEffectorConstants::kSideVoltsScale);
}

frc2::CommandPtr EndEffector::SetVoltage(double volts) {
  return SetVoltage([volts] { return volts; });
}

frc2::CommandPtr EndEffector::Off() {
  return RunOnce([this] {
           m_reqVolts = 0.0;
           m_io->Off();
         })
      .WithName("EndEffector.off");
}
