// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "subsystems/endeffector/EndEffector.h"

#include <utility>

#include "util/Log.h"

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
  Log::Record("EndEffector/Inputs/MotorConnected", in.motorConnected);
  Log::Record("EndEffector/Inputs/VelocityRps", in.velocityRps);
  Log::Record("EndEffector/Inputs/Voltage", in.voltage);
  Log::Record("EndEffector/Inputs/StatorCurrent", in.statorCurrent);
  Log::Record("EndEffector/Inputs/SupplyCurrent", in.supplyCurrent);
  Log::Record("EndEffector/Inputs/TempCelsius", in.tempCelsius);
  Log::Record("EndEffector/Inputs/SensorConnected", in.sensorConnected);
  Log::Record("EndEffector/Inputs/SensorDistanceMeters", in.sensorDistanceMeters);
  Log::Record("EndEffector/Inputs/HasCoral", in.hasCoral);
}

frc2::CommandPtr EndEffector::SetVoltage(std::function<double()> volts) {
  return Run([this, volts = std::move(volts)] {
           m_reqVolts = volts();
           m_io->SetVoltage(m_reqVolts);
         })
      .WithName("EndEffector.setVoltage");
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
