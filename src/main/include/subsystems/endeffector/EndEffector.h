// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <functional>
#include <memory>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/Trigger.h>

#include "subsystems/endeffector/EndEffectorIO.h"
#include "util/DisableSubsystem.h"

/** End effector rollers and coral sensor. Positive voltage pulls coral in. */
class EndEffector : public DisableSubsystem {
 public:
  EndEffector(bool enabled, std::unique_ptr<EndEffectorIO> io);

  void Periodic() override;

  /** Runs the rollers at a voltage that can change every loop (the Superstructure's target). */
  frc2::CommandPtr SetVoltage(std::function<double()> volts);
  frc2::CommandPtr SetVoltage(double volts);
  frc2::CommandPtr Off();

  bool HasCoral() const { return m_inputs.hasCoral; }
  double VelocityRps() const { return m_inputs.velocityRps; }

  frc2::Trigger hasCoral{[this] { return HasCoral(); }};

 private:
  void LogInputs() const;

  std::unique_ptr<EndEffectorIO> m_io;
  EndEffectorIOInputs m_inputs;
  double m_reqVolts = 0.0;
};
