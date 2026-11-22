// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <functional>
#include <memory>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/Trigger.h>

#include "subsystems/endeffector/EndEffectorIO.h"
#include "util/DisableSubsystem.h"

/**
 * End effector: top roller + side wheels, and the coral sensor. Positive voltage pulls coral in.
 * Commands set the top roller's voltage; the side wheels get it scaled by kSideVoltsScale so both
 * move the coral at the same speed.
 */
class EndEffector : public DisableSubsystem {
 public:
  EndEffector(bool enabled, std::unique_ptr<EndEffectorIO> io);

  void Periodic() override;

  /** Runs the rollers at a voltage that can change every loop (the Superstructure's target). */
  frc2::CommandPtr SetVoltage(std::function<double()> volts);
  frc2::CommandPtr SetVoltage(double volts);
  frc2::CommandPtr Off();

  bool HasCoral() const { return m_inputs.hasCoral; }
  /** Roller rotations per second. */
  double TopRollerRps() const { return m_inputs.top.velocityRps; }
  double SideWheelRps() const { return m_inputs.sideLeader.velocityRps; }

  frc2::Trigger hasCoral{[this] { return HasCoral(); }};

 private:
  void LogInputs() const;
  void Apply(double volts);

  std::unique_ptr<EndEffectorIO> m_io;
  EndEffectorIOInputs m_inputs;
  double m_reqVolts = 0.0;
};
