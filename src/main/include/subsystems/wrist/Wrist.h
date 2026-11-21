// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <functional>
#include <memory>
#include <numbers>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/Trigger.h>

#include "subsystems/wrist/WristConstants.h"
#include "subsystems/wrist/WristIO.h"
#include "util/DisableSubsystem.h"

/** Wrist on the carriage. Angles are degrees relative to the elevator (0 = in line with it). */
class Wrist : public DisableSubsystem {
 public:
  Wrist(bool enabled, std::unique_ptr<WristIO> io);

  void Periodic() override;

  frc2::CommandPtr SetAngle(double degrees);
  frc2::CommandPtr SetAngle(std::function<double()> degrees);
  frc2::CommandPtr SetVoltage(double volts);
  frc2::CommandPtr Off();

  /** Pit tool: hold the wrist at its boot position by hand, then run this. */
  frc2::CommandPtr RezeroAtBootAngle();

  double GetAngleDeg() const;
  double GetAngleRad() const;
  bool ReachedPosition() const;
  bool IsNear(double degrees) const;

  void SetPivotAngleSupplier(std::function<double()> pivotAngleRad);

  frc2::Trigger reachedPosition{[this] { return ReachedPosition(); }};

 private:
  double GravityFeedforwardVolts() const;
  void ApplyAngle(double degrees);
  void LogInputs() const;

  std::unique_ptr<WristIO> m_io;
  WristIOInputs m_inputs;
  double m_reqAngleDeg = WristConstants::kBootAngleDeg;
  std::function<double()> m_pivotAngleRad = [] { return std::numbers::pi / 2.0; };
};
