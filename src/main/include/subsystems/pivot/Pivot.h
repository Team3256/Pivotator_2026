// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <functional>
#include <memory>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/Trigger.h>

#include "subsystems/pivot/PivotConstants.h"
#include "subsystems/pivot/PivotIO.h"
#include "util/DisableSubsystem.h"

/** Main pivot that tilts the whole elevator. Angles are degrees above horizontal (90 = upright). */
class Pivot : public DisableSubsystem {
 public:
  Pivot(bool enabled, std::unique_ptr<PivotIO> io);

  void Periodic() override;

  frc2::CommandPtr SetAngle(double degrees);
  frc2::CommandPtr SetAngle(std::function<double()> degrees);
  frc2::CommandPtr SetVoltage(double volts);
  frc2::CommandPtr Off();

  /**
   * Tell the pivot "you are at the boot angle right now". Use in the pits if the robot was turned
   * on with the pivot in the wrong spot: push it onto the hard stop by hand, then run this.
   */
  frc2::CommandPtr RezeroAtBootAngle();

  double GetAngleDeg() const;
  double GetAngleRad() const;
  double GetReqAngleDeg() const;
  bool ReachedPosition() const;
  bool IsNear(double degrees) const;

  void SetElevatorExtensionSupplier(std::function<double()> elevatorMeters);

  frc2::Trigger reachedPosition{[this] { return ReachedPosition(); }};

 private:
  /** kG(extension) * cos(angle): biggest when the elevator is flat, zero when it's upright. */
  double GravityFeedforwardVolts() const;
  void ApplyAngle(double degrees);
  void LogInputs() const;

  std::unique_ptr<PivotIO> m_io;
  PivotIOInputs m_inputs;
  double m_reqAngleDeg = PivotConstants::kBootAngleDeg;

  /** Live elevator extension (meters), so gravity feedforward grows as the elevator extends. */
  std::function<double()> m_elevatorMeters = [] { return 0.0; };
};
