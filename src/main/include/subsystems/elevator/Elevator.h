// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <functional>
#include <memory>

#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/Trigger.h>

#include "subsystems/elevator/ElevatorConstants.h"
#include "subsystems/elevator/ElevatorIO.h"
#include "util/DisableSubsystem.h"

class Elevator : public DisableSubsystem {
 public:
  Elevator(bool enabled, std::unique_ptr<ElevatorIO> io);

  void Periodic() override;

  // ---------------- Commands ----------------

  /** Drive to a fixed height (meters). Runs until interrupted; holds position the whole time. */
  frc2::CommandPtr SetPosition(double meters);

  /** Drive to a height that can change every loop (e.g. from a joystick or a shot calculator). */
  frc2::CommandPtr SetPosition(std::function<double()> meters);

  /** Drive to one of the named coral heights. */
  frc2::CommandPtr SetPosition(ElevatorConstants::Height height);

  /** Same as SetPosition(Height) but ends once the carriage gets there. */
  frc2::CommandPtr GoToAndWait(ElevatorConstants::Height height);

  frc2::CommandPtr GoToStow();
  frc2::CommandPtr GoToCoralIntake();
  frc2::CommandPtr GoToL1();
  frc2::CommandPtr GoToL2();
  frc2::CommandPtr GoToL3();
  frc2::CommandPtr GoToL4();

  /** Open-loop voltage, mostly for tuning / SysId. Soft limits still protect the mechanism. */
  frc2::CommandPtr SetVoltage(double volts);

  /** Manual jog from a joystick axis in [-1, 1], scaled to +/- maxVolts. */
  frc2::CommandPtr Manual(std::function<double()> axis, double maxVolts);

  frc2::CommandPtr Off();

  /**
   * Slowly drive down into the hard stop (ignoring soft limits), wait for a current spike, then
   * call that 0. Use this in pit / test mode or if the robot was powered on with the carriage up.
   */
  frc2::CommandPtr Home();

  // ---------------- State ----------------

  double GetPositionMeters() const;
  double GetReqPositionMeters() const;
  bool ReachedPosition() const;

  /** True if the carriage is currently near the given height (regardless of what was requested). */
  bool IsAt(ElevatorConstants::Height height) const;
  frc2::Trigger IsAtTrigger(ElevatorConstants::Height height);

  /**
   * True when the elevator is pulled in far enough that the pivot is allowed to swing. Swinging a
   * fully extended elevator puts huge torque on the pivot and can tip the robot.
   */
  bool IsSafeForPivot() const;

  /** Give the elevator the live pivot angle so its gravity feedforward follows the tilt. */
  void SetPivotAngleSupplier(std::function<double()> pivotAngleRad);

  bool IsHomed() const;

  /** True once the carriage is within tolerance of the last requested height. */
  frc2::Trigger reachedPosition{[this] { return ReachedPosition(); }};

  /** True when the pivot is allowed to swing (see IsSafeForPivot). */
  frc2::Trigger isSafeForPivot{[this] { return IsSafeForPivot(); }};

 private:
  /** Volts needed to hold the carriage at the current tilt: kG(vertical) * sin(pivot angle). */
  double GravityFeedforwardVolts() const;
  void ApplyPosition(double meters);
  void LogInputs() const;

  std::unique_ptr<ElevatorIO> m_io;
  ElevatorIOInputs m_inputs;

  double m_reqPositionMeters = ElevatorConstants::kMinHeightMeters;
  bool m_homed = true;  // IO zeroes at boot assuming the carriage starts at the bottom

  /** Live pivot angle (radians above horizontal). Set by RobotContainer once the Pivot exists. */
  std::function<double()> m_pivotAngleRad;

  // Simple side-view visual (works on real robot and in sim). Shows up under SmartDashboard.
  frc::Mechanism2d m_mech{2.0, 2.0};
  frc::MechanismLigament2d* m_carriage;
};
