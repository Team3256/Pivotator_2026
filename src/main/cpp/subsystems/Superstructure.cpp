// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "subsystems/Superstructure.h"

#include <algorithm>
#include <cmath>
#include <string>

#include <frc/DriverStation.h>
#include <frc/Timer.h>

#include "subsystems/endeffector/EndEffectorConstants.h"
#include "util/Log.h"

namespace SC = SuperstructureConstants;

namespace {
double Now() {
  return frc::Timer::GetFPGATimestamp().value();
}
}  // namespace

Superstructure::Superstructure(Elevator& elevator, Pivot& pivot, Wrist& wrist,
                               EndEffector& endEffector)
    : frc2::SubsystemBase("Superstructure"),
      m_elevator{elevator},
      m_pivot{pivot},
      m_wrist{wrist},
      m_endEffector{endEffector} {
  // The clock times the machines and stamps what they report; InputLogging::On logs every input
  // each loop so log replay can show why a transition fired.
  m_goal.Attach(m_goalObserver, Now, mayhem::InputLogging::On);
  m_motion.Attach(m_motionObserver, Now, mayhem::InputLogging::On);
  m_coral.Attach(m_coralObserver, Now, mayhem::InputLogging::On);
  m_pivotHoldDeg = pivot.GetReqAngleDeg();
  UpdateTargets(SC::For(m_goal.Current()));
}

void Superstructure::Request(Goal goal) {
  m_pending = goal;
}

void Superstructure::RequestScore() {
  m_scorePending = true;
}

void Superstructure::SetEjectHeld(bool held) {
  m_ejectHeld = held;
}

bool Superstructure::IsAt(Goal goal) const {
  return m_goal.Current() == goal && m_motion.Current() == Phase::AT_GOAL;
}

void Superstructure::Periodic() {
  const bool enabled = frc::DriverStation::IsEnabled();

  // 1. The goal: button presses / auto requests are one-loop events.
  auto& goalIn = m_goal.In();
  goalIn.enabled = enabled;
  goalIn.stowPressed = m_pending == Goal::STOW;
  goalIn.intakePressed = m_pending == Goal::CORAL_INTAKE;
  goalIn.l1Pressed = m_pending == Goal::L1;
  goalIn.l2Pressed = m_pending == Goal::L2;
  goalIn.l3Pressed = m_pending == Goal::L3;
  goalIn.l4Pressed = m_pending == Goal::L4;
  goalIn.scored = m_justScored;
  m_pending.reset();
  m_justScored = false;
  m_goal.Settle();
  const SC::Targets goal = SC::For(m_goal.Current());

  // 2. Where the mechanisms are relative to it. Both flags have hysteresis so the Motion machine
  // doesn't flicker while a mechanism settles at the edge of a tolerance.
  const double pivotErr = std::abs(m_pivot.GetAngleDeg() - goal.pivotDeg);
  const double elevatorErr = std::abs(m_elevator.GetPositionMeters() - goal.elevatorMeters);
  const double wristErr = std::abs(m_wrist.GetAngleDeg() - goal.wristDeg);
  m_pivotFar = m_pivotFar ? pivotErr > SC::kPivotSettledDeg : pivotErr > SC::kPivotMoveDeg;
  if (m_atGoal) {
    m_atGoal = elevatorErr <= SC::kLeaveGoalElevatorMeters && pivotErr <= SC::kLeaveGoalPivotDeg &&
               wristErr <= SC::kLeaveGoalWristDeg;
  } else {
    m_atGoal = elevatorErr <= SC::kAtGoalElevatorMeters && pivotErr <= SC::kAtGoalPivotDeg &&
               wristErr <= SC::kAtGoalWristDeg;
  }

  // 3. How to get there.
  const Phase before = m_motion.Current();
  auto& motionIn = m_motion.In();
  motionIn.enabled = enabled;
  motionIn.pivotFar = m_pivotFar;
  motionIn.elevatorSafe = m_elevator.IsSafeForPivot();
  motionIn.atGoal = m_atGoal;
  m_motion.Settle();
  if (m_motion.Current() == Phase::RETRACTING && before != Phase::RETRACTING) {
    // Hold the pivot where it was last sent while the elevator comes in.
    m_pivotHoldDeg = m_pivot.GetReqAngleDeg();
  }
  UpdateTargets(goal);

  // 4. The coral.
  const Goal current = m_goal.Current();
  const bool atGoal = m_motion.Current() == Phase::AT_GOAL;
  const Coral coralBefore = m_coral.Current();
  auto& coralIn = m_coral.In();
  coralIn.enabled = enabled;
  coralIn.hasCoral = m_endEffector.HasCoral();
  coralIn.scorePressed = m_scorePending;
  coralIn.ejectHeld = m_ejectHeld;
  coralIn.atIntake = current == Goal::CORAL_INTAKE && atGoal;
  coralIn.atReefLevel = (current == Goal::L1 || current == Goal::L2 || current == Goal::L3 ||
                         current == Goal::L4) &&
                        atGoal;
  m_scorePending = false;
  m_coral.Settle();
  m_justScored = coralBefore == Coral::SCORING && m_coral.Current() == Coral::EMPTY;
  switch (m_coral.Current()) {
    case Coral::EMPTY:
      m_rollerTarget = 0.0;
      break;
    case Coral::INTAKING:
      m_rollerTarget = EndEffectorConstants::kIntakeVolts;
      break;
    case Coral::HOLDING:
      m_rollerTarget = EndEffectorConstants::kHoldVolts;
      break;
    case Coral::SCORING:
      m_rollerTarget = EndEffectorConstants::kScoreVolts;
      break;
    case Coral::EJECTING:
      m_rollerTarget = EndEffectorConstants::kEjectVolts;
      break;
  }

  Log::Record("Superstructure/Goal", m_goal.StateName());
  Log::Record("Superstructure/Motion", m_motion.StateName());
  Log::Record("Superstructure/Coral", m_coral.StateName());
  Log::Record("Superstructure/RollerTargetVolts", m_rollerTarget);
  Log::Record("Superstructure/ElevatorTargetMeters", m_elevatorTarget);
  Log::Record("Superstructure/PivotTargetDeg", m_pivotTarget);
  Log::Record("Superstructure/WristTargetDeg", m_wristTarget);
}

void Superstructure::UpdateTargets(const SC::Targets& goal) {
  m_wristTarget = goal.wristDeg;
  switch (m_motion.Current()) {
    case Phase::RETRACTING:
      m_elevatorTarget = std::min(goal.elevatorMeters, SC::kElevatorHoldMeters);
      m_pivotTarget = m_pivotHoldDeg;
      break;
    case Phase::PIVOTING:
      m_elevatorTarget = std::min(goal.elevatorMeters, SC::kElevatorHoldMeters);
      m_pivotTarget = goal.pivotDeg;
      break;
    case Phase::IDLE:
    case Phase::EXTENDING:
    case Phase::AT_GOAL:
      m_elevatorTarget = goal.elevatorMeters;
      m_pivotTarget = goal.pivotDeg;
      break;
  }
}
