// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <optional>

#include <frc2/command/SubsystemBase.h>

#include "commands/MayhemObserver.h"
#include "commands/MayhemProject.h"
#include "subsystems/SuperstructureConstants.h"
#include "subsystems/elevator/Elevator.h"
#include "subsystems/pivot/Pivot.h"
#include "subsystems/wrist/Wrist.h"

/**
 * Coordinates the pivot, elevator and wrist with two Mayhem state machines (generated from
 * robot.mayhem.json into commands/MayhemProject.h, so a misspelled state or input won't compile):
 *
 *  - "Superstructure": the goal the driver (or an auto) asked for: STOW, CORAL_INTAKE, L1..L4.
 *  - "Motion": how to get there safely. The pivot swings the whole elevator, so a big pivot move
 *    first pulls the elevator in (RETRACTING), then swings (PIVOTING), then extends (EXTENDING)
 *    until everything is there (AT_GOAL).
 *
 * Every loop it fills both machines' inputs, Settle()s them, and sets the targets the three
 * subsystems' default commands follow. Both machines report to the Mayhem app live (NT) and in
 * logs, with every input, so the app's State machines page can replay why each transition fired.
 * It owns no motors: Elevator/Pivot/Wrist commands (manual jog, homing) still take over when run.
 */
class Superstructure : public frc2::SubsystemBase {
 public:
  using Goal = mayhem_generated::Superstructure::State;
  using Phase = mayhem_generated::Motion::State;

  Superstructure(Elevator& elevator, Pivot& pivot, Wrist& wrist);

  void Periodic() override;

  /** Asks for a new goal (a button press or an auto step). Used on the next Periodic(). */
  void Request(Goal goal);

  Goal CurrentGoal() const { return m_goal.Current(); }
  Phase CurrentPhase() const { return m_motion.Current(); }
  /** True when {@code goal} is the current goal and the mechanisms have reached it. */
  bool IsAt(Goal goal) const;

  double ElevatorTargetMeters() const { return m_elevatorTarget; }
  double PivotTargetDeg() const { return m_pivotTarget; }
  double WristTargetDeg() const { return m_wristTarget; }

 private:
  void UpdateTargets(const SuperstructureConstants::Targets& goal);

  Elevator& m_elevator;
  Pivot& m_pivot;
  Wrist& m_wrist;

  MayhemObserver m_goalObserver{"Superstructure"};
  MayhemObserver m_motionObserver{"Motion"};
  mayhem_generated::Superstructure::Machine m_goal;
  mayhem_generated::Motion::Machine m_motion;

  std::optional<Goal> m_pending;
  bool m_pivotFar = false;
  bool m_atGoal = false;
  double m_pivotHoldDeg;
  double m_elevatorTarget;
  double m_pivotTarget;
  double m_wristTarget;
};
