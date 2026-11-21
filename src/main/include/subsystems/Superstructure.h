// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <optional>

#include <frc2/command/SubsystemBase.h>

#include "commands/MayhemObserver.h"
#include "commands/MayhemProject.h"
#include "subsystems/SuperstructureConstants.h"
#include "subsystems/elevator/Elevator.h"
#include "subsystems/endeffector/EndEffector.h"
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
 *  - "EndEffector": the coral. At the coral intake the rollers pull one in (INTAKING) until the
 *    sensor sees it (HOLDING); RT at a reef level scores it (SCORING), and then the goal goes back
 *    to STOW. LT held spits it out anywhere (EJECTING).
 *
 * Every loop it fills the machines' inputs, Settle()s them, and sets the targets the four
 * subsystems' default commands follow. Both machines report to the Mayhem app live (NT) and in
 * logs, with every input, so the app's State machines page can replay why each transition fired.
 * It owns no motors: Elevator/Pivot/Wrist commands (manual jog, homing) still take over when run.
 */
class Superstructure : public frc2::SubsystemBase {
 public:
  using Goal = mayhem_generated::Superstructure::State;
  using Phase = mayhem_generated::Motion::State;
  using Coral = mayhem_generated::EndEffector::State;

  Superstructure(Elevator& elevator, Pivot& pivot, Wrist& wrist, EndEffector& endEffector);

  void Periodic() override;

  /** Asks for a new goal (a button press or an auto step). Used on the next Periodic(). */
  void Request(Goal goal);
  /** Score the held coral (RT). Only does anything at a reef level. */
  void RequestScore();
  /** LT: spit out whatever is in the end effector while held. */
  void SetEjectHeld(bool held);

  Goal CurrentGoal() const { return m_goal.Current(); }
  Phase CurrentPhase() const { return m_motion.Current(); }
  Coral CurrentCoral() const { return m_coral.Current(); }
  /** True when {@code goal} is the current goal and the mechanisms have reached it. */
  bool IsAt(Goal goal) const;

  double ElevatorTargetMeters() const { return m_elevatorTarget; }
  double PivotTargetDeg() const { return m_pivotTarget; }
  double WristTargetDeg() const { return m_wristTarget; }
  double RollerTargetVolts() const { return m_rollerTarget; }

 private:
  void UpdateTargets(const SuperstructureConstants::Targets& goal);

  Elevator& m_elevator;
  Pivot& m_pivot;
  Wrist& m_wrist;
  EndEffector& m_endEffector;

  MayhemObserver m_goalObserver{"Superstructure"};
  MayhemObserver m_motionObserver{"Motion"};
  MayhemObserver m_coralObserver{"EndEffector"};
  mayhem_generated::Superstructure::Machine m_goal;
  mayhem_generated::Motion::Machine m_motion;
  mayhem_generated::EndEffector::Machine m_coral;

  std::optional<Goal> m_pending;
  bool m_scorePending = false;
  bool m_ejectHeld = false;
  /** The coral left last loop (SCORING -> EMPTY): the goal goes back to STOW. */
  bool m_justScored = false;
  double m_rollerTarget = 0.0;
  bool m_pivotFar = false;
  bool m_atGoal = false;
  double m_pivotHoldDeg;
  double m_elevatorTarget;
  double m_pivotTarget;
  double m_wristTarget;
};
