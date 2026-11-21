// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "subsystems/elevator/Elevator.h"
#include "subsystems/pivot/Pivot.h"
#include "subsystems/wrist/Wrist.h"

/** Creates the subsystems and wires controller buttons to commands. */
class RobotContainer {
 public:
  RobotContainer();

  /** Called from Robot::RobotPeriodic(), after the CommandScheduler (so subsystems already ran). */
  void Periodic();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  void ConfigureTestBindings();

  frc2::CommandXboxController m_testController{OperatorConstants::kTestControllerPort};

  // Real robot -> talk to the real TalonFXs. Sim -> same TalonFX code, fed by a physics model.
  // Each sim needs live values from the other mechanisms, so they read them through lambdas.
  Elevator m_elevator;
  Pivot m_pivot;
  Wrist m_wrist;
};
