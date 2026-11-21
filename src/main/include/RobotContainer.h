// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <ctre/phoenix6/swerve/SwerveRequest.hpp>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "commands/MayhemAutos.h"
#include "subsystems/Superstructure.h"
#include "subsystems/elevator/Elevator.h"
#include "subsystems/pivot/Pivot.h"
#include "subsystems/swerve/Drivetrain.h"
#include "subsystems/wrist/Wrist.h"

/** Creates the subsystems and wires controller buttons to commands. */
class RobotContainer {
 public:
  RobotContainer();
  ~RobotContainer();

  /**
   * Called from Robot::RobotPeriodic(), after the CommandScheduler (so subsystems already ran):
   * the driver's drive request in teleop, drivetrain logging, the 3D poses.
   */
  void Periodic();
  /** Called at the end of every robot loop (the drivetrain re-reads its state next loop). */
  void EndLoop();

  MayhemAutos& Autos() { return m_autos; }
  /** The autonomous chooser (Mayhem's sim proof selects autos through this). */
  AutoChooser& GetAutoChooser() { return m_autos.Chooser(); }
  Drivetrain& GetDrivetrain() { return m_drivetrain; }
  Superstructure& GetSuperstructure() { return m_superstructure; }
  Elevator& GetElevator() { return m_elevator; }
  Pivot& GetPivot() { return m_pivot; }
  Wrist& GetWrist() { return m_wrist; }

 private:
  void ConfigureBindings();
  /** 1 with the elevator in, down to SwerveConstants::kExtendedSpeedScale fully extended. */
  double DriverSpeedScale();

  frc2::CommandXboxController m_driverController{OperatorConstants::kDriverControllerPort};

  // The drivetrain first: it's shut down first in ~RobotContainer and destroyed last.
  Drivetrain m_drivetrain;
  // Real robot -> real TalonFXs. Sim -> the same TalonFX code, simulated on sim::PhysicsWorld.
  Elevator m_elevator;
  Pivot m_pivot;
  Wrist m_wrist;
  Superstructure m_superstructure;
  MayhemAutos m_autos;

  ctre::phoenix6::swerve::requests::FieldCentric m_driveRequest;
};
