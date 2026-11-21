// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "Robot.h"

#include <ctre/phoenix6/SignalLogger.hpp>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc2/command/CommandScheduler.h>

#include "sim/SimBattery.h"

Robot::Robot() {
  // Logging replaces AdvantageKit: every value from Log::Record goes to NetworkTables (live view
  // in AdvantageScope) and DataLogManager saves all of NetworkTables + DS/joystick data to a
  // .wpilog (roboRIO: USB stick if plugged in, else /home/lvuser/logs).
  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());

  // We log Phoenix signals ourselves already, so skip CTRE's separate hoot logs.
  ctre::phoenix6::SignalLogger::EnableAutoLogging(false);
  if (IsSimulation()) {
    // In sim there's usually no controller plugged in; don't spam the console about it.
    frc::DriverStation::SilenceJoystickConnectionWarning(true);
  }

  m_container.emplace();
}

void Robot::RobotPeriodic() {
  // Runs buttons, commands, and every subsystem's Periodic(). Nothing works without this.
  frc2::CommandScheduler::GetInstance().Run();
  m_container->Periodic();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container->GetAutonomousCommand();
  if (m_autonomousCommand) {
    frc2::CommandScheduler::GetInstance().Schedule(m_autonomousCommand.value());
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {
  // Sag the sim battery using every mechanism's current draw from this loop.
  SimBattery::Update();
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
