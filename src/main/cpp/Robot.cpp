// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "Robot.h"

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

#include <ctre/phoenix6/SignalLogger.hpp>
#include <frc/DriverStation.h>
#include <frc/Filesystem.h>
#include <frc2/command/CommandScheduler.h>

#include "sim/PhysicsWorld.h"
#include "util/Log.h"

namespace {

/**
 * Where the .wpilog goes. Robot: WPILib's default (USB stick, else /home/lvuser/logs). Sim:
 * ROBOT_LOG_DIR when set (Mayhem's sim proof sets it), else ./logs; tests that set ROBOT_NO_LOG
 * skip the file.
 */
void StartLogging(bool real) {
  if (real) {
    Log::Start();
  } else if (const char* dir = std::getenv("ROBOT_LOG_DIR")) {
    Log::Start(dir);
  } else if (std::getenv("ROBOT_NO_LOG") == nullptr) {
    Log::Start("logs");
  }
}

/**
 * The build stamp (deploy/mayhem-build.json, written by the build): which commit this is. Logged
 * once as Mayhem/Build so the Mayhem app can tell which code a log ran and what's on the robot.
 */
std::string ReadBuildStamp() {
  std::ifstream in{frc::filesystem::GetDeployDirectory() + "/mayhem-build.json"};
  std::stringstream text;
  text << in.rdbuf();
  return text.str();
}

}  // namespace

Robot::Robot() {
  StartLogging(IsReal());
  Log::Record("Mayhem/Build", ReadBuildStamp());

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
  m_container->Autos().Start();
}

void Robot::AutonomousPeriodic() {
  m_container->Autos().Periodic();
}

void Robot::AutonomousExit() {
  m_container->Autos().Stop();
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {
  // One loop of physics for every simulated mechanism and the battery (Mayhem's engine).
  sim::PhysicsWorld::Get().Step(GetPeriod().value());
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
