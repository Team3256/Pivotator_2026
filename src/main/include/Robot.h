// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <optional>

#include <frc/TimedRobot.h>

#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void AutonomousExit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

  RobotContainer& Container() { return *m_container; }

 private:
  // optional so it's created in the Robot() body AFTER logging starts (a plain member would be
  // built before the constructor body runs).
  std::optional<RobotContainer> m_container;
};
