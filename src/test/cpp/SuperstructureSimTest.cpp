// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

// Runs the real subsystem code against the physics sims and checks the mechanisms move where
// they're told to. CTRE's simulated Talons run in real time, so each test takes a few seconds.

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include <frc/simulation/DriverStationSim.h>
#include <frc2/command/CommandScheduler.h>
#include <gtest/gtest.h>

#include "sim/SimBattery.h"
#include "subsystems/elevator/Elevator.h"
#include "subsystems/elevator/ElevatorIOSim.h"
#include "subsystems/pivot/Pivot.h"
#include "subsystems/pivot/PivotIOSim.h"
#include "subsystems/wrist/Wrist.h"
#include "subsystems/wrist/WristIOSim.h"

namespace {

struct SimRobot {
  // Built in the same order as RobotContainer, with the same cross-wiring.
  SimRobot()
      : elevator{true, std::make_unique<ElevatorIOSim>([this] { return pivot.GetAngleRad(); })},
        pivot{true, std::make_unique<PivotIOSim>([this] { return elevator.GetPositionMeters(); })},
        wrist{true, std::make_unique<WristIOSim>([this] { return pivot.GetAngleRad(); })} {
    elevator.SetPivotAngleSupplier([this] { return pivot.GetAngleRad(); });
    pivot.SetElevatorExtensionSupplier([this] { return elevator.GetPositionMeters(); });
    wrist.SetPivotAngleSupplier([this] { return pivot.GetAngleRad(); });
  }

  /** Runs the robot loop for `seconds` of real time (Phoenix sim is real-time). */
  void RunFor(double seconds) {
    int loops = static_cast<int>(seconds / 0.02);
    for (int i = 0; i < loops; i++) {
      frc2::CommandScheduler::GetInstance().Run();
      SimBattery::Update();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  Elevator elevator;
  Pivot pivot;
  Wrist wrist;
};

class SuperstructureSimTest : public testing::Test {
 protected:
  void SetUp() override {
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();
  }

  void TearDown() override {
    frc2::CommandScheduler::GetInstance().CancelAll();
    frc2::CommandScheduler::GetInstance().UnregisterAllSubsystems();
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
  }
};

}  // namespace

TEST_F(SuperstructureSimTest, ElevatorReachesL2AndPivotWristHold) {
  SimRobot robot;
  robot.RunFor(0.5);  // let the sim Talons come up

  EXPECT_NEAR(robot.pivot.GetAngleDeg(), 90.0, 1.5);
  EXPECT_NEAR(robot.wrist.GetAngleDeg(), 0.0, 2.0);
  EXPECT_NEAR(robot.elevator.GetPositionMeters(), 0.0, 0.02);

  frc2::CommandScheduler::GetInstance().Schedule(robot.elevator.GoToL2());
  robot.RunFor(2.5);

  EXPECT_NEAR(robot.elevator.GetPositionMeters(), 0.30, 0.02);
  EXPECT_TRUE(robot.elevator.ReachedPosition());
  // Upright pivot + in-line wrist have no gravity torque, so they should stay put.
  EXPECT_NEAR(robot.pivot.GetAngleDeg(), 90.0, 1.5);
  EXPECT_NEAR(robot.wrist.GetAngleDeg(), 0.0, 2.0);
}

TEST_F(SuperstructureSimTest, PivotAndWristReachSetpoints) {
  SimRobot robot;
  robot.RunFor(0.5);

  frc2::CommandScheduler::GetInstance().Schedule(robot.pivot.SetAngle(45.0));
  frc2::CommandScheduler::GetInstance().Schedule(robot.wrist.SetAngle(30.0));
  robot.RunFor(3.0);

  EXPECT_NEAR(robot.pivot.GetAngleDeg(), 45.0, 1.5);
  EXPECT_NEAR(robot.wrist.GetAngleDeg(), 30.0, 2.0);
}
