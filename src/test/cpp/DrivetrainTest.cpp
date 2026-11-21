// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

// The swerve drivetrain on Mayhem's physics: driver sticks, the elevator speed limit, and the
// autos driving on the REEFSCAPE field. Stepped time, so each test runs in about a second.

#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include "RobotHarness.h"
#include "commands/MayhemProject.h"
#include "sim/PhysicsWorld.h"
#include "sim/Reefscape2025.h"

namespace {

using Goal = Superstructure::Goal;

void Y(frc::sim::XboxControllerSim& c, bool down) { c.SetYButton(down); }

double Speed() {
  const auto v = sim::PhysicsWorld::Get().Swerve().FieldVelocity();
  return std::hypot(v.vx, v.vy);
}

}  // namespace

TEST(Drivetrain, StartsAtTheProjectStartPose) {
  RobotHarness h;
  h.Step(2);
  const auto pose = sim::PhysicsWorld::Get().TruePose();
  EXPECT_NEAR(pose.X().value(), mayhem_generated::startPose.x, 0.01);
  EXPECT_NEAR(pose.Y().value(), mayhem_generated::startPose.y, 0.01);
}

TEST(Drivetrain, LeftStickDrivesFieldCentric) {
  RobotHarness h;
  h.Teleop();
  h.Step(5);
  const double x0 = sim::PhysicsWorld::Get().TruePose().X().value();
  // Stick forward (up is negative Y): on blue, "forward" is toward the red wall, +x.
  h.Driver().SetLeftY(-1.0);
  h.Step(40);
  h.Driver().SetLeftY(0.0);
  h.Step(40);
  const auto pose = sim::PhysicsWorld::Get().TruePose();
  EXPECT_GT(pose.X().value() - x0, 1.0);
  EXPECT_NEAR(pose.Y().value(), mayhem_generated::startPose.y, 0.15);
  // The odometry estimate follows the true pose.
  const auto& est = h.C().GetDrivetrain().Pose();
  EXPECT_NEAR(est.X().value(), pose.X().value(), 0.1);
  // Let go: it stops.
  EXPECT_LT(Speed(), 0.2);
}

TEST(Drivetrain, ElevatorUpSlowsTheDriver) {
  RobotHarness h;
  h.Teleop();
  h.Step(5);
  h.Driver().SetLeftX(-1.0);  // strafe left (+y)
  h.Step(40);
  const double stowedSpeed = Speed();
  h.Driver().SetLeftX(0.0);
  h.Step(40);

  h.Tap(Y);
  ASSERT_TRUE(h.StepUntil([&] { return h.C().GetSuperstructure().IsAt(Goal::L4); }, 250));
  h.Driver().SetLeftX(-1.0);
  h.Step(40);
  const double extendedSpeed = Speed();
  EXPECT_GT(stowedSpeed, 2.5);
  EXPECT_LT(extendedSpeed, 0.5 * stowedSpeed);
  EXPECT_FALSE(sim::PhysicsWorld::Get().GetMetrics().tippedOver);
}

TEST(Drivetrain, ReefAutoDrivesUpAndScoresWithoutHittingAnything) {
  RobotHarness h{/*red=*/false, /*field=*/true};
  h.Step(2);
  h.C().GetAutoChooser().Select("Drive to reef and score L4");
  h.Autonomous();
  double xAtL4 = -1;
  for (int i = 0; i < 750; i++) {  // the 15 s auto period
    h.Step();
    if (xAtL4 < 0 && h.C().GetSuperstructure().IsAt(Goal::L4)) {
      xAtL4 = sim::PhysicsWorld::Get().TruePose().X().value();
    }
  }
  EXPECT_EQ(h.C().Autos().Status(), mayhem::RunStatus::Completed);
  // Raised to L4 right at the reef face, and ended backed off and stowed.
  EXPECT_NEAR(xAtL4, 5.85, 0.05);
  EXPECT_NEAR(sim::PhysicsWorld::Get().TruePose().X().value(), 6.8, 0.05);
  EXPECT_TRUE(h.C().GetSuperstructure().IsAt(Goal::STOW));
  const auto& m = sim::PhysicsWorld::Get().GetMetrics();
  EXPECT_EQ(m.wallHits + m.obstacleHits, 0) << (m.hits.empty() ? "" : m.hits.front());
  EXPECT_FALSE(m.tippedOver);
}

TEST(Drivetrain, RedAllianceRunsTheFlippedAuto) {
  RobotHarness h{/*red=*/true, /*field=*/true};
  h.Step(2);
  h.C().GetAutoChooser().Select("Drive to reef and score L4");
  h.Autonomous();
  h.Step(750);
  EXPECT_EQ(h.C().Autos().Status(), mayhem::RunStatus::Completed);
  // Same routine, mirrored through the field centre (REEFSCAPE is rotationally symmetric).
  const auto pose = sim::PhysicsWorld::Get().TruePose();
  EXPECT_NEAR(pose.X().value(), reefscape::kFieldLength - 6.8, 0.05);
  EXPECT_NEAR(pose.Y().value(), reefscape::kFieldWidth - 4.026, 0.05);
  const auto& m = sim::PhysicsWorld::Get().GetMetrics();
  EXPECT_EQ(m.wallHits + m.obstacleHits, 0) << (m.hits.empty() ? "" : m.hits.front());
}
