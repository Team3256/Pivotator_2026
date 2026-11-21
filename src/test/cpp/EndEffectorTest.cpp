// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

// The coral flow on Mayhem's physics: intake at the coral station, hold, score at a reef level,
// eject. Stepped time, so each test runs in well under a second.

#include <gtest/gtest.h>

#include "RobotHarness.h"
#include "sim/PhysicsWorld.h"
#include "subsystems/elevator/ElevatorConstants.h"

namespace {

using Goal = Superstructure::Goal;
using Coral = Superstructure::Coral;

void A(frc::sim::XboxControllerSim& c, bool down) { c.SetAButton(down); }
void B(frc::sim::XboxControllerSim& c, bool down) { c.SetBButton(down); }
void Y(frc::sim::XboxControllerSim& c, bool down) { c.SetYButton(down); }
void RB(frc::sim::XboxControllerSim& c, bool down) { c.SetRightBumperButton(down); }
void RT(frc::sim::XboxControllerSim& c, bool down) { c.SetRightTriggerAxis(down ? 1.0 : 0.0); }

constexpr int k5Seconds = 250;  // loops

/** Enabled in teleop, settled at STOW. */
void Ready(RobotHarness& h) {
  h.Teleop();
  ASSERT_TRUE(h.StepUntil([&] { return h.C().GetSuperstructure().IsAt(Goal::STOW); }, 50));
}

}  // namespace

TEST(EndEffector, StartsHoldingThePreload) {
  RobotHarness h;
  h.Teleop();
  h.Step(5);
  EXPECT_TRUE(h.C().GetEndEffector().HasCoral());
  EXPECT_EQ(h.C().GetSuperstructure().CurrentCoral(), Coral::HOLDING);
}

TEST(EndEffector, ScoresAtL4AndStowsAfter) {
  RobotHarness h;
  Ready(h);
  h.Tap(Y);
  ASSERT_TRUE(h.StepUntil([&] { return h.C().GetSuperstructure().IsAt(Goal::L4); }, k5Seconds));
  h.Tap(RT);
  ASSERT_TRUE(h.StepUntil([&] { return h.C().GetSuperstructure().CurrentCoral() == Coral::EMPTY; },
                          100));
  const auto& m = sim::PhysicsWorld::Get().GetMetrics();
  ASSERT_EQ(m.scoredOwnHub, 1);
  // Scored from the L4 height.
  EXPECT_NEAR(m.scoredHeights.at(0), ElevatorConstants::Meters(ElevatorConstants::Height::kL4),
              0.03);
  // Scoring sends the goal back to STOW by itself.
  ASSERT_TRUE(h.StepUntil([&] { return h.C().GetSuperstructure().IsAt(Goal::STOW); }, k5Seconds));
  EXPECT_FALSE(h.C().GetEndEffector().HasCoral());
}

TEST(EndEffector, ScoreDoesNothingAwayFromTheReef) {
  RobotHarness h;
  Ready(h);
  h.Tap(RT);  // at STOW
  h.Step(50);
  EXPECT_EQ(h.C().GetSuperstructure().CurrentCoral(), Coral::HOLDING);
  EXPECT_TRUE(h.C().GetEndEffector().HasCoral());
}

TEST(EndEffector, IntakesAtTheCoralStation) {
  RobotHarness h;
  sim::PhysicsWorld::Get().SetHasCoral(false);  // no preload
  Ready(h);
  EXPECT_EQ(h.C().GetSuperstructure().CurrentCoral(), Coral::EMPTY);
  h.Tap(RB);
  bool sawIntaking = false;
  ASSERT_TRUE(h.StepUntil(
      [&] {
        sawIntaking |= h.C().GetSuperstructure().CurrentCoral() == Coral::INTAKING;
        return h.C().GetSuperstructure().CurrentCoral() == Coral::HOLDING;
      },
      k5Seconds));
  EXPECT_TRUE(sawIntaking);
  EXPECT_EQ(sim::PhysicsWorld::Get().GetMetrics().intaked, 1);
  // Then score it at L2.
  h.Tap(B);
  ASSERT_TRUE(h.StepUntil([&] { return h.C().GetSuperstructure().IsAt(Goal::L2); }, k5Seconds));
  h.Tap(RT);
  ASSERT_TRUE(h.StepUntil([&] { return h.C().GetSuperstructure().CurrentCoral() == Coral::EMPTY; },
                          100));
  EXPECT_EQ(sim::PhysicsWorld::Get().GetMetrics().scoredOwnHub, 1);
}

TEST(EndEffector, LeftTriggerEjects) {
  RobotHarness h;
  Ready(h);
  h.Driver().SetLeftTriggerAxis(1.0);
  ASSERT_TRUE(h.StepUntil([&] { return !h.C().GetEndEffector().HasCoral(); }, 100));
  EXPECT_EQ(h.C().GetSuperstructure().CurrentCoral(), Coral::EJECTING);
  h.Driver().SetLeftTriggerAxis(0.0);
  h.Step(3);
  EXPECT_EQ(h.C().GetSuperstructure().CurrentCoral(), Coral::EMPTY);
}
