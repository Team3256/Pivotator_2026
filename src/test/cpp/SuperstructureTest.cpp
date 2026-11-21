// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

// The whole robot on Mayhem's physics: driver buttons -> Superstructure machines -> subsystems ->
// simulated TalonFXs -> mechanisms. Runs on stepped time, so each test takes well under a second.

#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include "RobotHarness.h"
#include "commands/MayhemProject.h"
#include "sim/PhysicsWorld.h"
#include "subsystems/elevator/ElevatorConstants.h"

namespace {

using Goal = Superstructure::Goal;
using Phase = Superstructure::Phase;
namespace SC = SuperstructureConstants;

void PressA(frc::sim::XboxControllerSim& c, bool down) { c.SetAButton(down); }
void PressB(frc::sim::XboxControllerSim& c, bool down) { c.SetBButton(down); }
void PressY(frc::sim::XboxControllerSim& c, bool down) { c.SetYButton(down); }
void PressRB(frc::sim::XboxControllerSim& c, bool down) { c.SetRightBumperButton(down); }

constexpr int k5Seconds = 250;  // loops

double Deg(double rad) {
  return rad * 180.0 / M_PI;
}

/** The true mechanism state is at the goal's targets. */
void ExpectAtTargets(Goal goal) {
  auto& world = sim::PhysicsWorld::Get();
  const auto t = SC::For(goal);
  EXPECT_NEAR(world.ElevatorMeters(), t.elevatorMeters, 0.02);
  EXPECT_NEAR(Deg(world.PivotAngleRad()), t.pivotDeg, 2.0);
  EXPECT_NEAR(Deg(world.WristAngleRad()), t.wristDeg, 3.0);
}

}  // namespace

TEST(Superstructure, StartsStowedAndIdleWhileDisabled) {
  RobotHarness h;
  h.Step(5);
  EXPECT_EQ(h.C().GetSuperstructure().CurrentGoal(), Goal::STOW);
  EXPECT_EQ(h.C().GetSuperstructure().CurrentPhase(), Phase::IDLE);
}

TEST(Superstructure, StowIsAlreadyThereOnEnable) {
  RobotHarness h;
  h.Teleop();
  ASSERT_TRUE(h.StepUntil([&] { return h.C().GetSuperstructure().IsAt(Goal::STOW); }, 25));
  ExpectAtTargets(Goal::STOW);
}

TEST(Superstructure, BGoesToL2PivotingBeforeExtending) {
  RobotHarness h;
  h.Teleop();
  h.Step(5);
  h.Tap(PressB);  // 90 -> 80 degrees: a pivot move big enough to sequence
  EXPECT_EQ(h.C().GetSuperstructure().CurrentGoal(), Goal::L2);

  bool sawPivoting = false;
  const double safe = ElevatorConstants::kPivotSafeMaxHeightMeters;
  ASSERT_TRUE(h.StepUntil(
      [&] {
        auto& world = sim::PhysicsWorld::Get();
        const double pivotErr = std::abs(Deg(world.PivotAngleRad()) - SC::For(Goal::L2).pivotDeg);
        // The elevator never goes out while the pivot still has a big move to make.
        if (pivotErr > SC::kPivotMoveDeg) {
          EXPECT_LT(world.ElevatorMeters(), safe);
        }
        sawPivoting |= h.C().GetSuperstructure().CurrentPhase() == Phase::PIVOTING;
        return h.C().GetSuperstructure().IsAt(Goal::L2);
      },
      k5Seconds));
  EXPECT_TRUE(sawPivoting);
  ExpectAtTargets(Goal::L2);
}

TEST(Superstructure, YGoesToL4) {
  RobotHarness h;
  h.Teleop();
  h.Step(5);
  h.Tap(PressY);
  EXPECT_EQ(h.C().GetSuperstructure().CurrentGoal(), Goal::L4);
  ASSERT_TRUE(h.StepUntil([&] { return h.C().GetSuperstructure().IsAt(Goal::L4); }, k5Seconds));
  ExpectAtTargets(Goal::L4);
}

TEST(Superstructure, StowFromL4RetractsBeforeThePivotMoves) {
  RobotHarness h;
  h.Teleop();
  h.Step(5);
  h.Tap(PressY);
  ASSERT_TRUE(h.StepUntil([&] { return h.C().GetSuperstructure().IsAt(Goal::L4); }, k5Seconds));

  // Coral intake is a big pivot move (88 -> 60 degrees) with the elevator out at L4.
  h.Tap(PressRB);
  bool sawRetracting = false;
  const double l4Pivot = SC::For(Goal::L4).pivotDeg;
  const double safe = ElevatorConstants::kPivotSafeMaxHeightMeters;
  ASSERT_TRUE(h.StepUntil(
      [&] {
        auto& world = sim::PhysicsWorld::Get();
        // While the elevator is still out, the pivot holds where it was.
        if (world.ElevatorMeters() > safe) {
          EXPECT_NEAR(Deg(world.PivotAngleRad()), l4Pivot, 2.0);
        }
        sawRetracting |= h.C().GetSuperstructure().CurrentPhase() == Phase::RETRACTING;
        return h.C().GetSuperstructure().IsAt(Goal::CORAL_INTAKE);
      },
      k5Seconds));
  EXPECT_TRUE(sawRetracting);
  ExpectAtTargets(Goal::CORAL_INTAKE);

  h.Tap(PressA);
  ASSERT_TRUE(h.StepUntil([&] { return h.C().GetSuperstructure().IsAt(Goal::STOW); }, k5Seconds));
  ExpectAtTargets(Goal::STOW);
}

TEST(Superstructure, DisablingGoesIdleAndBackToStow) {
  RobotHarness h;
  h.Teleop();
  h.Step(5);
  h.Tap(PressB);
  ASSERT_TRUE(h.StepUntil([&] { return h.C().GetSuperstructure().IsAt(Goal::L2); }, k5Seconds));
  h.Disable();
  h.Step(2);
  EXPECT_EQ(h.C().GetSuperstructure().CurrentPhase(), Phase::IDLE);
  EXPECT_EQ(h.C().GetSuperstructure().CurrentGoal(), Goal::STOW);
}

TEST(Superstructure, EveryAutoCompletes) {
  for (const auto& routine : mayhem_generated::routines) {
    SCOPED_TRACE(std::string{routine.name});
    RobotHarness h;
    h.Step(2);
    h.C().GetAutoChooser().Select(std::string{routine.name});
    ASSERT_EQ(h.C().GetAutoChooser().Selected(), routine.name);
    h.Autonomous();
    // The 15 s REEFSCAPE auto period.
    bool reachedL4 = false;
    for (int i = 0; i < 750; i++) {
      h.Step();
      reachedL4 |= h.C().GetSuperstructure().IsAt(Goal::L4);
    }
    EXPECT_TRUE(reachedL4);
    EXPECT_EQ(h.C().Autos().Status(), mayhem::RunStatus::Completed);
    EXPECT_TRUE(h.C().GetSuperstructure().IsAt(Goal::STOW));
    // The preloaded coral was scored.
    EXPECT_EQ(sim::PhysicsWorld::Get().GetMetrics().scoredOwnHub, 1);
  }
}

TEST(Superstructure, BatteryStaysHealthy) {
  RobotHarness h;
  h.Teleop();
  h.Step(5);
  h.Tap(PressY);
  h.Step(k5Seconds);
  h.Tap(PressA);
  h.Step(k5Seconds);
  const auto& m = sim::PhysicsWorld::Get().GetMetrics();
  EXPECT_FALSE(m.brownedOut);
  EXPECT_GT(m.minBatteryVolts, 9.0);
}
