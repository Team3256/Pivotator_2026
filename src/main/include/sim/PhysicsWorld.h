// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>

#include "mayhem/sim/battery.hpp"
#include "mayhem/sim/mechanisms.hpp"
#include "mayhem/sim/talonfx.hpp"

namespace sim {

/** The simulated mechanisms, one physics plant each. */
enum class Mechanism { kPivot, kElevator, kWrist };

/**
 * What happened during a run, for tests and Mayhem's sim proof (its probe reads these fields).
 * The drivetrain/field/game-piece counters stay 0: this robot has no drivetrain yet.
 */
struct Metrics {
  int shotsFired = 0, intaked = 0, scoredOwnHub = 0;
  int wallHits = 0, obstacleHits = 0;
  std::vector<std::string> hits;
  double beachedSeconds = 0, minBatteryVolts = 99, maxTotalCurrent = 0;
  bool tippedOver = false, brownedOut = false;
};

/**
 * The simulated robot: Mayhem's physics engine (mayhem::sim, vendored in src/main/include/mayhem)
 * running the pivot, elevator, wrist and battery on simulated time.
 *
 * The robot code keeps commanding its real Phoenix TalonFX objects. Each loop the world reads
 * every motor's applied control (TalonFX::GetAppliedControl) into Mayhem's TalonFX emulator,
 * configured from the same TalonFXConfiguration, so Motion Magic, the gains, current limits,
 * soft limits and followers all run as they would on the motor controller. Then it steps the
 * physics 20 x 1 ms. Nothing depends on the wall clock, so a test or Mayhem's sim proof runs the
 * robot as fast as the code allows.
 *
 * The three mechanisms are coupled the way a pivotvator is:
 *  - the elevator's gravity is g * sin(pivot angle) along its travel,
 *  - the pivot's gravity torque and inertia grow as the elevator extends,
 *  - the wrist's gravity depends on (pivot + wrist).
 *
 * Exists only in simulation: created by the first Get(), destroyed by RobotContainer's destructor
 * so each test robot starts fresh.
 */
class PhysicsWorld {
 public:
  static PhysicsWorld& Get();
  static bool Exists();
  static void Destroy();

  /** Simulate a motor of {@code mechanism}, configured like the real one. */
  void AttachMotor(Mechanism mechanism, ctre::phoenix6::hardware::TalonFX& device,
                   const ctre::phoenix6::configs::TalonFXConfiguration& config,
                   const mayhem::sim::DcMotor& trapezoidal, const mayhem::sim::DcMotor& foc);
  /** The emulator standing in for {@code device} (the sensor values an IO's UpdateInputs reads). */
  const mayhem::sim::TalonFx& Motor(const ctre::phoenix6::hardware::TalonFX& device) const;
  /** Mirrors TalonFX::SetPosition (the encoder is told where the mechanism is). */
  void SetMechanismPosition(const ctre::phoenix6::hardware::TalonFX& device, double rotations);

  /** Runs one robot loop of physics (20 x 1 ms). Called from Robot::SimulationPeriodic. */
  void Step(double loopSeconds);

  /** True mechanism state (what the robot really did, not what the encoders say). */
  double PivotAngleRad();
  double ElevatorMeters();
  double WristAngleRad();

  /** The robot's true pose. Parked (no drivetrain yet). */
  frc::Pose2d TruePose() const;
  frc::Pose3d TruePose3d() const;

  double BatteryVolts() const { return m_battery.BusVoltage(); }
  const Metrics& GetMetrics() const { return m_metrics; }
  double SimSeconds() const { return m_time; }

 private:
  PhysicsWorld();
  struct MotorEntry;

  void ReadControls();
  void StepMechanisms(double dt, double bus);
  void Publish();
  MotorEntry* Find(const ctre::phoenix6::hardware::TalonFX& device) const;

  std::vector<std::unique_ptr<MotorEntry>> m_motors;
  mayhem::sim::Arm m_pivot;
  mayhem::sim::Elevator m_elevator;
  mayhem::sim::Arm m_wrist;
  mayhem::sim::Battery m_battery;
  Metrics m_metrics;
  bool m_brownedOut = false;
  double m_time = 0;
};

}  // namespace sim
