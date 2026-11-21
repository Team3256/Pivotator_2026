// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <memory>
#include <set>
#include <string>
#include <vector>

#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>

#include "mayhem/sim/battery.hpp"
#include "mayhem/sim/field.hpp"
#include "mayhem/sim/mechanisms.hpp"
#include "mayhem/sim/swerve.hpp"
#include "mayhem/sim/talonfx.hpp"

namespace sim {

/** The simulated mechanisms, one physics plant each. */
enum class Mechanism { kPivot, kElevator, kWrist };

/**
 * The drivetrain's side of the simulation. Phoenix applies the drivetrain's request and samples
 * odometry on its 250 Hz thread; the world calls this every 4 ms of simulated time instead.
 */
class DriveHooks {
 public:
  virtual ~DriveHooks() = default;
  /** Apply the current swerve request to the simulated modules and update odometry. */
  virtual void ControlTick(double fpgaSeconds) = 0;
};

/** What happened during a run, for tests and Mayhem's sim proof (its probe reads these fields). */
struct Metrics {
  int shotsFired = 0, intaked = 0, scoredOwnHub = 0;
  int wallHits = 0, obstacleHits = 0;
  /** Each hit: "<obstacle> at t=<s> robot (x, y, heading) speed <m/s>", in order. */
  std::vector<std::string> hits;
  double beachedSeconds = 0, maxTiltDeg = 0, minBatteryVolts = 99, maxTotalCurrent = 0;
  bool tippedOver = false, brownedOut = false;
};

/**
 * The simulated robot: Mayhem's physics engine (mayhem::sim, vendored in src/main/include/mayhem)
 * running the swerve drivetrain on the 2025 REEFSCAPE field, the pivot, elevator, wrist and the
 * battery, on simulated time.
 *
 * The robot code keeps commanding its real Phoenix devices. Each loop the world reads every
 * mechanism motor's applied control (TalonFX::GetAppliedControl) into Mayhem's TalonFX emulator,
 * configured from the same TalonFXConfiguration, and the drivetrain's swerve request goes through
 * Mayhem's Phoenix swerve emulator (sim::DrivetrainSim). Then it steps 20 x 1 ms. Nothing depends
 * on the wall clock, so a test or Mayhem's sim proof runs the robot as fast as the code allows.
 *
 * The mechanisms are coupled the way a pivotvator is (elevator gravity follows the pivot, the
 * pivot's load grows with extension, the wrist's gravity is pivot + wrist), and the robot's own
 * pitch on the field tilts all of them.
 *
 * Exists only in simulation: created by the first Get(), destroyed by RobotContainer's destructor
 * so each test robot starts fresh.
 */
class PhysicsWorld {
 public:
  static PhysicsWorld& Get();
  static bool Exists();
  static void Destroy();
  /**
   * Whether the next world has the REEFSCAPE field (walls, reefs, coral stations) or is an empty
   * carpet. Default: the field.
   */
  static void SetNextFieldEnabled(bool enabled);
  bool FieldEnabled() const { return m_fieldEnabled; }

  /** Simulate a motor of {@code mechanism}, configured like the real one. */
  void AttachMotor(Mechanism mechanism, ctre::phoenix6::hardware::TalonFX& device,
                   const ctre::phoenix6::configs::TalonFXConfiguration& config,
                   const mayhem::sim::DcMotor& trapezoidal, const mayhem::sim::DcMotor& foc);
  /** The emulator standing in for {@code device} (the sensor values an IO's UpdateInputs reads). */
  const mayhem::sim::TalonFx& Motor(const ctre::phoenix6::hardware::TalonFX& device) const;
  /** Mirrors TalonFX::SetPosition (the encoder is told where the mechanism is). */
  void SetMechanismPosition(const ctre::phoenix6::hardware::TalonFX& device, double rotations);

  mayhem::sim::SwerveSim& Swerve() { return *m_swerve; }
  void SetDriveHooks(DriveHooks* hooks) { m_driveHooks = hooks; }

  /** Runs one robot loop of physics (20 x 1 ms). Called from Robot::SimulationPeriodic. */
  void Step(double loopSeconds);

  /** True mechanism state (what the robot really did, not what the encoders say). */
  double PivotAngleRad();
  double ElevatorMeters();
  double WristAngleRad();

  /** The robot's true pose on the field (the estimate is the drivetrain's). */
  frc::Pose2d TruePose() const;
  frc::Pose3d TruePose3d() const;
  /** Places the robot (zero velocity). */
  void SetTruePose(const frc::Pose2d& pose);
  /**
   * Drivetrain::ResetPose calls this: the robot is moved to the reset pose, as if it had been
   * placed there, unless KeepPoseOnReset is set.
   */
  void OnOdometryReset(const frc::Pose2d& pose);
  void KeepPoseOnReset(bool keep) { m_keepPoseOnReset = keep; }

  double BatteryVolts() const { return m_battery.BusVoltage(); }
  const Metrics& GetMetrics() const { return m_metrics; }
  double SimSeconds() const { return m_time; }

 private:
  PhysicsWorld();
  struct MotorEntry;

  void ReadControls();
  void StepMechanisms(double dt, double bus, double robotPitch);
  void UpdateMetrics(double dt);
  void Publish();
  MotorEntry* Find(const ctre::phoenix6::hardware::TalonFX& device) const;
  /** Height of the top of the robot above the floor, from the pivot and elevator. */
  double RobotHeight();

  std::vector<std::unique_ptr<MotorEntry>> m_motors;
  mayhem::sim::Arm m_pivot;
  mayhem::sim::Elevator m_elevator;
  mayhem::sim::Arm m_wrist;
  std::unique_ptr<mayhem::sim::SwerveSim> m_swerve;
  mayhem::sim::Terrain m_terrain;
  mayhem::sim::Obstacles m_obstacles;
  mayhem::sim::Battery m_battery;
  DriveHooks* m_driveHooks = nullptr;
  Metrics m_metrics;
  std::set<std::string> m_touching;
  bool m_fieldEnabled = true;
  bool m_keepPoseOnReset = false;
  bool m_brownedOut = false;
  double m_time = 0;
};

}  // namespace sim
