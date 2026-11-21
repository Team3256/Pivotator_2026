// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <array>
#include <functional>
#include <memory>
#include <utility>

#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <units/time.h>

#include "subsystems/swerve/generated/TunerConstants.h"

namespace sim {
class DrivetrainSim;
}

/**
 * The Phoenix 6 swerve drivetrain (ported from FRC_Offseason_2026). Nothing owns it through the
 * command scheduler: each loop RobotContainer picks one control (driver or autonomous) and calls
 * exactly one drive method.
 *
 * In simulation Phoenix's 250 Hz odometry thread is stopped (it runs on the wall clock), and the
 * drivetrain runs on Mayhem's swerve physics through sim::DrivetrainSim on stepped time instead.
 *
 * State() caches one GetState() copy per loop (GetState copies the whole state under a lock).
 */
class Drivetrain : public TunerConstants::TunerSwerveDrivetrain {
 public:
  using SwerveDriveState = TunerConstants::TunerSwerveDrivetrain::SwerveDriveState;

  template <typename... Arg>
  explicit Drivetrain(Arg&&... args)
      : TunerConstants::TunerSwerveDrivetrain{std::forward<Arg>(args)...} {
    Init();
  }

  ~Drivetrain() override;

  /** Cached drivetrain state, at most one GetState() copy per loop. */
  const SwerveDriveState& State();
  const frc::Pose2d& Pose() { return State().Pose; }
  /** Called at the start of every robot loop. */
  void InvalidateStateCache() { m_stateValid = false; }

  frc::ChassisSpeeds GetFieldRelativeSpeeds();
  /** Blue-origin field speeds from Mayhem's AutoRunner. */
  void ApplyMayhemSpeeds(frc::ChassisSpeeds speeds);
  /** Holds still (wheels stopped). */
  void Stop();

  /** Call once per loop before any drive method: operator perspective and logging. */
  void Periodic();

  /**
   * Phoenix's SetControl. In simulation the request goes to sim::DrivetrainSim, which applies it
   * every 4 ms of simulated time.
   */
  template <typename Request>
  void SetControl(Request&& request) {
    if (m_sim) {
      SimSetControl(request);
    } else {
      TunerSwerveDrivetrain::SetControl(std::forward<Request>(request));
    }
  }

  void ResetPose(frc::Pose2d const& pose) override;
  void SeedFieldCentric(frc::Rotation2d const& rotation = frc::Rotation2d{}) override;

  /**
   * Stops the odometry and simulation, and parks the drivetrain on a request it owns. Phoenix
   * keeps a reference to the last request and re-applies it at 250 Hz, so this must run before a
   * request owned elsewhere (RobotContainer) is destroyed. Called by ~RobotContainer.
   */
  void Shutdown();

 private:
  void Init();
  void SimSetControl(const ctre::phoenix6::swerve::requests::SwerveRequest& request);

  std::unique_ptr<sim::DrivetrainSim> m_sim;
  SwerveDriveState m_state{};
  bool m_stateValid = false;
  bool m_hasAppliedOperatorPerspective = false;
  bool m_shutDown = false;

  ctre::phoenix6::swerve::requests::ApplyFieldSpeeds m_mayhemRequest;
  ctre::phoenix6::swerve::requests::SwerveDriveBrake m_stopRequest;
  ctre::phoenix6::swerve::requests::Idle m_idleRequest;
};
