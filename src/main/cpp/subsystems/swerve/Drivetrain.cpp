// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "subsystems/swerve/Drivetrain.h"

#include <array>

#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/geometry/Pose3d.h>

#include "sim/DrivetrainSim.h"
#include "util/Log.h"

using namespace ctre::phoenix6;

namespace {
/* Blue alliance sees forward as 0 degrees (toward the red alliance wall), red as 180. */
const frc::Rotation2d kBluePerspective{0_deg};
const frc::Rotation2d kRedPerspective{180_deg};
}  // namespace

void Drivetrain::Init() {
  m_mayhemRequest.WithDriveRequestType(swerve::DriveRequestType::Velocity)
      .WithForwardPerspective(swerve::requests::ForwardPerspectiveValue::BlueAlliance)
      .WithDesaturateWheelSpeeds(true);
  m_mayhemRequest.WheelForceFeedforwardsX.assign(4, 0_N);
  m_mayhemRequest.WheelForceFeedforwardsY.assign(4, 0_N);
  if (frc::RobotBase::IsSimulation()) {
    // Mayhem's physics drives the simulated drivetrain on simulated time; Phoenix's odometry
    // thread and device simulation would run on the wall clock (see sim/DrivetrainSim.h).
    GetOdometryThread().Stop();
    m_sim = std::make_unique<::sim::DrivetrainSim>([this] { return GetOperatorForwardDirection(); });
  }
}

Drivetrain::~Drivetrain() {
  Shutdown();
}

void Drivetrain::Shutdown() {
  if (m_shutDown) {
    return;
  }
  m_shutDown = true;
  GetOdometryThread().Stop();
  SetControl(m_idleRequest);
  m_sim.reset();
}

const Drivetrain::SwerveDriveState& Drivetrain::State() {
  if (!m_stateValid) {
    m_state = m_sim ? m_sim->State() : GetState();
    m_stateValid = true;
  }
  return m_state;
}

void Drivetrain::SimSetControl(const swerve::requests::SwerveRequest& request) {
  m_sim->SetControl(request);
}

frc::ChassisSpeeds Drivetrain::GetFieldRelativeSpeeds() {
  const auto& st = State();
  return frc::ChassisSpeeds::FromRobotRelativeSpeeds(st.Speeds, st.Pose.Rotation());
}

void Drivetrain::ApplyMayhemSpeeds(frc::ChassisSpeeds speeds) {
  m_mayhemRequest.Speeds = speeds;
  SetControl(m_mayhemRequest);
}

void Drivetrain::Stop() {
  SetControl(m_stopRequest);
}

void Drivetrain::ResetPose(frc::Pose2d const& pose) {
  if (m_sim) {
    m_sim->ResetPose(pose);
  } else {
    TunerSwerveDrivetrain::ResetPose(pose);
  }
  InvalidateStateCache();
}

void Drivetrain::SeedFieldCentric(frc::Rotation2d const& rotation) {
  if (m_sim) {
    m_sim->SeedFieldCentric(rotation);
  } else {
    TunerSwerveDrivetrain::SeedFieldCentric(rotation);
  }
  InvalidateStateCache();
}

void Drivetrain::Periodic() {
  /*
   * Apply the operator perspective for the alliance. The first time, regardless of DS state
   * (corrects it if the code restarts mid-match); after that only while disabled, so driving
   * doesn't change direction until an explicit disable.
   */
  if (!m_hasAppliedOperatorPerspective || frc::DriverStation::IsDisabled()) {
    if (auto alliance = frc::DriverStation::GetAlliance()) {
      SetOperatorPerspectiveForward(*alliance == frc::DriverStation::Alliance::kRed
                                        ? kRedPerspective
                                        : kBluePerspective);
      m_hasAppliedOperatorPerspective = true;
    }
  }

  const auto& st = State();
  // The pose estimate (Mayhem draws it as a ghost next to the true pose in its 3D replay).
  Log::Record("Odometry/Robot", st.Pose);
  Log::Record("Drive/Speeds",
              std::array<double, 3>{st.Speeds.vx.value(), st.Speeds.vy.value(),
                                    st.Speeds.omega.value()});
}

Drivetrain TunerConstants::CreateDrivetrain() {
  return Drivetrain{DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight};
}
