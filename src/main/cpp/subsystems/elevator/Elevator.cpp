// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "subsystems/elevator/Elevator.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>
#include <utility>

#include <frc/MathUtil.h>
#include <frc/filter/Debouncer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/util/Color.h>
#include <frc/util/Color8Bit.h>
#include <units/angle.h>

#include "util/Log.h"

namespace EC = ElevatorConstants;
using EC::Height;

Elevator::Elevator(bool enabled, std::unique_ptr<ElevatorIO> io)
    : DisableSubsystem(enabled, "Elevator"),
      m_io{std::move(io)},
      m_pivotAngleRad{[] { return std::numbers::pi / 2.0; }} {
  frc::MechanismRoot2d* root = m_mech.GetRoot("ElevatorBase", 1.0, 0.2);
  m_carriage = root->Append<frc::MechanismLigament2d>(
      "Carriage", EC::kMinHeightMeters, units::degree_t{90}, 8,
      frc::Color8Bit{frc::Color::kOrange});
  frc::SmartDashboard::PutData("Elevator/Mechanism2d", &m_mech);
}

void Elevator::Periodic() {
  DisableSubsystem::Periodic();
  m_io->UpdateInputs(m_inputs);
  LogInputs();

  Log::Record("Elevator/ReqPositionMeters", m_reqPositionMeters);
  Log::Record("Elevator/ReachedPosition", ReachedPosition());
  Log::Record("Elevator/Homed", m_homed);
  Log::Record("Elevator/SafeForPivot", IsSafeForPivot());

  m_carriage->SetLength(std::max(m_inputs.leaderPositionMeters, 0.01));
  m_carriage->SetAngle(units::radian_t{m_pivotAngleRad()});
}

void Elevator::LogInputs() const {
  const ElevatorIOInputs& in = m_inputs;
  Log::Record("Elevator/Inputs/LeaderConnected", in.leaderConnected);
  Log::Record("Elevator/Inputs/LeaderPositionMeters", in.leaderPositionMeters);
  Log::Record("Elevator/Inputs/LeaderVelocityMetersPerSec", in.leaderVelocityMetersPerSec);
  Log::Record("Elevator/Inputs/LeaderVoltage", in.leaderVoltage);
  Log::Record("Elevator/Inputs/LeaderStatorCurrent", in.leaderStatorCurrent);
  Log::Record("Elevator/Inputs/LeaderSupplyCurrent", in.leaderSupplyCurrent);
  Log::Record("Elevator/Inputs/LeaderTempCelsius", in.leaderTempCelsius);
  Log::Record("Elevator/Inputs/FollowerConnected", in.followerConnected);
  Log::Record("Elevator/Inputs/FollowerPositionMeters", in.followerPositionMeters);
  Log::Record("Elevator/Inputs/FollowerVelocityMetersPerSec", in.followerVelocityMetersPerSec);
  Log::Record("Elevator/Inputs/FollowerVoltage", in.followerVoltage);
  Log::Record("Elevator/Inputs/FollowerStatorCurrent", in.followerStatorCurrent);
  Log::Record("Elevator/Inputs/FollowerSupplyCurrent", in.followerSupplyCurrent);
  Log::Record("Elevator/Inputs/FollowerTempCelsius", in.followerTempCelsius);
  Log::Record("Elevator/Inputs/ClosedLoopReferenceMeters", in.closedLoopReferenceMeters);
  Log::Record("Elevator/Inputs/ClosedLoopErrorMeters", in.closedLoopErrorMeters);
}

// ---------------- Commands ----------------

frc2::CommandPtr Elevator::SetPosition(double meters) {
  return Run([this, meters] { ApplyPosition(meters); }).WithName("Elevator.setPosition");
}

frc2::CommandPtr Elevator::SetPosition(std::function<double()> meters) {
  return Run([this, meters = std::move(meters)] { ApplyPosition(meters()); })
      .WithName("Elevator.setPosition");
}

frc2::CommandPtr Elevator::SetPosition(Height height) {
  return Run([this, height] { ApplyPosition(EC::Meters(height)); })
      .WithName("Elevator." + std::string{EC::Name(height)});
}

frc2::CommandPtr Elevator::GoToAndWait(Height height) {
  return SetPosition(height)
      .Until([this] { return ReachedPosition(); })
      .WithName("Elevator.waitFor" + std::string{EC::Name(height)});
}

frc2::CommandPtr Elevator::GoToStow() {
  return SetPosition(Height::kStow);
}

frc2::CommandPtr Elevator::GoToCoralIntake() {
  return SetPosition(Height::kCoralIntake);
}

frc2::CommandPtr Elevator::GoToL1() {
  return SetPosition(Height::kL1);
}

frc2::CommandPtr Elevator::GoToL2() {
  return SetPosition(Height::kL2);
}

frc2::CommandPtr Elevator::GoToL3() {
  return SetPosition(Height::kL3);
}

frc2::CommandPtr Elevator::GoToL4() {
  return SetPosition(Height::kL4);
}

frc2::CommandPtr Elevator::SetVoltage(double volts) {
  return Run([this, volts] { m_io->SetVoltage(volts); }).WithName("Elevator.setVoltage");
}

frc2::CommandPtr Elevator::Manual(std::function<double()> axis, double maxVolts) {
  return Run([this, axis = std::move(axis), maxVolts] {
           m_io->SetVoltage(frc::ApplyDeadband(axis(), 0.1) * maxVolts);
         })
      .WithName("Elevator.manual");
}

frc2::CommandPtr Elevator::Off() {
  return RunOnce([this] { m_io->Off(); }).WithName("Elevator.off");
}

frc2::CommandPtr Elevator::Home() {
  // shared_ptr so the debouncer lives as long as the command does.
  auto stallDebouncer = std::make_shared<frc::Debouncer>(EC::kHomingDebounce,
                                                         frc::Debouncer::DebounceType::kRising);
  return RunOnce([this, stallDebouncer] {
           m_homed = false;
           stallDebouncer->Calculate(false);
         })
      .AndThen(Run([this] { m_io->SetVoltageIgnoringLimits(EC::kHomingVoltage); })
                   .Until([this, stallDebouncer] {
                     return stallDebouncer->Calculate(std::abs(m_inputs.leaderStatorCurrent) >
                                                      EC::kHomingCurrentThresholdAmps);
                   }))
      .WithTimeout(EC::kHomingTimeout)
      .FinallyDo([this](bool interrupted) {
        m_io->Off();
        if (!interrupted) {
          m_io->ResetPosition(EC::kMinHeightMeters);
          m_reqPositionMeters = EC::kMinHeightMeters;
          m_homed = true;
        }
      })
      .WithName("Elevator.home");
}

// ---------------- State ----------------

double Elevator::GetPositionMeters() const {
  return m_inputs.leaderPositionMeters;
}

double Elevator::GetReqPositionMeters() const {
  return m_reqPositionMeters;
}

bool Elevator::ReachedPosition() const {
  return frc::IsNear(m_reqPositionMeters, m_inputs.leaderPositionMeters,
                     EC::kPositionToleranceMeters);
}

bool Elevator::IsAt(Height height) const {
  return frc::IsNear(EC::Meters(height), m_inputs.leaderPositionMeters,
                     EC::kPositionToleranceMeters);
}

frc2::Trigger Elevator::IsAtTrigger(Height height) {
  return frc2::Trigger{[this, height] { return IsAt(height); }};
}

bool Elevator::IsSafeForPivot() const {
  return m_inputs.leaderPositionMeters < EC::kPivotSafeMaxHeightMeters;
}

void Elevator::SetPivotAngleSupplier(std::function<double()> pivotAngleRad) {
  m_pivotAngleRad = std::move(pivotAngleRad);
}

double Elevator::GravityFeedforwardVolts() const {
  return EC::kGVerticalVolts * std::sin(m_pivotAngleRad());
}

bool Elevator::IsHomed() const {
  return m_homed;
}

void Elevator::ApplyPosition(double meters) {
  m_reqPositionMeters = std::clamp(meters, EC::kMinHeightMeters, EC::kMaxHeightMeters);
  m_io->SetPosition(m_reqPositionMeters, GravityFeedforwardVolts());
}
