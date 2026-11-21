// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "subsystems/pivot/Pivot.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include <frc/MathUtil.h>
#include <units/angle.h>

#include "util/Log.h"

namespace PC = PivotConstants;

Pivot::Pivot(bool enabled, std::unique_ptr<PivotIO> io)
    : DisableSubsystem(enabled, "Pivot"), m_io{std::move(io)} {}

void Pivot::Periodic() {
  DisableSubsystem::Periodic();
  m_io->UpdateInputs(m_inputs);
  LogInputs();
  Log::Record("Pivot/ReqAngleDeg", m_reqAngleDeg);
  Log::Record("Pivot/ReachedPosition", ReachedPosition());
  Log::Record("Pivot/GravityFFVolts", GravityFeedforwardVolts());
}

void Pivot::LogInputs() const {
  const PivotIOInputs& in = m_inputs;
  Log::Record("Pivot/Inputs/LeaderConnected", in.leaderConnected);
  Log::Record("Pivot/Inputs/FollowersConnected", in.followersConnected);
  Log::Record("Pivot/Inputs/PositionDeg", in.positionDeg);
  Log::Record("Pivot/Inputs/VelocityDegPerSec", in.velocityDegPerSec);
  Log::Record("Pivot/Inputs/LeaderVoltage", in.leaderVoltage);
  Log::Record("Pivot/Inputs/StatorCurrents", in.statorCurrents);
  Log::Record("Pivot/Inputs/SupplyCurrents", in.supplyCurrents);
  Log::Record("Pivot/Inputs/TempsCelsius", in.tempsCelsius);
  Log::Record("Pivot/Inputs/ClosedLoopErrorDeg", in.closedLoopErrorDeg);
}

frc2::CommandPtr Pivot::SetAngle(double degrees) {
  return Run([this, degrees] { ApplyAngle(degrees); }).WithName("Pivot.setAngle");
}

frc2::CommandPtr Pivot::SetAngle(std::function<double()> degrees) {
  return Run([this, degrees = std::move(degrees)] { ApplyAngle(degrees()); })
      .WithName("Pivot.setAngle");
}

frc2::CommandPtr Pivot::SetVoltage(double volts) {
  return Run([this, volts] { m_io->SetVoltage(volts); }).WithName("Pivot.setVoltage");
}

frc2::CommandPtr Pivot::Off() {
  return RunOnce([this] { m_io->Off(); }).WithName("Pivot.off");
}

frc2::CommandPtr Pivot::RezeroAtBootAngle() {
  return RunOnce([this] {
           m_io->ResetPosition(PC::kBootAngleDeg);
           m_reqAngleDeg = PC::kBootAngleDeg;
         })
      .IgnoringDisable(true)
      .WithName("Pivot.rezeroAtBootAngle");
}

double Pivot::GetAngleDeg() const {
  return m_inputs.positionDeg;
}

double Pivot::GetAngleRad() const {
  return units::radian_t{units::degree_t{m_inputs.positionDeg}}.value();
}

double Pivot::GetReqAngleDeg() const {
  return m_reqAngleDeg;
}

bool Pivot::ReachedPosition() const {
  return IsNear(m_reqAngleDeg);
}

bool Pivot::IsNear(double degrees) const {
  return frc::IsNear(degrees, m_inputs.positionDeg, PC::kToleranceDeg);
}

void Pivot::SetElevatorExtensionSupplier(std::function<double()> elevatorMeters) {
  m_elevatorMeters = std::move(elevatorMeters);
}

double Pivot::GravityFeedforwardVolts() const {
  return PC::KGVolts(m_elevatorMeters()) * std::cos(GetAngleRad());
}

void Pivot::ApplyAngle(double degrees) {
  m_reqAngleDeg = std::clamp(degrees, PC::kMinAngleDeg, PC::kMaxAngleDeg);
  m_io->SetPosition(m_reqAngleDeg, GravityFeedforwardVolts());
}
