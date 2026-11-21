// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "subsystems/wrist/Wrist.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include <frc/MathUtil.h>
#include <units/angle.h>

#include "util/Log.h"

namespace WC = WristConstants;

Wrist::Wrist(bool enabled, std::unique_ptr<WristIO> io)
    : DisableSubsystem(enabled, "Wrist"), m_io{std::move(io)} {}

void Wrist::Periodic() {
  DisableSubsystem::Periodic();
  m_io->UpdateInputs(m_inputs);
  LogInputs();
  Log::Record("Wrist/ReqAngleDeg", m_reqAngleDeg);
  Log::Record("Wrist/ReachedPosition", ReachedPosition());
  Log::Record("Wrist/AngleFromHorizontalDeg",
              units::degree_t{units::radian_t{m_pivotAngleRad()}}.value() + m_inputs.positionDeg);
}

void Wrist::LogInputs() const {
  const WristIOInputs& in = m_inputs;
  Log::Record("Wrist/Inputs/LeaderConnected", in.leaderConnected);
  Log::Record("Wrist/Inputs/FollowerConnected", in.followerConnected);
  Log::Record("Wrist/Inputs/PositionDeg", in.positionDeg);
  Log::Record("Wrist/Inputs/VelocityDegPerSec", in.velocityDegPerSec);
  Log::Record("Wrist/Inputs/LeaderVoltage", in.leaderVoltage);
  Log::Record("Wrist/Inputs/LeaderStatorCurrent", in.leaderStatorCurrent);
  Log::Record("Wrist/Inputs/FollowerStatorCurrent", in.followerStatorCurrent);
  Log::Record("Wrist/Inputs/LeaderTempCelsius", in.leaderTempCelsius);
  Log::Record("Wrist/Inputs/FollowerTempCelsius", in.followerTempCelsius);
}

frc2::CommandPtr Wrist::SetAngle(double degrees) {
  return Run([this, degrees] { ApplyAngle(degrees); }).WithName("Wrist.setAngle");
}

frc2::CommandPtr Wrist::SetAngle(std::function<double()> degrees) {
  return Run([this, degrees = std::move(degrees)] { ApplyAngle(degrees()); })
      .WithName("Wrist.setAngle");
}

frc2::CommandPtr Wrist::SetVoltage(double volts) {
  return Run([this, volts] { m_io->SetVoltage(volts); }).WithName("Wrist.setVoltage");
}

frc2::CommandPtr Wrist::Off() {
  return RunOnce([this] { m_io->Off(); }).WithName("Wrist.off");
}

frc2::CommandPtr Wrist::RezeroAtBootAngle() {
  return RunOnce([this] {
           m_io->ResetPosition(WC::kBootAngleDeg);
           m_reqAngleDeg = WC::kBootAngleDeg;
         })
      .IgnoringDisable(true)
      .WithName("Wrist.rezeroAtBootAngle");
}

double Wrist::GetAngleDeg() const {
  return m_inputs.positionDeg;
}

double Wrist::GetAngleRad() const {
  return units::radian_t{units::degree_t{m_inputs.positionDeg}}.value();
}

bool Wrist::ReachedPosition() const {
  return IsNear(m_reqAngleDeg);
}

bool Wrist::IsNear(double degrees) const {
  return frc::IsNear(degrees, m_inputs.positionDeg, WC::kToleranceDeg);
}

void Wrist::SetPivotAngleSupplier(std::function<double()> pivotAngleRad) {
  m_pivotAngleRad = std::move(pivotAngleRad);
}

double Wrist::GravityFeedforwardVolts() const {
  return WC::kGVolts * std::cos(m_pivotAngleRad() + GetAngleRad());
}

void Wrist::ApplyAngle(double degrees) {
  m_reqAngleDeg = std::clamp(degrees, WC::kMinAngleDeg, WC::kMaxAngleDeg);
  m_io->SetPosition(m_reqAngleDeg, GravityFeedforwardVolts());
}
