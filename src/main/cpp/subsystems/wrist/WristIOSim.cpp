// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "subsystems/wrist/WristIOSim.h"

#include <cmath>
#include <numbers>
#include <utility>

#include <frc/RobotController.h>
#include <frc/TimedRobot.h>
#include <units/angle.h>

#include "sim/SimBattery.h"
#include "subsystems/wrist/WristConstants.h"
#include "util/Log.h"

using namespace ctre::phoenix6;
using namespace units::literals;
namespace WC = WristConstants;

WristIOSim::WristIOSim(std::function<double()> pivotAngleRad)
    : WristIOTalonFX(),
      m_leaderSim{m_leader.GetSimState()},
      m_followerSim{m_follower.GetSimState()},
      m_pivotAngleRad{std::move(pivotAngleRad)},
      m_physics{WC::kGearbox,
                WC::kGearRatio,
                units::kilogram_square_meter_t{WC::kMomentOfInertia},
                units::meter_t{WC::kComMeters},
                units::degree_t{WC::kMinAngleDeg},
                units::degree_t{WC::kMaxAngleDeg},
                false,
                units::degree_t{WC::kBootAngleDeg}} {
  m_leaderSim.Orientation = sim::ChassisReference::CounterClockwise_Positive;
  m_followerSim.Orientation = WC::kFollowerAlignment == signals::MotorAlignmentValue::Aligned
                                  ? sim::ChassisReference::CounterClockwise_Positive
                                  : sim::ChassisReference::Clockwise_Positive;
}

void WristIOSim::UpdateInputs(WristIOInputs& inputs) {
  units::volt_t battery = frc::RobotController::GetBatteryVoltage();
  m_leaderSim.SetSupplyVoltage(battery);
  m_followerSim.SetSupplyVoltage(battery);

  double absoluteAngle = m_pivotAngleRad() + m_physics.GetAngle().value();
  double gravityTorque = WC::kMassKg * 9.81 * WC::kComMeters * std::cos(absoluteAngle);
  units::volt_t gravityVolts =
      WC::kGearbox.Voltage(units::newton_meter_t{gravityTorque / WC::kGearRatio}, 0_rad_per_s);

  m_physics.SetInputVoltage(m_leaderSim.GetMotorVoltage() - gravityVolts);
  m_physics.Update(frc::TimedRobot::kDefaultPeriod);

  // Measured from boot, see PivotIOSim for why.
  double bootRad = units::radian_t{units::degree_t{WC::kBootAngleDeg}}.value();
  units::turn_t rotorRot{(m_physics.GetAngle().value() - bootRad) / (2 * std::numbers::pi) *
                         WC::kGearRatio};
  units::turns_per_second_t rotorVel{m_physics.GetVelocity().value() / (2 * std::numbers::pi) *
                                     WC::kGearRatio};
  m_leaderSim.SetRawRotorPosition(rotorRot);
  m_leaderSim.SetRotorVelocity(rotorVel);
  m_followerSim.SetRawRotorPosition(rotorRot);
  m_followerSim.SetRotorVelocity(rotorVel);

  SimBattery::Report("Wrist", m_physics.GetCurrentDraw().value());
  WristIOTalonFX::UpdateInputs(inputs);
  Log::Record("WristSim/AngleDeg", units::degree_t{m_physics.GetAngle()}.value());
}
