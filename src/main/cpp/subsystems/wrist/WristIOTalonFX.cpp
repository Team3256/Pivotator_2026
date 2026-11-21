// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "subsystems/wrist/WristIOTalonFX.h"

#include <ctre/phoenix6/controls/Follower.hpp>

#include "subsystems/wrist/WristConstants.h"
#include "util/PhoenixUtil.h"

using namespace ctre::phoenix6;
using namespace units::literals;
namespace WC = WristConstants;

WristIOTalonFX::WristIOTalonFX()
    : m_leader{WC::kLeaderMotorID, CANBus{WC::kCanBus}},
      m_follower{WC::kFollowerMotorID, CANBus{WC::kCanBus}},
      m_motionMagicRequest{
          controls::MotionMagicVoltage{0_tr}.WithSlot(0).WithEnableFOC(WC::kUseFOC)},
      m_voltageRequest{controls::VoltageOut{0_V}.WithEnableFOC(WC::kUseFOC)},
      m_neutralRequest{},
      m_position{m_leader.GetPosition()},
      m_velocity{m_leader.GetVelocity()},
      m_voltage{m_leader.GetMotorVoltage()},
      m_leaderStator{m_leader.GetStatorCurrent()},
      m_leaderTemp{m_leader.GetDeviceTemp()},
      m_followerStator{m_follower.GetStatorCurrent()},
      m_followerTemp{m_follower.GetDeviceTemp()} {
  PhoenixUtil::ApplyMotorConfigs(m_leader, WC::kMotorConfigs, WC::kFlashConfigRetries);
  PhoenixUtil::ApplyMotorConfigs(m_follower, WC::kMotorConfigs, WC::kFlashConfigRetries);
  m_follower.SetControl(controls::Follower{WC::kLeaderMotorID, WC::kFollowerAlignment});

  BaseStatusSignal::SetUpdateFrequencyForAll(WC::kUpdateFrequency, m_position, m_velocity,
                                             m_voltage, m_leaderStator, m_leaderTemp);
  BaseStatusSignal::SetUpdateFrequencyForAll(WC::kUpdateFrequency, m_followerStator,
                                             m_followerTemp);
  m_leader.OptimizeBusUtilization();
  m_follower.OptimizeBusUtilization();

  ResetPosition(WC::kBootAngleDeg);
}

void WristIOTalonFX::UpdateInputs(WristIOInputs& inputs) {
  inputs.leaderConnected = BaseStatusSignal::RefreshAll(m_position, m_velocity, m_voltage,
                                                        m_leaderStator, m_leaderTemp)
                               .IsOK();
  inputs.followerConnected =
      BaseStatusSignal::RefreshAll(m_followerStator, m_followerTemp).IsOK();
  inputs.positionDeg = m_position.GetValue().value() * 360.0;
  inputs.velocityDegPerSec = m_velocity.GetValue().value() * 360.0;
  inputs.leaderVoltage = m_voltage.GetValue().value();
  inputs.leaderStatorCurrent = m_leaderStator.GetValue().value();
  inputs.leaderTempCelsius = m_leaderTemp.GetValue().value();
  inputs.followerStatorCurrent = m_followerStator.GetValue().value();
  inputs.followerTempCelsius = m_followerTemp.GetValue().value();
}

void WristIOTalonFX::SetPosition(double degrees, double gravityFeedforwardVolts) {
  m_leader.SetControl(m_motionMagicRequest.WithPosition(units::turn_t{degrees / 360.0})
                          .WithFeedForward(units::volt_t{gravityFeedforwardVolts}));
}

void WristIOTalonFX::SetVoltage(double volts) {
  m_leader.SetControl(m_voltageRequest.WithOutput(units::volt_t{volts}));
}

void WristIOTalonFX::Off() {
  m_leader.SetControl(m_neutralRequest);
}

void WristIOTalonFX::ResetPosition(double degrees) {
  m_leader.SetPosition(units::turn_t{degrees / 360.0});
  m_follower.SetPosition(units::turn_t{degrees / 360.0});
}
