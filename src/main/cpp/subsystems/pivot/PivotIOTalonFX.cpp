// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "subsystems/pivot/PivotIOTalonFX.h"

#include <ctre/phoenix6/controls/Follower.hpp>

#include "subsystems/pivot/PivotConstants.h"
#include "util/PhoenixUtil.h"

using namespace ctre::phoenix6;
using namespace units::literals;
namespace PC = PivotConstants;

PivotIOTalonFX::PivotIOTalonFX()
    : m_leader{PC::kLeaderMotorID, CANBus{PC::kCanBus}},
      m_sameSideFollower{PC::kSameSideFollowerID, CANBus{PC::kCanBus}},
      m_otherSideLeader{PC::kOtherSideLeaderID, CANBus{PC::kCanBus}},
      m_otherSideFollower{PC::kOtherSideFollowerID, CANBus{PC::kCanBus}},
      m_motors{&m_leader, &m_sameSideFollower, &m_otherSideLeader, &m_otherSideFollower},
      m_motionMagicRequest{
          controls::MotionMagicVoltage{0_tr}.WithSlot(0).WithEnableFOC(PC::kUseFOC)},
      m_voltageRequest{controls::VoltageOut{0_V}.WithEnableFOC(PC::kUseFOC)},
      m_neutralRequest{},
      m_position{m_leader.GetPosition()},
      m_velocity{m_leader.GetVelocity()},
      m_voltage{m_leader.GetMotorVoltage()},
      m_closedLoopError{m_leader.GetClosedLoopError()} {
  for (hardware::TalonFX* m : m_motors) {
    PhoenixUtil::ApplyMotorConfigs(*m, PC::kMotorConfigs, PC::kFlashConfigRetries);
  }
  m_sameSideFollower.SetControl(controls::Follower{PC::kLeaderMotorID, PC::kSameSideAlignment});
  m_otherSideLeader.SetControl(controls::Follower{PC::kLeaderMotorID, PC::kOtherSideAlignment});
  m_otherSideFollower.SetControl(
      controls::Follower{PC::kLeaderMotorID, PC::kOtherSideAlignment});

  for (int i = 0; i < 4; i++) {
    m_stator[i] = &m_motors[i]->GetStatorCurrent();
    m_supply[i] = &m_motors[i]->GetSupplyCurrent();
    m_temps[i] = &m_motors[i]->GetDeviceTemp();
  }
  m_leaderSignals = {&m_position,       &m_velocity, &m_voltage,  &m_closedLoopError,
                     m_stator[0],       m_supply[0], m_temps[0]};
  m_followerSignals = {m_stator[1], m_supply[1], m_temps[1], m_stator[2], m_supply[2],
                       m_temps[2],  m_stator[3], m_supply[3], m_temps[3]};

  BaseStatusSignal::SetUpdateFrequencyForAll(PC::kUpdateFrequency, m_leaderSignals);
  BaseStatusSignal::SetUpdateFrequencyForAll(PC::kUpdateFrequency, m_followerSignals);
  for (hardware::TalonFX* m : m_motors) {
    m->OptimizeBusUtilization();
  }

  ResetPosition(PC::kBootAngleDeg);
}

void PivotIOTalonFX::UpdateInputs(PivotIOInputs& inputs) {
  inputs.leaderConnected = BaseStatusSignal::RefreshAll(m_leaderSignals).IsOK();
  inputs.followersConnected = BaseStatusSignal::RefreshAll(m_followerSignals).IsOK();
  inputs.positionDeg = m_position.GetValue().value() * 360.0;
  inputs.velocityDegPerSec = m_velocity.GetValue().value() * 360.0;
  inputs.leaderVoltage = m_voltage.GetValue().value();
  inputs.closedLoopErrorDeg = m_closedLoopError.GetValue() * 360.0;
  for (int i = 0; i < 4; i++) {
    inputs.statorCurrents[i] = m_stator[i]->GetValue().value();
    inputs.supplyCurrents[i] = m_supply[i]->GetValue().value();
    inputs.tempsCelsius[i] = m_temps[i]->GetValue().value();
  }
}

void PivotIOTalonFX::SetPosition(double degrees, double gravityFeedforwardVolts) {
  m_leader.SetControl(m_motionMagicRequest.WithPosition(units::turn_t{degrees / 360.0})
                          .WithFeedForward(units::volt_t{gravityFeedforwardVolts}));
}

void PivotIOTalonFX::SetVoltage(double volts) {
  m_leader.SetControl(m_voltageRequest.WithOutput(units::volt_t{volts}));
}

void PivotIOTalonFX::Off() {
  m_leader.SetControl(m_neutralRequest);
}

void PivotIOTalonFX::ResetPosition(double degrees) {
  for (hardware::TalonFX* m : m_motors) {
    m->SetPosition(units::turn_t{degrees / 360.0});
  }
}
