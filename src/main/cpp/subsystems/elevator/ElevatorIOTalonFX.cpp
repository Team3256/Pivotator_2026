// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "subsystems/elevator/ElevatorIOTalonFX.h"

#include <ctre/phoenix6/controls/Follower.hpp>

#include "subsystems/elevator/ElevatorConstants.h"
#include "util/PhoenixUtil.h"

using namespace ctre::phoenix6;
using namespace units::literals;
namespace EC = ElevatorConstants;

ElevatorIOTalonFX::ElevatorIOTalonFX()
    : m_leader{EC::kLeaderMotorID, CANBus{EC::kCanBus}},
      m_follower{EC::kFollowerMotorID, CANBus{EC::kCanBus}},
      m_motionMagicRequest{controls::MotionMagicVoltage{0_tr}.WithSlot(0).WithEnableFOC(
          EC::kUseFOC)},
      m_positionRequest{
          controls::PositionVoltage{0_tr}.WithSlot(0).WithEnableFOC(EC::kUseFOC)},
      m_voltageRequest{controls::VoltageOut{0_V}.WithEnableFOC(EC::kUseFOC)},
      m_neutralRequest{},
      m_leaderPosition{m_leader.GetPosition()},
      m_leaderVelocity{m_leader.GetVelocity()},
      m_leaderVoltage{m_leader.GetMotorVoltage()},
      m_leaderStatorCurrent{m_leader.GetStatorCurrent()},
      m_leaderSupplyCurrent{m_leader.GetSupplyCurrent()},
      m_leaderTemp{m_leader.GetDeviceTemp()},
      m_closedLoopReference{m_leader.GetClosedLoopReference()},
      m_closedLoopError{m_leader.GetClosedLoopError()},
      m_followerPosition{m_follower.GetPosition()},
      m_followerVelocity{m_follower.GetVelocity()},
      m_followerVoltage{m_follower.GetMotorVoltage()},
      m_followerStatorCurrent{m_follower.GetStatorCurrent()},
      m_followerSupplyCurrent{m_follower.GetSupplyCurrent()},
      m_followerTemp{m_follower.GetDeviceTemp()} {
  PhoenixUtil::ApplyMotorConfigs(m_leader, EC::kMotorConfigs, EC::kFlashConfigRetries);
  PhoenixUtil::ApplyMotorConfigs(m_follower, EC::kMotorConfigs, EC::kFlashConfigRetries);

  m_follower.SetControl(controls::Follower{EC::kLeaderMotorID, EC::kFollowerAlignment});

  BaseStatusSignal::SetUpdateFrequencyForAll(
      EC::kUpdateFrequency, m_leaderPosition, m_leaderVelocity, m_leaderVoltage,
      m_leaderStatorCurrent, m_leaderSupplyCurrent, m_leaderTemp, m_closedLoopReference,
      m_closedLoopError);
  BaseStatusSignal::SetUpdateFrequencyForAll(
      EC::kUpdateFrequency, m_followerPosition, m_followerVelocity, m_followerVoltage,
      m_followerStatorCurrent, m_followerSupplyCurrent, m_followerTemp);
  m_leader.OptimizeBusUtilization();
  m_follower.OptimizeBusUtilization();

  // Elevator is assumed to boot fully down. Run Elevator::Home() if that's ever not true.
  ResetPosition(EC::kMinHeightMeters);
}

void ElevatorIOTalonFX::UpdateInputs(ElevatorIOInputs& inputs) {
  inputs.leaderConnected =
      BaseStatusSignal::RefreshAll(m_leaderPosition, m_leaderVelocity, m_leaderVoltage,
                                   m_leaderStatorCurrent, m_leaderSupplyCurrent, m_leaderTemp,
                                   m_closedLoopReference, m_closedLoopError)
          .IsOK();
  inputs.leaderPositionMeters = m_leaderPosition.GetValue().value();
  inputs.leaderVelocityMetersPerSec = m_leaderVelocity.GetValue().value();
  inputs.leaderVoltage = m_leaderVoltage.GetValue().value();
  inputs.leaderStatorCurrent = m_leaderStatorCurrent.GetValue().value();
  inputs.leaderSupplyCurrent = m_leaderSupplyCurrent.GetValue().value();
  inputs.leaderTempCelsius = m_leaderTemp.GetValue().value();
  inputs.closedLoopReferenceMeters = m_closedLoopReference.GetValue();
  inputs.closedLoopErrorMeters = m_closedLoopError.GetValue();

  inputs.followerConnected =
      BaseStatusSignal::RefreshAll(m_followerPosition, m_followerVelocity, m_followerVoltage,
                                   m_followerStatorCurrent, m_followerSupplyCurrent,
                                   m_followerTemp)
          .IsOK();
  inputs.followerPositionMeters = m_followerPosition.GetValue().value();
  inputs.followerVelocityMetersPerSec = m_followerVelocity.GetValue().value();
  inputs.followerVoltage = m_followerVoltage.GetValue().value();
  inputs.followerStatorCurrent = m_followerStatorCurrent.GetValue().value();
  inputs.followerSupplyCurrent = m_followerSupplyCurrent.GetValue().value();
  inputs.followerTempCelsius = m_followerTemp.GetValue().value();
}

void ElevatorIOTalonFX::SetPosition(double meters, double gravityFeedforwardVolts) {
  if constexpr (kUseMotionMagic) {
    m_leader.SetControl(m_motionMagicRequest.WithPosition(units::turn_t{meters})
                            .WithFeedForward(units::volt_t{gravityFeedforwardVolts}));
  } else {
    m_leader.SetControl(m_positionRequest.WithPosition(units::turn_t{meters})
                            .WithFeedForward(units::volt_t{gravityFeedforwardVolts}));
  }
}

void ElevatorIOTalonFX::SetVoltage(double volts) {
  m_leader.SetControl(
      m_voltageRequest.WithOutput(units::volt_t{volts}).WithIgnoreSoftwareLimits(false));
}

void ElevatorIOTalonFX::SetVoltageIgnoringLimits(double volts) {
  m_leader.SetControl(
      m_voltageRequest.WithOutput(units::volt_t{volts}).WithIgnoreSoftwareLimits(true));
}

void ElevatorIOTalonFX::Off() {
  m_leader.SetControl(m_neutralRequest);
}

void ElevatorIOTalonFX::ResetPosition(double meters) {
  m_leader.SetPosition(units::turn_t{meters});
  m_follower.SetPosition(units::turn_t{meters});
}
