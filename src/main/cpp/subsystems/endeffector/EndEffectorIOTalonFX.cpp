// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "subsystems/endeffector/EndEffectorIOTalonFX.h"

#include <ctre/phoenix6/controls/Follower.hpp>

#include "subsystems/endeffector/EndEffectorConstants.h"
#include "util/PhoenixUtil.h"

using namespace ctre::phoenix6;
using namespace units::literals;
namespace EEC = EndEffectorConstants;

EndEffectorIOTalonFX::Motor::Motor(int id, std::string_view bus)
    : device{id, CANBus{bus}},
      velocity{device.GetVelocity()},
      voltage{device.GetMotorVoltage()},
      statorCurrent{device.GetStatorCurrent()},
      supplyCurrent{device.GetSupplyCurrent()},
      temp{device.GetDeviceTemp()} {
  BaseStatusSignal::SetUpdateFrequencyForAll(EEC::kUpdateFrequency, velocity, voltage,
                                             statorCurrent, supplyCurrent, temp);
}

bool EndEffectorIOTalonFX::Motor::Refresh(RollerMotorInputs& out) {
  out.connected =
      BaseStatusSignal::RefreshAll(velocity, voltage, statorCurrent, supplyCurrent, temp).IsOK();
  out.velocityRps = velocity.GetValue().value();
  out.voltage = voltage.GetValue().value();
  out.statorCurrent = statorCurrent.GetValue().value();
  out.supplyCurrent = supplyCurrent.GetValue().value();
  out.tempCelsius = temp.GetValue().value();
  return out.connected;
}

EndEffectorIOTalonFX::EndEffectorIOTalonFX()
    : m_top{EEC::kTopMotorID, EEC::kCanBus},
      m_sideLeader{EEC::kSideLeaderMotorID, EEC::kCanBus},
      m_sideFollower{EEC::kSideFollowerMotorID, EEC::kCanBus},
      m_canRange{EEC::kCanRangeID, CANBus{EEC::kCanBus}},
      m_voltageRequest{controls::VoltageOut{0_V}.WithEnableFOC(EEC::kUseFOC)},
      m_neutralRequest{},
      m_distance{m_canRange.GetDistance()},
      m_detected{m_canRange.GetIsDetected()} {
  PhoenixUtil::ApplyMotorConfigs(m_top.device, EEC::kTopMotorConfigs, EEC::kFlashConfigRetries);
  PhoenixUtil::ApplyMotorConfigs(m_sideLeader.device, EEC::kSideMotorConfigs,
                                 EEC::kFlashConfigRetries);
  PhoenixUtil::ApplyMotorConfigs(m_sideFollower.device, EEC::kSideMotorConfigs,
                                 EEC::kFlashConfigRetries);
  m_sideFollower.device.SetControl(
      controls::Follower{EEC::kSideLeaderMotorID, EEC::kSideFollowerAlignment});
  PhoenixUtil::SpamGetStatusCode(
      [this] { return m_canRange.GetConfigurator().Apply(EEC::kCanRangeConfigs); });

  BaseStatusSignal::SetUpdateFrequencyForAll(EEC::kUpdateFrequency, m_distance, m_detected);
  m_top.device.OptimizeBusUtilization();
  m_sideLeader.device.OptimizeBusUtilization();
  m_sideFollower.device.OptimizeBusUtilization();
  m_canRange.OptimizeBusUtilization();
}

void EndEffectorIOTalonFX::UpdateInputs(EndEffectorIOInputs& inputs) {
  m_top.Refresh(inputs.top);
  m_sideLeader.Refresh(inputs.sideLeader);
  m_sideFollower.Refresh(inputs.sideFollower);

  inputs.sensorConnected = BaseStatusSignal::RefreshAll(m_distance, m_detected).IsOK();
  inputs.sensorDistanceMeters = m_distance.GetValue().value();
  // A disconnected sensor reads as "no coral" (fail safe: we never think we hold one we don't).
  inputs.hasCoral = inputs.sensorConnected && m_detected.GetValue();
}

void EndEffectorIOTalonFX::SetVoltage(double topVolts, double sideVolts) {
  m_top.device.SetControl(m_voltageRequest.WithOutput(units::volt_t{topVolts}));
  m_sideLeader.device.SetControl(m_voltageRequest.WithOutput(units::volt_t{sideVolts}));
}

void EndEffectorIOTalonFX::Off() {
  m_top.device.SetControl(m_neutralRequest);
  m_sideLeader.device.SetControl(m_neutralRequest);
}
