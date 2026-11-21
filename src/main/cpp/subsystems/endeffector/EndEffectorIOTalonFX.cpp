// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "subsystems/endeffector/EndEffectorIOTalonFX.h"

#include "subsystems/endeffector/EndEffectorConstants.h"
#include "util/PhoenixUtil.h"

using namespace ctre::phoenix6;
using namespace units::literals;
namespace EEC = EndEffectorConstants;

EndEffectorIOTalonFX::EndEffectorIOTalonFX()
    : m_motor{EEC::kMotorID, CANBus{EEC::kCanBus}},
      m_canRange{EEC::kCanRangeID, CANBus{EEC::kCanBus}},
      m_voltageRequest{controls::VoltageOut{0_V}.WithEnableFOC(EEC::kUseFOC)},
      m_neutralRequest{},
      m_velocity{m_motor.GetVelocity()},
      m_voltage{m_motor.GetMotorVoltage()},
      m_statorCurrent{m_motor.GetStatorCurrent()},
      m_supplyCurrent{m_motor.GetSupplyCurrent()},
      m_temp{m_motor.GetDeviceTemp()},
      m_distance{m_canRange.GetDistance()},
      m_detected{m_canRange.GetIsDetected()} {
  PhoenixUtil::ApplyMotorConfigs(m_motor, EEC::kMotorConfigs, EEC::kFlashConfigRetries);
  PhoenixUtil::SpamGetStatusCode(
      [this] { return m_canRange.GetConfigurator().Apply(EEC::kCanRangeConfigs); });

  BaseStatusSignal::SetUpdateFrequencyForAll(EEC::kUpdateFrequency, m_velocity, m_voltage,
                                             m_statorCurrent, m_supplyCurrent, m_temp,
                                             m_distance, m_detected);
  m_motor.OptimizeBusUtilization();
  m_canRange.OptimizeBusUtilization();
}

void EndEffectorIOTalonFX::UpdateInputs(EndEffectorIOInputs& inputs) {
  inputs.motorConnected = BaseStatusSignal::RefreshAll(m_velocity, m_voltage, m_statorCurrent,
                                                       m_supplyCurrent, m_temp)
                              .IsOK();
  inputs.velocityRps = m_velocity.GetValue().value();
  inputs.voltage = m_voltage.GetValue().value();
  inputs.statorCurrent = m_statorCurrent.GetValue().value();
  inputs.supplyCurrent = m_supplyCurrent.GetValue().value();
  inputs.tempCelsius = m_temp.GetValue().value();

  inputs.sensorConnected = BaseStatusSignal::RefreshAll(m_distance, m_detected).IsOK();
  inputs.sensorDistanceMeters = m_distance.GetValue().value();
  // A disconnected sensor reads as "no coral" (fail safe: we never think we hold one we don't).
  inputs.hasCoral = inputs.sensorConnected && m_detected.GetValue();
}

void EndEffectorIOTalonFX::SetVoltage(double volts) {
  m_motor.SetControl(m_voltageRequest.WithOutput(units::volt_t{volts}));
}

void EndEffectorIOTalonFX::Off() {
  m_motor.SetControl(m_neutralRequest);
}
