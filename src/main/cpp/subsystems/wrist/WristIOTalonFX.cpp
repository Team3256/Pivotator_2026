// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "subsystems/wrist/WristIOTalonFX.h"

#include "subsystems/wrist/WristConstants.h"
#include "util/PhoenixUtil.h"

using namespace ctre::phoenix6;
using namespace units::literals;
namespace WC = WristConstants;

WristIOTalonFX::WristIOTalonFX()
    : m_motor{WC::kMotorID, CANBus{WC::kCanBus}},
      m_motionMagicRequest{
          controls::MotionMagicVoltage{0_tr}.WithSlot(0).WithEnableFOC(WC::kUseFOC)},
      m_voltageRequest{controls::VoltageOut{0_V}.WithEnableFOC(WC::kUseFOC)},
      m_neutralRequest{},
      m_position{m_motor.GetPosition()},
      m_velocity{m_motor.GetVelocity()},
      m_voltage{m_motor.GetMotorVoltage()},
      m_stator{m_motor.GetStatorCurrent()},
      m_temp{m_motor.GetDeviceTemp()} {
  PhoenixUtil::ApplyMotorConfigs(m_motor, WC::kMotorConfigs, WC::kFlashConfigRetries);

  BaseStatusSignal::SetUpdateFrequencyForAll(WC::kUpdateFrequency, m_position, m_velocity,
                                             m_voltage, m_stator, m_temp);
  m_motor.OptimizeBusUtilization();

  ResetPosition(WC::kBootAngleDeg);
}

void WristIOTalonFX::UpdateInputs(WristIOInputs& inputs) {
  inputs.motorConnected =
      BaseStatusSignal::RefreshAll(m_position, m_velocity, m_voltage, m_stator, m_temp).IsOK();
  inputs.positionDeg = m_position.GetValue().value() * 360.0;
  inputs.velocityDegPerSec = m_velocity.GetValue().value() * 360.0;
  inputs.voltage = m_voltage.GetValue().value();
  inputs.statorCurrent = m_stator.GetValue().value();
  inputs.tempCelsius = m_temp.GetValue().value();
}

void WristIOTalonFX::SetPosition(double degrees, double gravityFeedforwardVolts) {
  m_motor.SetControl(m_motionMagicRequest.WithPosition(units::turn_t{degrees / 360.0})
                         .WithFeedForward(units::volt_t{gravityFeedforwardVolts}));
}

void WristIOTalonFX::SetVoltage(double volts) {
  m_motor.SetControl(m_voltageRequest.WithOutput(units::volt_t{volts}));
}

void WristIOTalonFX::Off() {
  m_motor.SetControl(m_neutralRequest);
}

void WristIOTalonFX::ResetPosition(double degrees) {
  m_motor.SetPosition(units::turn_t{degrees / 360.0});
}
