// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "subsystems/wrist/WristIOSim.h"

#include "sim/PhysicsWorld.h"
#include "subsystems/wrist/WristConstants.h"

namespace WC = WristConstants;

WristIOSim::WristIOSim() {
  sim::PhysicsWorld::Get().AttachMotor(sim::Mechanism::kWrist, m_motor, WC::kMotorConfigs,
                                       mayhem::sim::DcMotor::KrakenX44(),
                                       mayhem::sim::DcMotor::KrakenX44Foc());
}

void WristIOSim::UpdateInputs(WristIOInputs& inputs) {
  const auto& motor = sim::PhysicsWorld::Get().Motor(m_motor);
  inputs.motorConnected = true;
  inputs.positionDeg = motor.Position() * 360.0;
  inputs.velocityDegPerSec = motor.Velocity() * 360.0;
  inputs.voltage = motor.MotorVoltage();
  inputs.statorCurrent = motor.StatorCurrent();
  inputs.tempCelsius = 25.0;
}

void WristIOSim::ResetPosition(double degrees) {
  WristIOTalonFX::ResetPosition(degrees);
  if (sim::PhysicsWorld::Exists()) {
    sim::PhysicsWorld::Get().SetMechanismPosition(m_motor, degrees / 360.0);
  }
}
