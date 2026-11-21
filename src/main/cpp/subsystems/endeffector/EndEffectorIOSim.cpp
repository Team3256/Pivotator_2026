// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "subsystems/endeffector/EndEffectorIOSim.h"

#include "sim/PhysicsWorld.h"
#include "subsystems/endeffector/EndEffectorConstants.h"

namespace EEC = EndEffectorConstants;

EndEffectorIOSim::EndEffectorIOSim() {
  sim::PhysicsWorld::Get().AttachMotor(sim::Mechanism::kEndEffector, m_motor, EEC::kMotorConfigs,
                                       mayhem::sim::DcMotor::KrakenX44(),
                                       mayhem::sim::DcMotor::KrakenX44Foc());
}

void EndEffectorIOSim::UpdateInputs(EndEffectorIOInputs& inputs) {
  auto& world = sim::PhysicsWorld::Get();
  const auto& motor = world.Motor(m_motor);
  inputs.motorConnected = true;
  inputs.velocityRps = motor.Velocity();
  inputs.voltage = motor.MotorVoltage();
  inputs.statorCurrent = motor.StatorCurrent();
  inputs.supplyCurrent = motor.SupplyCurrent();
  inputs.tempCelsius = 25.0;
  inputs.sensorConnected = true;
  inputs.hasCoral = world.HasCoral();
  inputs.sensorDistanceMeters = inputs.hasCoral ? 0.03 : 0.5;
}
