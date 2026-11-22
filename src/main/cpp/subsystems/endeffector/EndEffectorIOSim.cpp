// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "subsystems/endeffector/EndEffectorIOSim.h"

#include "sim/PhysicsWorld.h"
#include "subsystems/endeffector/EndEffectorConstants.h"

namespace EEC = EndEffectorConstants;

namespace {

void Read(const mayhem::sim::TalonFx& motor, RollerMotorInputs& out) {
  out.connected = true;
  out.velocityRps = motor.Velocity();
  out.voltage = motor.MotorVoltage();
  out.statorCurrent = motor.StatorCurrent();
  out.supplyCurrent = motor.SupplyCurrent();
  out.tempCelsius = 25.0;
}

}  // namespace

EndEffectorIOSim::EndEffectorIOSim() {
  auto& world = sim::PhysicsWorld::Get();
  world.AttachMotor(sim::Mechanism::kEndEffectorTop, m_top.device, EEC::kTopMotorConfigs,
                    mayhem::sim::DcMotor::KrakenX60(), mayhem::sim::DcMotor::KrakenX60Foc());
  for (auto* side : {&m_sideLeader, &m_sideFollower}) {
    world.AttachMotor(sim::Mechanism::kEndEffectorSides, side->device, EEC::kSideMotorConfigs,
                      mayhem::sim::DcMotor::KrakenX44(), mayhem::sim::DcMotor::KrakenX44Foc());
  }
}

void EndEffectorIOSim::UpdateInputs(EndEffectorIOInputs& inputs) {
  auto& world = sim::PhysicsWorld::Get();
  Read(world.Motor(m_top.device), inputs.top);
  Read(world.Motor(m_sideLeader.device), inputs.sideLeader);
  Read(world.Motor(m_sideFollower.device), inputs.sideFollower);
  inputs.sensorConnected = true;
  inputs.hasCoral = world.HasCoral();
  inputs.sensorDistanceMeters = inputs.hasCoral ? 0.03 : 0.5;
}
