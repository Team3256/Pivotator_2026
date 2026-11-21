// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "subsystems/pivot/PivotIOSim.h"

#include "sim/PhysicsWorld.h"
#include "subsystems/pivot/PivotConstants.h"

namespace PC = PivotConstants;

PivotIOSim::PivotIOSim() {
  auto& world = sim::PhysicsWorld::Get();
  for (auto* motor : m_motors) {
    world.AttachMotor(sim::Mechanism::kPivot, *motor, PC::kMotorConfigs,
                      mayhem::sim::DcMotor::KrakenX60(), mayhem::sim::DcMotor::KrakenX60Foc());
  }
}

void PivotIOSim::UpdateInputs(PivotIOInputs& inputs) {
  const auto& world = sim::PhysicsWorld::Get();
  const auto& leader = world.Motor(m_leader);
  inputs.leaderConnected = true;
  inputs.followersConnected = true;
  // Phoenix reports pivot rotations; the subsystem works in degrees.
  inputs.positionDeg = leader.Position() * 360.0;
  inputs.velocityDegPerSec = leader.Velocity() * 360.0;
  inputs.leaderVoltage = leader.MotorVoltage();
  inputs.closedLoopErrorDeg = leader.ClosedLoopError() * 360.0;
  for (int i = 0; i < 4; i++) {
    const auto& m = world.Motor(*m_motors[i]);
    inputs.statorCurrents[i] = m.StatorCurrent();
    inputs.supplyCurrents[i] = m.SupplyCurrent();
    inputs.tempsCelsius[i] = 25.0;
  }
}

void PivotIOSim::ResetPosition(double degrees) {
  PivotIOTalonFX::ResetPosition(degrees);
  if (sim::PhysicsWorld::Exists()) {
    for (auto* motor : m_motors) {
      sim::PhysicsWorld::Get().SetMechanismPosition(*motor, degrees / 360.0);
    }
  }
}
