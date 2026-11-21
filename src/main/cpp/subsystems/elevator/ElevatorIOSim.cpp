// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "subsystems/elevator/ElevatorIOSim.h"

#include "sim/PhysicsWorld.h"
#include "subsystems/elevator/ElevatorConstants.h"

namespace EC = ElevatorConstants;

ElevatorIOSim::ElevatorIOSim() {
  auto& world = sim::PhysicsWorld::Get();
  for (auto* motor : {&m_leader, &m_follower}) {
    world.AttachMotor(sim::Mechanism::kElevator, *motor, EC::kMotorConfigs,
                      mayhem::sim::DcMotor::KrakenX60(), mayhem::sim::DcMotor::KrakenX60Foc());
  }
}

void ElevatorIOSim::UpdateInputs(ElevatorIOInputs& inputs) {
  const auto& world = sim::PhysicsWorld::Get();
  const auto& leader = world.Motor(m_leader);
  const auto& follower = world.Motor(m_follower);
  inputs.leaderConnected = true;
  inputs.leaderPositionMeters = leader.Position();
  inputs.leaderVelocityMetersPerSec = leader.Velocity();
  inputs.leaderVoltage = leader.MotorVoltage();
  inputs.leaderStatorCurrent = leader.StatorCurrent();
  inputs.leaderSupplyCurrent = leader.SupplyCurrent();
  inputs.leaderTempCelsius = 25.0;
  inputs.closedLoopReferenceMeters = leader.ClosedLoopReference();
  inputs.closedLoopErrorMeters = leader.ClosedLoopError();

  inputs.followerConnected = true;
  inputs.followerPositionMeters = follower.Position();
  inputs.followerVelocityMetersPerSec = follower.Velocity();
  inputs.followerVoltage = follower.MotorVoltage();
  inputs.followerStatorCurrent = follower.StatorCurrent();
  inputs.followerSupplyCurrent = follower.SupplyCurrent();
  inputs.followerTempCelsius = 25.0;
}

void ElevatorIOSim::ResetPosition(double meters) {
  ElevatorIOTalonFX::ResetPosition(meters);
  if (sim::PhysicsWorld::Exists()) {
    auto& world = sim::PhysicsWorld::Get();
    world.SetMechanismPosition(m_leader, meters);
    world.SetMechanismPosition(m_follower, meters);
  }
}
