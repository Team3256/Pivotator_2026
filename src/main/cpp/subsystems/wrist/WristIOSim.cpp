// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "subsystems/wrist/WristIOSim.h"

#include "sim/PhysicsWorld.h"
#include "subsystems/wrist/WristConstants.h"

namespace WC = WristConstants;

WristIOSim::WristIOSim() {
  auto& world = sim::PhysicsWorld::Get();
  for (auto* motor : {&m_leader, &m_follower}) {
    world.AttachMotor(sim::Mechanism::kWrist, *motor, WC::kMotorConfigs,
                      mayhem::sim::DcMotor::KrakenX44(), mayhem::sim::DcMotor::KrakenX44Foc());
  }
}

void WristIOSim::UpdateInputs(WristIOInputs& inputs) {
  const auto& world = sim::PhysicsWorld::Get();
  const auto& leader = world.Motor(m_leader);
  const auto& follower = world.Motor(m_follower);
  inputs.leaderConnected = true;
  inputs.followerConnected = true;
  inputs.positionDeg = leader.Position() * 360.0;
  inputs.velocityDegPerSec = leader.Velocity() * 360.0;
  inputs.leaderVoltage = leader.MotorVoltage();
  inputs.leaderStatorCurrent = leader.StatorCurrent();
  inputs.followerStatorCurrent = follower.StatorCurrent();
  inputs.leaderTempCelsius = 25.0;
  inputs.followerTempCelsius = 25.0;
}

void WristIOSim::ResetPosition(double degrees) {
  WristIOTalonFX::ResetPosition(degrees);
  if (sim::PhysicsWorld::Exists()) {
    auto& world = sim::PhysicsWorld::Get();
    world.SetMechanismPosition(m_leader, degrees / 360.0);
    world.SetMechanismPosition(m_follower, degrees / 360.0);
  }
}
