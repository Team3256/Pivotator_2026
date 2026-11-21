// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include "subsystems/endeffector/EndEffectorIOTalonFX.h"

/**
 * Simulated end effector: the real TalonFX IO, the roller simulated on sim::PhysicsWorld, and
 * the coral sensor answered by the world (which tracks whether a coral is in the end effector).
 */
class EndEffectorIOSim : public EndEffectorIOTalonFX {
 public:
  EndEffectorIOSim();

  void UpdateInputs(EndEffectorIOInputs& inputs) override;
};
