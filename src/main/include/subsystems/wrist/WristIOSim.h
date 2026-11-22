// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include "subsystems/wrist/WristIOTalonFX.h"

/**
 * Simulated wrist: the real TalonFX IO, its motor and the wrist simulated on sim::PhysicsWorld.
 * Its gravity depends on (pivot + wrist); the world handles that.
 */
class WristIOSim : public WristIOTalonFX {
 public:
  WristIOSim();

  void UpdateInputs(WristIOInputs& inputs) override;
  void ResetPosition(double degrees) override;
};
