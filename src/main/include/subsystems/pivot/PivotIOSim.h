// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include "subsystems/pivot/PivotIOTalonFX.h"

/**
 * Simulated pivot: the real TalonFX IO, all four motors and the pivot (carrying the elevator,
 * whose load grows as it extends) simulated on sim::PhysicsWorld.
 */
class PivotIOSim : public PivotIOTalonFX {
 public:
  PivotIOSim();

  void UpdateInputs(PivotIOInputs& inputs) override;
  void ResetPosition(double degrees) override;
};
