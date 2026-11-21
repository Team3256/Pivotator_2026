// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include "subsystems/elevator/ElevatorIOTalonFX.h"

/**
 * Simulated elevator: the real TalonFX IO (same requests, same configs), with both motors and the
 * carriage simulated on sim::PhysicsWorld (Mayhem's physics engine). The world tilts the
 * elevator's gravity with the simulated pivot, so nothing here needs the pivot angle.
 */
class ElevatorIOSim : public ElevatorIOTalonFX {
 public:
  ElevatorIOSim();

  void UpdateInputs(ElevatorIOInputs& inputs) override;
  void ResetPosition(double meters) override;
};
