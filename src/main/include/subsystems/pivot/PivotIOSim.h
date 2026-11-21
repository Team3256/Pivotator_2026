// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <array>
#include <functional>

#include <ctre/phoenix6/sim/TalonFXSimState.hpp>
#include <frc/simulation/SingleJointedArmSim.h>

#include "subsystems/pivot/PivotIOTalonFX.h"

/**
 * Simulated pivot. Like the elevator sim, the real TalonFX code runs and we only fake the physics.
 *
 * The load on the pivot changes as the elevator extends, so WPILib's built-in arm gravity (fixed
 * length) is turned off and we subtract the gravity voltage ourselves using the live extension.
 */
class PivotIOSim : public PivotIOTalonFX {
 public:
  /** @param elevatorMeters live elevator extension, so gravity torque grows as it extends */
  explicit PivotIOSim(std::function<double()> elevatorMeters);

  void UpdateInputs(PivotIOInputs& inputs) override;

 private:
  std::array<ctre::phoenix6::sim::TalonFXSimState*, 4> m_simStates;
  std::function<double()> m_elevatorMeters;
  frc::sim::SingleJointedArmSim m_physics;
};
