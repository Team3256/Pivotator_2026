// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <functional>

#include <ctre/phoenix6/sim/TalonFXSimState.hpp>
#include <frc/simulation/ElevatorSim.h>

#include "subsystems/elevator/ElevatorIOTalonFX.h"

/**
 * Simulated elevator. Reuses all the real TalonFX code (so the onboard Motion Magic loop + gains
 * run in sim too) and just feeds the Talon sim states from a WPILib ElevatorSim physics model.
 *
 * Because the pivot tilts the elevator, WPILib's built-in gravity (always straight down the
 * elevator) would be wrong. So gravity is turned off in ElevatorSim and we subtract the voltage
 * gravity "steals" ourselves, using the live pivot angle.
 */
class ElevatorIOSim : public ElevatorIOTalonFX {
 public:
  /** @param pivotAngleRad live pivot angle, radians above horizontal (pi/2 = vertical) */
  explicit ElevatorIOSim(std::function<double()> pivotAngleRad);

  void UpdateInputs(ElevatorIOInputs& inputs) override;

 private:
  ctre::phoenix6::sim::TalonFXSimState& m_leaderSim;
  ctre::phoenix6::sim::TalonFXSimState& m_followerSim;
  std::function<double()> m_pivotAngleRad;
  frc::sim::ElevatorSim m_physics;
};
