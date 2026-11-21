// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <functional>

#include <ctre/phoenix6/sim/TalonFXSimState.hpp>
#include <frc/simulation/SingleJointedArmSim.h>

#include "subsystems/wrist/WristIOTalonFX.h"

/** Simulated wrist. Gravity depends on (pivot + wrist), so we apply it ourselves. */
class WristIOSim : public WristIOTalonFX {
 public:
  explicit WristIOSim(std::function<double()> pivotAngleRad);

  void UpdateInputs(WristIOInputs& inputs) override;

 private:
  ctre::phoenix6::sim::TalonFXSimState& m_leaderSim;
  ctre::phoenix6::sim::TalonFXSimState& m_followerSim;
  std::function<double()> m_pivotAngleRad;
  frc::sim::SingleJointedArmSim m_physics;
};
