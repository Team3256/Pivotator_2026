// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/MotionMagicVoltage.hpp>
#include <ctre/phoenix6/controls/NeutralOut.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>

#include "subsystems/wrist/WristIO.h"

class WristIOTalonFX : public WristIO {
 public:
  WristIOTalonFX();

  void UpdateInputs(WristIOInputs& inputs) override;
  void SetPosition(double degrees, double gravityFeedforwardVolts) override;
  void SetVoltage(double volts) override;
  void Off() override;
  void ResetPosition(double degrees) override;

 protected:
  ctre::phoenix6::hardware::TalonFX m_leader;
  ctre::phoenix6::hardware::TalonFX m_follower;

 private:
  ctre::phoenix6::controls::MotionMagicVoltage m_motionMagicRequest;
  ctre::phoenix6::controls::VoltageOut m_voltageRequest;
  ctre::phoenix6::controls::NeutralOut m_neutralRequest;

  ctre::phoenix6::StatusSignal<units::turn_t>& m_position;
  ctre::phoenix6::StatusSignal<units::turns_per_second_t>& m_velocity;
  ctre::phoenix6::StatusSignal<units::volt_t>& m_voltage;
  ctre::phoenix6::StatusSignal<units::ampere_t>& m_leaderStator;
  ctre::phoenix6::StatusSignal<units::celsius_t>& m_leaderTemp;
  ctre::phoenix6::StatusSignal<units::ampere_t>& m_followerStator;
  ctre::phoenix6::StatusSignal<units::celsius_t>& m_followerTemp;
};
