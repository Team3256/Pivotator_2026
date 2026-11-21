// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <array>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/MotionMagicVoltage.hpp>
#include <ctre/phoenix6/controls/NeutralOut.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>

#include "subsystems/pivot/PivotIO.h"

/** 4-motor pivot: one leader runs Motion Magic, the other three follow it. */
class PivotIOTalonFX : public PivotIO {
 public:
  PivotIOTalonFX();

  void UpdateInputs(PivotIOInputs& inputs) override;
  void SetPosition(double degrees, double gravityFeedforwardVolts) override;
  void SetVoltage(double volts) override;
  void Off() override;
  void ResetPosition(double degrees) override;

 protected:
  ctre::phoenix6::hardware::TalonFX m_leader;
  ctre::phoenix6::hardware::TalonFX m_sameSideFollower;
  ctre::phoenix6::hardware::TalonFX m_otherSideLeader;
  ctre::phoenix6::hardware::TalonFX m_otherSideFollower;
  /** All four, leader first. */
  std::array<ctre::phoenix6::hardware::TalonFX*, 4> m_motors;

 private:
  ctre::phoenix6::controls::MotionMagicVoltage m_motionMagicRequest;
  ctre::phoenix6::controls::VoltageOut m_voltageRequest;
  ctre::phoenix6::controls::NeutralOut m_neutralRequest;

  ctre::phoenix6::StatusSignal<units::turn_t>& m_position;
  ctre::phoenix6::StatusSignal<units::turns_per_second_t>& m_velocity;
  ctre::phoenix6::StatusSignal<units::volt_t>& m_voltage;
  ctre::phoenix6::StatusSignal<double>& m_closedLoopError;
  std::array<ctre::phoenix6::StatusSignal<units::ampere_t>*, 4> m_stator;
  std::array<ctre::phoenix6::StatusSignal<units::ampere_t>*, 4> m_supply;
  std::array<ctre::phoenix6::StatusSignal<units::celsius_t>*, 4> m_temps;
  std::array<ctre::phoenix6::BaseStatusSignal*, 7> m_leaderSignals;
  std::array<ctre::phoenix6::BaseStatusSignal*, 9> m_followerSignals;
};
