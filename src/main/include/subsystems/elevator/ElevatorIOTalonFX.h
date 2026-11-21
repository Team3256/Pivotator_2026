// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/MotionMagicVoltage.hpp>
#include <ctre/phoenix6/controls/NeutralOut.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>

#include "subsystems/elevator/ElevatorIO.h"

/**
 * Two Kraken/Falcon elevator: leader runs closed loop, follower mirrors it with a Follower
 * request.
 *
 * Because SensorToMechanismRatio = rotor rotations per meter, every Phoenix "rotation" in here
 * is actually a meter of carriage travel.
 */
class ElevatorIOTalonFX : public ElevatorIO {
 public:
  ElevatorIOTalonFX();

  void UpdateInputs(ElevatorIOInputs& inputs) override;
  void SetPosition(double meters, double gravityFeedforwardVolts) override;
  void SetVoltage(double volts) override;
  void SetVoltageIgnoringLimits(double volts) override;
  void Off() override;
  void ResetPosition(double meters) override;

 protected:
  // Declared first so they're constructed before the signals below.
  ctre::phoenix6::hardware::TalonFX m_leader;
  ctre::phoenix6::hardware::TalonFX m_follower;

 private:
  /** Flip to false to use plain PositionVoltage instead of Motion Magic (e.g. while tuning kP). */
  static constexpr bool kUseMotionMagic = true;

  ctre::phoenix6::controls::MotionMagicVoltage m_motionMagicRequest;
  ctre::phoenix6::controls::PositionVoltage m_positionRequest;
  ctre::phoenix6::controls::VoltageOut m_voltageRequest;
  ctre::phoenix6::controls::NeutralOut m_neutralRequest;

  ctre::phoenix6::StatusSignal<units::turn_t>& m_leaderPosition;
  ctre::phoenix6::StatusSignal<units::turns_per_second_t>& m_leaderVelocity;
  ctre::phoenix6::StatusSignal<units::volt_t>& m_leaderVoltage;
  ctre::phoenix6::StatusSignal<units::ampere_t>& m_leaderStatorCurrent;
  ctre::phoenix6::StatusSignal<units::ampere_t>& m_leaderSupplyCurrent;
  ctre::phoenix6::StatusSignal<units::celsius_t>& m_leaderTemp;
  ctre::phoenix6::StatusSignal<double>& m_closedLoopReference;
  ctre::phoenix6::StatusSignal<double>& m_closedLoopError;

  ctre::phoenix6::StatusSignal<units::turn_t>& m_followerPosition;
  ctre::phoenix6::StatusSignal<units::turns_per_second_t>& m_followerVelocity;
  ctre::phoenix6::StatusSignal<units::volt_t>& m_followerVoltage;
  ctre::phoenix6::StatusSignal<units::ampere_t>& m_followerStatorCurrent;
  ctre::phoenix6::StatusSignal<units::ampere_t>& m_followerSupplyCurrent;
  ctre::phoenix6::StatusSignal<units::celsius_t>& m_followerTemp;
};
