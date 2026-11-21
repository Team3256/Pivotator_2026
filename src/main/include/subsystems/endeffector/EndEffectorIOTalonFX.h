// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <ctre/phoenix6/CANrange.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/NeutralOut.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>

#include "subsystems/endeffector/EndEffectorIO.h"

/** One Kraken on the rollers, a CANrange watching for coral. */
class EndEffectorIOTalonFX : public EndEffectorIO {
 public:
  EndEffectorIOTalonFX();

  void UpdateInputs(EndEffectorIOInputs& inputs) override;
  void SetVoltage(double volts) override;
  void Off() override;

 protected:
  ctre::phoenix6::hardware::TalonFX m_motor;
  ctre::phoenix6::hardware::CANrange m_canRange;

 private:
  ctre::phoenix6::controls::VoltageOut m_voltageRequest;
  ctre::phoenix6::controls::NeutralOut m_neutralRequest;

  ctre::phoenix6::StatusSignal<units::turns_per_second_t>& m_velocity;
  ctre::phoenix6::StatusSignal<units::volt_t>& m_voltage;
  ctre::phoenix6::StatusSignal<units::ampere_t>& m_statorCurrent;
  ctre::phoenix6::StatusSignal<units::ampere_t>& m_supplyCurrent;
  ctre::phoenix6::StatusSignal<units::celsius_t>& m_temp;
  ctre::phoenix6::StatusSignal<units::meter_t>& m_distance;
  ctre::phoenix6::StatusSignal<bool>& m_detected;
};
