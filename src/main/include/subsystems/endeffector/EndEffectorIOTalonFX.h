// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <ctre/phoenix6/CANrange.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/NeutralOut.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>

#include "subsystems/endeffector/EndEffectorIO.h"

/** A Kraken X60 on the top roller, two Kraken X44s on the side wheels, a CANrange for coral. */
class EndEffectorIOTalonFX : public EndEffectorIO {
 public:
  EndEffectorIOTalonFX();

  void UpdateInputs(EndEffectorIOInputs& inputs) override;
  void SetVoltage(double topVolts, double sideVolts) override;
  void Off() override;

 protected:
  /** A motor and the status signals read from it. */
  struct Motor {
    Motor(int id, std::string_view bus);
    bool Refresh(RollerMotorInputs& out);

    ctre::phoenix6::hardware::TalonFX device;
    ctre::phoenix6::StatusSignal<units::turns_per_second_t>& velocity;
    ctre::phoenix6::StatusSignal<units::volt_t>& voltage;
    ctre::phoenix6::StatusSignal<units::ampere_t>& statorCurrent;
    ctre::phoenix6::StatusSignal<units::ampere_t>& supplyCurrent;
    ctre::phoenix6::StatusSignal<units::celsius_t>& temp;
  };

  Motor m_top;
  Motor m_sideLeader;
  Motor m_sideFollower;
  ctre::phoenix6::hardware::CANrange m_canRange;

 private:
  ctre::phoenix6::controls::VoltageOut m_voltageRequest;
  ctre::phoenix6::controls::NeutralOut m_neutralRequest;

  ctre::phoenix6::StatusSignal<units::meter_t>& m_distance;
  ctre::phoenix6::StatusSignal<bool>& m_detected;
};
