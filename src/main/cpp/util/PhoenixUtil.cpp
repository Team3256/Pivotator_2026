// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "util/PhoenixUtil.h"

#include <cstdio>
#include <string>

#include <frc/Errors.h>
#include <frc/RobotBase.h>

using namespace ctre::phoenix6;

namespace {

constexpr bool kLogInequality = true;
constexpr double kTalonConfigEpsilon = 0.05;
constexpr double kEpsilon = 1e-12;

// Phoenix C++ configs mix plain numbers and units types. Raw() turns either into a double.
double Raw(double value) {
  return value;
}
template <typename Unit>
double Raw(Unit value) {
  return value.value();
}

template <typename A, typename B>
bool Near(A a, B b, double epsilon = kTalonConfigEpsilon) {
  double x = Raw(a);
  double y = Raw(b);
  return (x - epsilon <= y) && (x + epsilon >= y);
}

bool Report(bool equal, const char* what) {
  if (kLogInequality && !equal) {
    std::printf("%s not equal\n", what);
  }
  return equal;
}

template <typename Slot>
bool SlotEqual(const Slot& a, const Slot& b, const char* what) {
  return Report(Near(a.kP, b.kP) && Near(a.kI, b.kI) && Near(a.kD, b.kD) &&
                    Near(a.kV, b.kV) && Near(a.kS, b.kS),
                what);
}

bool IsEqual(const configs::MotorOutputConfigs& a, const configs::MotorOutputConfigs& b) {
  return Report(a.Inverted == b.Inverted && a.NeutralMode == b.NeutralMode &&
                    Near(a.DutyCycleNeutralDeadband, b.DutyCycleNeutralDeadband) &&
                    Near(a.PeakForwardDutyCycle, b.PeakForwardDutyCycle) &&
                    Near(a.PeakReverseDutyCycle, b.PeakReverseDutyCycle),
                "MotorOutputConfigs");
}

bool IsEqual(const configs::CurrentLimitsConfigs& a, const configs::CurrentLimitsConfigs& b) {
  return Report(Near(a.StatorCurrentLimit, b.StatorCurrentLimit) &&
                    Near(a.SupplyCurrentLimit, b.SupplyCurrentLimit) &&
                    a.StatorCurrentLimitEnable == b.StatorCurrentLimitEnable &&
                    a.SupplyCurrentLimitEnable == b.SupplyCurrentLimitEnable,
                "CurrentLimitsConfigs");
}

bool IsEqual(const configs::VoltageConfigs& a, const configs::VoltageConfigs& b) {
  return Report(Near(a.SupplyVoltageTimeConstant, b.SupplyVoltageTimeConstant) &&
                    Near(a.PeakForwardVoltage, b.PeakForwardVoltage, kEpsilon) &&
                    Near(a.PeakReverseVoltage, b.PeakReverseVoltage, kEpsilon),
                "VoltageConfigs");
}

bool IsEqual(const configs::TorqueCurrentConfigs& a, const configs::TorqueCurrentConfigs& b) {
  return Report(Near(a.PeakForwardTorqueCurrent, b.PeakForwardTorqueCurrent) &&
                    Near(a.PeakReverseTorqueCurrent, b.PeakReverseTorqueCurrent) &&
                    Near(a.TorqueNeutralDeadband, b.TorqueNeutralDeadband),
                "TorqueCurrentConfigs");
}

bool IsEqual(const configs::FeedbackConfigs& a, const configs::FeedbackConfigs& b) {
  bool equal = Near(a.FeedbackRotorOffset, b.FeedbackRotorOffset) &&
               Near(a.SensorToMechanismRatio, b.SensorToMechanismRatio) &&
               Near(a.RotorToSensorRatio, b.RotorToSensorRatio) &&
               a.FeedbackSensorSource == b.FeedbackSensorSource &&
               a.FeedbackRemoteSensorID == b.FeedbackRemoteSensorID;
  if (kLogInequality && !equal) {
    std::printf("A-SensorToMechanismRatio: %f, B-SensorToMechanismRatio: %f\n",
                Raw(a.SensorToMechanismRatio), Raw(b.SensorToMechanismRatio));
    std::printf("A-RotorToSensorRatio: %f, B-RotorToSensorRatio: %f\n",
                Raw(a.RotorToSensorRatio), Raw(b.RotorToSensorRatio));
  }
  return Report(equal, "FeedbackConfigs");
}

bool IsEqual(const configs::OpenLoopRampsConfigs& a, const configs::OpenLoopRampsConfigs& b) {
  return Report(Near(a.DutyCycleOpenLoopRampPeriod, b.DutyCycleOpenLoopRampPeriod) &&
                    Near(a.VoltageOpenLoopRampPeriod, b.VoltageOpenLoopRampPeriod) &&
                    Near(a.TorqueOpenLoopRampPeriod, b.TorqueOpenLoopRampPeriod),
                "OpenLoopRampsConfigs");
}

bool IsEqual(const configs::ClosedLoopRampsConfigs& a,
             const configs::ClosedLoopRampsConfigs& b) {
  return Report(Near(a.DutyCycleClosedLoopRampPeriod, b.DutyCycleClosedLoopRampPeriod) &&
                    Near(a.VoltageClosedLoopRampPeriod, b.VoltageClosedLoopRampPeriod) &&
                    Near(a.TorqueClosedLoopRampPeriod, b.TorqueClosedLoopRampPeriod),
                "ClosedLoopRampsConfigs");
}

bool IsEqual(const configs::HardwareLimitSwitchConfigs& a,
             const configs::HardwareLimitSwitchConfigs& b) {
  return Report(
      a.ForwardLimitAutosetPositionEnable == b.ForwardLimitAutosetPositionEnable &&
          a.ForwardLimitEnable == b.ForwardLimitEnable &&
          a.ReverseLimitAutosetPositionEnable == b.ReverseLimitAutosetPositionEnable &&
          a.ReverseLimitEnable == b.ReverseLimitEnable &&
          Near(a.ForwardLimitAutosetPositionValue, b.ForwardLimitAutosetPositionValue) &&
          Near(a.ReverseLimitAutosetPositionValue, b.ReverseLimitAutosetPositionValue) &&
          a.ForwardLimitRemoteSensorID == b.ForwardLimitRemoteSensorID &&
          a.ReverseLimitRemoteSensorID == b.ReverseLimitRemoteSensorID &&
          a.ForwardLimitSource == b.ForwardLimitSource &&
          a.ReverseLimitSource == b.ReverseLimitSource &&
          a.ForwardLimitType == b.ForwardLimitType && a.ReverseLimitType == b.ReverseLimitType,
      "HardwareLimitSwitchConfigs");
}

bool IsEqual(const configs::SoftwareLimitSwitchConfigs& a,
             const configs::SoftwareLimitSwitchConfigs& b) {
  return Report(Near(a.ForwardSoftLimitThreshold, b.ForwardSoftLimitThreshold) &&
                    Near(a.ReverseSoftLimitThreshold, b.ReverseSoftLimitThreshold) &&
                    a.ForwardSoftLimitEnable == b.ForwardSoftLimitEnable &&
                    a.ReverseSoftLimitEnable == b.ReverseSoftLimitEnable,
                "SoftwareLimitSwitchConfigs");
}

bool IsEqual(const configs::MotionMagicConfigs& a, const configs::MotionMagicConfigs& b) {
  return Report(Near(a.MotionMagicAcceleration, b.MotionMagicAcceleration) &&
                    Near(a.MotionMagicCruiseVelocity, b.MotionMagicCruiseVelocity) &&
                    Near(a.MotionMagicJerk, b.MotionMagicJerk),
                "MotionMagicConfigs");
}

std::string Attempt(int i, int numTries) {
  return " (attempt " + std::to_string(i + 1) + " of " + std::to_string(numTries) + ")";
}

}  // namespace

bool PhoenixUtil::SpamGetStatusCode(const std::function<ctre::phoenix::StatusCode()>& function,
                                    int numTries) {
  ctre::phoenix::StatusCode code = function();
  int tries = 0;
  while (!code.IsOK() && tries < numTries) {
    FRC_ReportWarning("Retrying CTRE Device Config {}", code.GetName());
    code = function();
    tries++;
  }
  if (!code.IsOK()) {
    FRC_ReportError(frc::err::Error, "Failed to execute phoenix pro api call after {} attempts",
                    numTries);
    return false;
  }
  return true;
}

bool PhoenixUtil::IsEqual(const configs::TalonFXConfiguration& a,
                          const configs::TalonFXConfiguration& b) {
  return SlotEqual(a.Slot0, b.Slot0, "Slot0Configs") &&
         SlotEqual(a.Slot1, b.Slot1, "Slot1Configs") &&
         SlotEqual(a.Slot2, b.Slot2, "Slot2Configs") &&
         ::IsEqual(a.MotorOutput, b.MotorOutput) && ::IsEqual(a.CurrentLimits, b.CurrentLimits) &&
         ::IsEqual(a.Voltage, b.Voltage) && ::IsEqual(a.TorqueCurrent, b.TorqueCurrent) &&
         ::IsEqual(a.Feedback, b.Feedback) && ::IsEqual(a.OpenLoopRamps, b.OpenLoopRamps) &&
         ::IsEqual(a.ClosedLoopRamps, b.ClosedLoopRamps) &&
         ::IsEqual(a.HardwareLimitSwitch, b.HardwareLimitSwitch) &&
         Report(a.Audio.BeepOnBoot == b.Audio.BeepOnBoot, "AudioConfigs") &&
         ::IsEqual(a.SoftwareLimitSwitch, b.SoftwareLimitSwitch) &&
         ::IsEqual(a.MotionMagic, b.MotionMagic);
}

bool PhoenixUtil::ReadAndVerifyConfiguration(hardware::TalonFX& talon,
                                             const configs::TalonFXConfiguration& config) {
  configs::TalonFXConfiguration readConfig{};
  if (!SpamGetStatusCode([&] { return talon.GetConfigurator().Refresh(readConfig); })) {
    FRC_ReportWarning("Failed to read config for talon [{}]", talon.GetDeviceID());
    return false;
  }
  if (!IsEqual(config, readConfig)) {
    FRC_ReportWarning("Configuration verification failed for talon [{}]", talon.GetDeviceID());
    return false;
  }
  return true;
}

bool PhoenixUtil::ApplyMotorConfigs(hardware::TalonFX& motor,
                                    const configs::TalonFXConfiguration& config, int numTries) {
  for (int i = 0; i < numTries; i++) {
    if (frc::RobotBase::IsSimulation()) {
      return motor.GetConfigurator().Apply(config).IsOK();
    }
    if (SpamGetStatusCode([&] { return motor.GetConfigurator().Apply(config); })) {
      // API says we applied the config, make sure it's actually right.
      if (ReadAndVerifyConfiguration(motor, config)) {
        return true;
      }
      FRC_ReportWarning("Failed to verify config for talon [{}]{}", motor.GetDeviceID(),
                        Attempt(i, numTries));
    } else {
      FRC_ReportWarning("Failed to apply config for talon [{}]{}", motor.GetDeviceID(),
                        Attempt(i, numTries));
    }
  }
  FRC_ReportError(frc::err::Error, "Failed to apply config for talon after {} attempts",
                  numTries);
  return false;
}
