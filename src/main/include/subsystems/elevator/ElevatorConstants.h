// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <numbers>
#include <string_view>

#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/system/plant/DCMotor.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_jerk.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/time.h>
#include <units/torque.h>

/**
 * Elevator constants.
 *
 * UNITS: every position in this subsystem is in METERS of carriage travel (0 = fully down), and
 * every velocity is in meters/second. We get this for free by setting the TalonFX
 * SensorToMechanismRatio to "rotor rotations per meter", so the Talon's onboard position/Motion
 * Magic loops work directly in meters. (Phoenix still calls them "turns" in C++, so 1_tr here
 * means 1 meter.)
 *
 * Anything marked TODO must be measured/tuned on the real robot.
 */
namespace ElevatorConstants {

using namespace units::literals;

// ---------------- Hardware ----------------
inline constexpr int kLeaderMotorID = 20;    // TODO: real CAN ID
inline constexpr int kFollowerMotorID = 21;  // TODO: real CAN ID
inline constexpr std::string_view kCanBus = "";  // "" = roboRIO bus. CANivore name if used.

/**
 * CAD: each motor's pinion drives its own 50T gear and the two 50T gears mesh each other, so the
 * motors spin in opposite directions. VERIFY on the real gearbox before first enable.
 */
inline const ctre::phoenix6::signals::MotorAlignmentValue kFollowerAlignment =
    ctre::phoenix6::signals::MotorAlignmentValue::Opposed;

inline constexpr bool kUseFOC = true;
inline constexpr int kFlashConfigRetries = 5;
inline constexpr units::hertz_t kUpdateFrequency = 50_Hz;

// ---------------- Geometry (from Onshape "00. Layout and Block" variables) ----------------
/** 2x Kraken X60, 12T SplineXS pinion -> 50T 20DP gear on the drive shaft = 4.1667:1. */
inline constexpr double kGearRatio = 50.0 / 12.0;

/** 30T HTD5 drive pulley (WCP-0562). Pitch diameter = teeth * 5 mm / pi. */
inline constexpr double kDrivePulleyTeeth = 30;
inline constexpr double kDrumRadiusMeters = (kDrivePulleyTeeth * 0.005 / std::numbers::pi) / 2.0;

/**
 * 2-stage, continuous rigging (belt ends clamped in the carriage), so the carriage moves exactly
 * as far as the belt does: multiplier 1. Would be 2 for a cascade measured at the carriage.
 */
inline constexpr double kStageMultiplier = 1.0;

/** Rotor rotations per meter of carriage travel. Used as the Talon SensorToMechanismRatio. */
inline constexpr double kRotorRotationsPerMeter =
    kGearRatio / (2.0 * std::numbers::pi * kDrumRadiusMeters * kStageMultiplier);

inline constexpr double kMinHeightMeters = 0.0;
/** CAD: 18 in 1st stage + 18 in carriage = 36 in of carriage travel. */
inline constexpr double kMaxHeightMeters = units::meter_t{36_in}.value();

// ---------------- Setpoints ----------------
/**
 * Coral scoring / handling heights, in meters of carriage travel above fully retracted (max
 * 0.914). PLACEHOLDERS: the real values depend on the pivot angle used at each level, so tune
 * them on the robot together with the pivot setpoints.
 */
enum class Height { kStow, kCoralIntake, kL1, kL2, kL3, kL4 };

constexpr double Meters(Height height) {
  switch (height) {
    case Height::kStow:
      return 0.0;
    case Height::kCoralIntake:
      return 0.0;
    case Height::kL1:
      return 0.10;
    case Height::kL2:
      return 0.30;
    case Height::kL3:
      return 0.55;
    case Height::kL4:
      return 0.89;
  }
  return 0.0;
}

constexpr std::string_view Name(Height height) {
  switch (height) {
    case Height::kStow:
      return "STOW";
    case Height::kCoralIntake:
      return "CORAL_INTAKE";
    case Height::kL1:
      return "L1";
    case Height::kL2:
      return "L2";
    case Height::kL3:
      return "L3";
    case Height::kL4:
      return "L4";
  }
  return "UNKNOWN";
}

/**
 * The pivot swings the WHOLE elevator, so the Superstructure pulls the carriage in below this
 * height before making big pivot moves. TODO: tune; lower = safer but slower cycles.
 */
inline constexpr double kPivotSafeMaxHeightMeters = 0.15;

/** How close (meters) the carriage must be to the setpoint to count as "there". */
inline constexpr double kPositionToleranceMeters = 0.015;

// ---------------- Homing ----------------
/** Voltage used to gently drive down onto the hard stop when homing. */
inline constexpr double kHomingVoltage = -1.5;

/** Stator current (A) that means we've hit the bottom hard stop. TODO: tune. */
inline constexpr double kHomingCurrentThresholdAmps = 30.0;

/** Must see the stall for this long before we trust it (filters the startup spike). */
inline constexpr units::second_t kHomingDebounce = 0.15_s;

inline constexpr units::second_t kHomingTimeout = 3.0_s;

// ---------------- Simulation ----------------
namespace Sim {
/** Carriage + wrist + end effector + coral. TODO: weigh, or get mass properties from CAD. */
inline constexpr double kCarriageMassKg = 6.0;
inline constexpr double kStartingHeightMeters = 0.0;
}  // namespace Sim

// ---------------- Gravity ----------------
inline constexpr frc::DCMotor kGearbox = frc::DCMotor::KrakenX60FOC(2);

/**
 * Volts to hold the carriage still when the elevator is STRAIGHT UP. The pivot tilts the whole
 * elevator, so Elevator multiplies this by sin(pivot angle) every loop and sends it as a
 * feedforward (that's why Slot0 kG below is 0). Starting value is calculated from physics using
 * the sim mass; tune on the robot by finding the voltage that just holds it vertical.
 */
inline constexpr double kGVerticalVolts =
    kGearbox
        .Voltage(units::newton_meter_t{Sim::kCarriageMassKg * 9.81 * kDrumRadiusMeters *
                                       kStageMultiplier / kGearRatio},
                 0_rad_per_s)
        .value();

// ---------------- Motor config ----------------
/*
 * Gain starting points (all in meters because of SensorToMechanismRatio):
 *   kG: 0 here on purpose, see kGVerticalVolts.
 *   kS: volts to break static friction.
 *   kV: volts per (m/s). Kraken X60 is ~0.12 V per rotor rot/s, so kV ≈ 0.12 * rotorRotationsPerMeter.
 *   kP: volts per meter of error.
 */
inline const ctre::phoenix6::configs::TalonFXConfiguration kMotorConfigs =
    ctre::phoenix6::configs::TalonFXConfiguration{}
        .WithSlot0(ctre::phoenix6::configs::Slot0Configs{}
                       .WithKS(0.1)
                       .WithKG(0.0)  // gravity is sent as a feedforward, see kGVerticalVolts
                       .WithKV(0.12 * kRotorRotationsPerMeter)
                       .WithKA(0.0)
                       .WithKP(30.0)
                       .WithKI(0.0)
                       .WithKD(0.0))
        .WithMotionMagic(ctre::phoenix6::configs::MotionMagicConfigs{}
                             .WithMotionMagicCruiseVelocity(2.0_tps)        // m/s
                             .WithMotionMagicAcceleration(6.0_tr_per_s_sq)  // m/s^2
                             .WithMotionMagicJerk(0.0_tr_per_s_cu))         // 0 = no jerk limit
        .WithMotorOutput(ctre::phoenix6::configs::MotorOutputConfigs{}
                             .WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake)
                             // TODO: positive voltage must move the carriage UP. Flip if not.
                             .WithInverted(ctre::phoenix6::signals::InvertedValue::
                                               CounterClockwise_Positive))
        .WithCurrentLimits(ctre::phoenix6::configs::CurrentLimitsConfigs{}
                               .WithStatorCurrentLimitEnable(true)
                               .WithStatorCurrentLimit(80_A)
                               .WithSupplyCurrentLimitEnable(true)
                               .WithSupplyCurrentLimit(60_A)
                               .WithSupplyCurrentLowerLimit(40_A)
                               .WithSupplyCurrentLowerTime(1.0_s))
        .WithSoftwareLimitSwitch(ctre::phoenix6::configs::SoftwareLimitSwitchConfigs{}
                                     .WithForwardSoftLimitEnable(true)
                                     .WithForwardSoftLimitThreshold(
                                         units::turn_t{kMaxHeightMeters})
                                     .WithReverseSoftLimitEnable(true)
                                     .WithReverseSoftLimitThreshold(
                                         units::turn_t{kMinHeightMeters}))
        .WithFeedback(ctre::phoenix6::configs::FeedbackConfigs{}.WithSensorToMechanismRatio(
            kRotorRotationsPerMeter));

}  // namespace ElevatorConstants
