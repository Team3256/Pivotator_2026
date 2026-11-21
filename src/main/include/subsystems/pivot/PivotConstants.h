// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <string_view>

#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/system/plant/DCMotor.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/frequency.h>
#include <units/time.h>
#include <units/torque.h>

#include "subsystems/elevator/ElevatorConstants.h"

/**
 * The main pivot: tilts the WHOLE elevator at its base.
 *
 * ANGLE CONVENTION: degrees above horizontal. 0 = elevator lying flat pointing forward, 90 =
 * elevator straight up. Phoenix works in "mechanism rotations" (1 rotation = 360 deg) because of
 * SensorToMechanismRatio, and the subsystem converts to degrees for you.
 *
 * Hardware from Onshape "01. Pivot" BOM: 4x Kraken X60 (2 per side). Per side: 12T pinion ->
 * 64T gear, 16T -> 62T gear, then 12T #35 sprocket -> 64T sprocket on the pivot.
 */
namespace PivotConstants {

using namespace units::literals;

// ---------------- Hardware ----------------
inline constexpr int kLeaderMotorID = 30;  // TODO: real CAN IDs
inline constexpr int kSameSideFollowerID = 31;
inline constexpr int kOtherSideLeaderID = 32;
inline constexpr int kOtherSideFollowerID = 33;
inline constexpr std::string_view kCanBus = "";

/** Both motors on one side mesh the same 64T gear, so they spin the same way. VERIFY. */
inline const ctre::phoenix6::signals::MotorAlignmentValue kSameSideAlignment =
    ctre::phoenix6::signals::MotorAlignmentValue::Aligned;

/** The two sides are mirror images, so the other side spins the opposite way. VERIFY. */
inline const ctre::phoenix6::signals::MotorAlignmentValue kOtherSideAlignment =
    ctre::phoenix6::signals::MotorAlignmentValue::Opposed;

inline constexpr bool kUseFOC = true;
inline constexpr int kFlashConfigRetries = 5;
inline constexpr units::hertz_t kUpdateFrequency = 50_Hz;

/** 64/12 * 62/16 * 64/12 = ~110.2 motor rotations per pivot rotation. */
inline constexpr double kGearRatio = (64.0 / 12.0) * (62.0 / 16.0) * (64.0 / 12.0);

// ---------------- Range ----------------
inline constexpr double kMinAngleDeg = 0.0;   // TODO: front hard stop
inline constexpr double kMaxAngleDeg = 95.0;  // TODO: back hard stop

/**
 * There is no absolute encoder (no CANcoder in the BOM), so the code assumes the pivot is resting
 * at this angle when the robot turns on. ALWAYS power on with the pivot in this position.
 * TODO: confirm which hard stop the robot rests on, or add a CANcoder/through-bore.
 */
inline constexpr double kBootAngleDeg = 90.0;

inline constexpr double kToleranceDeg = 1.5;

// ---------------- Gravity / mass model ----------------
/*
 * The pivot holds up the whole elevator, and the further out the carriage is, the harder gravity
 * pulls. We model three point masses measured from the pivot axle along the elevator:
 *   fixed stage (doesn't move), 1st stage (moves h/2), carriage+wrist+coral (moves h).
 * All TODO: get real numbers from CAD mass properties.
 */
inline constexpr double kFixedStageMassKg = 5.0;
inline constexpr double kFixedStageCoMMeters = 0.45;
inline constexpr double kStage1MassKg = 2.0;
inline constexpr double kStage1CoMRetractedMeters = 0.50;
inline constexpr double kCarriageMassKg = ElevatorConstants::Sim::kCarriageMassKg;
inline constexpr double kCarriageCoMRetractedMeters = 0.55;

/** Gravity torque on the pivot (N*m) when horizontal, at a given elevator extension. */
constexpr double GravityTorqueHorizontalNm(double elevatorMeters) {
  return 9.81 * (kFixedStageMassKg * kFixedStageCoMMeters +
                 kStage1MassKg * (kStage1CoMRetractedMeters + elevatorMeters / 2.0) +
                 kCarriageMassKg * (kCarriageCoMRetractedMeters + elevatorMeters));
}

/** Moment of inertia (kg*m^2) about the pivot axle, using the same point masses. */
constexpr double MomentOfInertia(double elevatorMeters) {
  double s1 = kStage1CoMRetractedMeters + elevatorMeters / 2.0;
  double c = kCarriageCoMRetractedMeters + elevatorMeters;
  return kFixedStageMassKg * kFixedStageCoMMeters * kFixedStageCoMMeters +
         kStage1MassKg * s1 * s1 + kCarriageMassKg * c * c;
}

inline constexpr frc::DCMotor kGearbox = frc::DCMotor::KrakenX60FOC(4);

/**
 * Volts to hold the pivot horizontal at a given elevator extension. The Pivot multiplies this by
 * cos(angle) each loop and sends it as a feedforward. Calculated from the mass model above.
 */
constexpr double KGVolts(double elevatorMeters) {
  return kGearbox
      .Voltage(units::newton_meter_t{GravityTorqueHorizontalNm(elevatorMeters) / kGearRatio},
               0_rad_per_s)
      .value();
}

// ---------------- Motor config ----------------
/*
 * Units are pivot ROTATIONS (1.0 = 360 deg) because of SensorToMechanismRatio.
 * kV: Kraken ~0.12 V per rotor rot/s * gearRatio.  kP: volts per rotation of error
 * (60 V/rot = 0.17 V per degree).
 */
inline const ctre::phoenix6::configs::TalonFXConfiguration kMotorConfigs =
    ctre::phoenix6::configs::TalonFXConfiguration{}
        .WithSlot0(ctre::phoenix6::configs::Slot0Configs{}
                       .WithKS(0.15)
                       .WithKG(0.0)  // gravity sent as a feedforward, see KGVolts()
                       .WithKV(0.12 * kGearRatio)
                       .WithKA(0.0)
                       .WithKP(60.0)
                       .WithKI(0.0)
                       .WithKD(0.0))
        .WithMotionMagic(ctre::phoenix6::configs::MotionMagicConfigs{}
                             .WithMotionMagicCruiseVelocity(0.35_tps)  // pivot rot/s (~125 deg/s)
                             .WithMotionMagicAcceleration(1.0_tr_per_s_sq))  // pivot rot/s^2
        .WithMotorOutput(ctre::phoenix6::configs::MotorOutputConfigs{}
                             .WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake)
                             // TODO: positive voltage must tilt the elevator BACK toward vertical
                             .WithInverted(ctre::phoenix6::signals::InvertedValue::
                                               CounterClockwise_Positive))
        .WithCurrentLimits(ctre::phoenix6::configs::CurrentLimitsConfigs{}
                               .WithStatorCurrentLimitEnable(true)
                               .WithStatorCurrentLimit(80_A)
                               .WithSupplyCurrentLimitEnable(true)
                               .WithSupplyCurrentLimit(50_A)
                               .WithSupplyCurrentLowerLimit(35_A)
                               .WithSupplyCurrentLowerTime(1.0_s))
        .WithSoftwareLimitSwitch(ctre::phoenix6::configs::SoftwareLimitSwitchConfigs{}
                                     .WithForwardSoftLimitEnable(true)
                                     .WithForwardSoftLimitThreshold(
                                         units::turn_t{kMaxAngleDeg / 360.0})
                                     .WithReverseSoftLimitEnable(true)
                                     .WithReverseSoftLimitThreshold(
                                         units::turn_t{kMinAngleDeg / 360.0}))
        .WithFeedback(
            ctre::phoenix6::configs::FeedbackConfigs{}.WithSensorToMechanismRatio(kGearRatio));

}  // namespace PivotConstants
