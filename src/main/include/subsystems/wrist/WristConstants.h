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
#include <units/torque.h>

/**
 * Wrist on the elevator carriage.
 *
 * ANGLE CONVENTION: degrees RELATIVE TO THE ELEVATOR. 0 = end effector pointing straight out
 * along the elevator (continuing it), positive = tipping the same way the pivot tips up. So the
 * wrist's angle from horizontal is (pivot angle + wrist angle).
 *
 * Hardware from the CAD (ascope_assets model_3): ONE Kraken X44, low in the middle of the end
 * effector. 12T HTD5 pulley -> belt -> 36T pulley -> one 15T spline shaft through BOTH Thrifty
 * Cycloidals (one per side, same input). The other two X44s and the X60 drive the end effector's
 * rollers (see EndEffectorConstants.h).
 */
namespace WristConstants {

using namespace units::literals;

inline constexpr int kMotorID = 40;  // TODO: real CAN ID
inline constexpr std::string_view kCanBus = "";

inline constexpr bool kUseFOC = true;
inline constexpr int kFlashConfigRetries = 5;
inline constexpr units::hertz_t kUpdateFrequency = 50_Hz;

/** 12T -> 36T belt stage before the cycloidals. */
inline constexpr double kBeltRatio = 36.0 / 12.0;
/** Thrifty Cycloidal Gearbox (ThriftyBot lists it as 23:1). */
inline constexpr double kCycloidalRatio = 23.0;
/** Motor rotations per wrist rotation: 3 * 23 = 69:1. */
inline constexpr double kGearRatio = kBeltRatio * kCycloidalRatio;

inline constexpr double kMinAngleDeg = -100.0;  // TODO: hard stops
inline constexpr double kMaxAngleDeg = 135.0;

/** No absolute encoder in the BOM, so the wrist must be at this angle at power-on. TODO. */
inline constexpr double kBootAngleDeg = 0.0;

inline constexpr double kToleranceDeg = 2.0;

// ---------------- Gravity ----------------
inline constexpr double kMassKg = 3.0;      // wrist + end effector + coral. TODO: CAD
inline constexpr double kComMeters = 0.18;  // axle to center of mass. TODO: CAD
inline constexpr frc::DCMotor kGearbox = frc::DCMotor::KrakenX44FOC(1);

/** Volts to hold the wrist horizontal. Wrist multiplies by cos(pivot + wrist). */
inline constexpr double kGVolts =
    kGearbox
        .Voltage(units::newton_meter_t{kMassKg * 9.81 * kComMeters / kGearRatio}, 0_rad_per_s)
        .value();

/** Moment of inertia for the sim (point mass). */
inline constexpr double kMomentOfInertia = kMassKg * kComMeters * kComMeters;

// ---------------- Motor config ----------------
// Units are wrist ROTATIONS. Kraken X44 is ~0.096 V per rotor rot/s.
inline const ctre::phoenix6::configs::TalonFXConfiguration kMotorConfigs =
    ctre::phoenix6::configs::TalonFXConfiguration{}
        .WithSlot0(ctre::phoenix6::configs::Slot0Configs{}
                       .WithKS(0.1)
                       .WithKG(0.0)  // gravity sent as feedforward, see kGVolts
                       .WithKV(0.096 * kGearRatio)
                       .WithKA(0.0)
                       .WithKP(40.0)
                       .WithKI(0.0)
                       .WithKD(0.0))
        .WithMotionMagic(ctre::phoenix6::configs::MotionMagicConfigs{}
                             // wrist rot/s. A Kraken X44 at 69:1 tops out near 1.8.
                             .WithMotionMagicCruiseVelocity(1.2_tps)
                             .WithMotionMagicAcceleration(6.0_tr_per_s_sq))
        .WithMotorOutput(ctre::phoenix6::configs::MotorOutputConfigs{}
                             .WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake)
                             // TODO: verify direction
                             .WithInverted(ctre::phoenix6::signals::InvertedValue::
                                               CounterClockwise_Positive))
        .WithCurrentLimits(ctre::phoenix6::configs::CurrentLimitsConfigs{}
                               .WithStatorCurrentLimitEnable(true)
                               .WithStatorCurrentLimit(60_A)
                               .WithSupplyCurrentLimitEnable(true)
                               .WithSupplyCurrentLimit(40_A))
        .WithSoftwareLimitSwitch(ctre::phoenix6::configs::SoftwareLimitSwitchConfigs{}
                                     .WithForwardSoftLimitEnable(true)
                                     .WithForwardSoftLimitThreshold(
                                         units::turn_t{kMaxAngleDeg / 360.0})
                                     .WithReverseSoftLimitEnable(true)
                                     .WithReverseSoftLimitThreshold(
                                         units::turn_t{kMinAngleDeg / 360.0}))
        .WithFeedback(
            ctre::phoenix6::configs::FeedbackConfigs{}.WithSensorToMechanismRatio(kGearRatio));

}  // namespace WristConstants
