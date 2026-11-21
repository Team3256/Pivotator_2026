// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <string_view>

#include <ctre/phoenix6/CANrange.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <units/current.h>
#include <units/frequency.h>
#include <units/length.h>
#include <units/time.h>

/**
 * End effector on the wrist: one roller that takes coral in and pushes it out onto the reef, and
 * a CANrange that sees whether a coral is in it.
 *
 * SIGN: positive voltage pulls coral IN. Every value marked TODO must be checked on the robot
 * (which motor drives the rollers vs the wrist is still to be confirmed from the "03. Wrist" BOM).
 */
namespace EndEffectorConstants {

using namespace units::literals;

inline constexpr int kMotorID = 50;    // TODO: real CAN ID
inline constexpr int kCanRangeID = 51;  // TODO: real CAN ID
inline constexpr std::string_view kCanBus = "";

inline constexpr bool kUseFOC = true;
inline constexpr int kFlashConfigRetries = 5;
inline constexpr units::hertz_t kUpdateFrequency = 50_Hz;

/** Motor rotations per roller rotation. TODO: measure. */
inline constexpr double kGearRatio = 3.0;

// ---------------- Roller voltages ----------------
inline constexpr double kIntakeVolts = 8.0;
/** Small inward push so a held coral doesn't slide out while the arm swings. */
inline constexpr double kHoldVolts = 1.0;
/** Pushes the coral out onto the reef. TODO: direction depends on how the end effector scores. */
inline constexpr double kScoreVolts = -9.0;
inline constexpr double kEjectVolts = -6.0;

/** Keep scoring this long after the sensor clears, so the coral is fully out. */
inline constexpr units::second_t kScoreClearTime = 0.25_s;

/** The CANrange sees a coral closer than this. TODO: tune with a coral in the end effector. */
inline constexpr units::meter_t kCoralDetectDistance = 0.08_m;

inline const ctre::phoenix6::configs::TalonFXConfiguration kMotorConfigs =
    ctre::phoenix6::configs::TalonFXConfiguration{}
        .WithMotorOutput(ctre::phoenix6::configs::MotorOutputConfigs{}
                             .WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake)
                             // TODO: positive voltage must pull coral IN.
                             .WithInverted(ctre::phoenix6::signals::InvertedValue::
                                               CounterClockwise_Positive))
        .WithCurrentLimits(ctre::phoenix6::configs::CurrentLimitsConfigs{}
                               .WithStatorCurrentLimitEnable(true)
                               .WithStatorCurrentLimit(50_A)
                               .WithSupplyCurrentLimitEnable(true)
                               .WithSupplyCurrentLimit(30_A))
        .WithFeedback(
            ctre::phoenix6::configs::FeedbackConfigs{}.WithSensorToMechanismRatio(kGearRatio));

inline const ctre::phoenix6::configs::CANrangeConfiguration kCanRangeConfigs =
    ctre::phoenix6::configs::CANrangeConfiguration{}.WithProximityParams(
        ctre::phoenix6::configs::ProximityParamsConfigs{}
            .WithProximityThreshold(kCoralDetectDistance)
            .WithProximityHysteresis(0.01_m));

// ---------------- Simulation ----------------
namespace Sim {
/** Roller + coral inertia at the roller (kg*m^2). */
inline constexpr double kRollerInertia = 0.0008;
/** Roller surface speed (rotations per second) that moves a coral. */
inline constexpr double kMoveSpeedRps = 3.0;
/** How long the roller must run to pull a coral in or push it out (seconds). */
inline constexpr double kTransferSeconds = 0.25;
/** The robot starts a match holding one coral. */
inline constexpr bool kPreload = true;
/**
 * Where the coral station feeds a coral (the robot isn't driving yet, so "at the station" means
 * the CORAL_INTAKE pose): pivot, elevator and wrist within these of that pose.
 */
inline constexpr double kIntakePivotDeg = 60.0, kIntakeWristDeg = -45.0;
inline constexpr double kIntakeToleranceDeg = 4.0, kIntakeMaxElevatorMeters = 0.05;
}  // namespace Sim

}  // namespace EndEffectorConstants
