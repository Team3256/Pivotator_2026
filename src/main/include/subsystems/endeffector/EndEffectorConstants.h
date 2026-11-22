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
 * End effector on the wrist. From the CAD (ascope_assets model_3), three roller motors grip the
 * coral from the top and both sides:
 *  - TOP ROLLER: Kraken X60 -> 12T:52T gears -> 14T:22T #25 chain -> a full-width 1/2" hex shaft
 *    with nine 2" flex wheels. 6.81:1.
 *  - SIDE WHEELS: two Kraken X44s, one per side (mirror images), each 12T:40T then 14T:40T gears
 *    -> two 4" compliant wheels on that side. 9.52:1. The two sides run as leader + follower.
 *  - A CANrange that sees whether a coral is in it. NOT in the CAD: confirm it's on the robot.
 * (The fourth motor on the wrist assembly, the middle X44, is the wrist; see WristConstants.h.)
 *
 * SIGN: positive voltage pulls coral IN on every roller. Every value marked TODO must be checked
 * on the robot.
 */
namespace EndEffectorConstants {

using namespace units::literals;

inline constexpr int kTopMotorID = 50;         // TODO: real CAN IDs
inline constexpr int kCanRangeID = 51;
inline constexpr int kSideLeaderMotorID = 52;
inline constexpr int kSideFollowerMotorID = 53;

/** The side motors are mirror images driving mirrored wheels, so they spin opposite ways. VERIFY. */
inline const ctre::phoenix6::signals::MotorAlignmentValue kSideFollowerAlignment =
    ctre::phoenix6::signals::MotorAlignmentValue::Opposed;
inline constexpr std::string_view kCanBus = "";

inline constexpr bool kUseFOC = true;
inline constexpr int kFlashConfigRetries = 5;
inline constexpr units::hertz_t kUpdateFrequency = 50_Hz;

/** Motor rotations per roller rotation. */
inline constexpr double kTopGearRatio = (52.0 / 12.0) * (22.0 / 14.0);
inline constexpr double kSideGearRatio = (40.0 / 12.0) * (40.0 / 14.0);

/** Wheel radii (m): 2" flex wheels on top, 4" compliant wheels on the sides. */
inline constexpr double kTopWheelRadiusMeters = 0.0254;
inline constexpr double kSideWheelRadiusMeters = 0.0508;

/**
 * The side wheels get this fraction of the top roller's voltage so both surfaces move the coral
 * at about the same speed (a Kraken X44 spins faster than an X60, and the side wheels are bigger):
 * free surface speed ratio = (X60 free speed / top ratio * top radius) / (X44 free speed / side
 * ratio * side radius). TODO: tune on the robot (more side speed can help center the coral).
 */
/** Free speeds with FOC (we run FOC), rotor rotations per second. */
inline constexpr double kX60FreeRps = 5800.0 / 60.0;
inline constexpr double kX44FreeRps = 7368.0 / 60.0;
inline constexpr double kSideVoltsScale =
    (kX60FreeRps / kTopGearRatio * kTopWheelRadiusMeters) /
    (kX44FreeRps / kSideGearRatio * kSideWheelRadiusMeters);

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

/** Top roller (Kraken X60). */
inline const ctre::phoenix6::configs::TalonFXConfiguration kTopMotorConfigs =
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
            ctre::phoenix6::configs::FeedbackConfigs{}.WithSensorToMechanismRatio(kTopGearRatio));

/** Side wheels (both Kraken X44s; the follower copies the leader). */
inline const ctre::phoenix6::configs::TalonFXConfiguration kSideMotorConfigs =
    ctre::phoenix6::configs::TalonFXConfiguration{}
        .WithMotorOutput(ctre::phoenix6::configs::MotorOutputConfigs{}
                             .WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake)
                             // TODO: positive voltage must pull coral IN.
                             .WithInverted(ctre::phoenix6::signals::InvertedValue::
                                               CounterClockwise_Positive))
        .WithCurrentLimits(ctre::phoenix6::configs::CurrentLimitsConfigs{}
                               .WithStatorCurrentLimitEnable(true)
                               .WithStatorCurrentLimit(40_A)
                               .WithSupplyCurrentLimitEnable(true)
                               .WithSupplyCurrentLimit(25_A))
        .WithFeedback(
            ctre::phoenix6::configs::FeedbackConfigs{}.WithSensorToMechanismRatio(kSideGearRatio));

inline const ctre::phoenix6::configs::CANrangeConfiguration kCanRangeConfigs =
    ctre::phoenix6::configs::CANrangeConfiguration{}.WithProximityParams(
        ctre::phoenix6::configs::ProximityParamsConfigs{}
            .WithProximityThreshold(kCoralDetectDistance)
            .WithProximityHysteresis(0.01_m));

// ---------------- Simulation ----------------
namespace Sim {
/** Roller + coral inertia at each roller shaft (kg*m^2). */
inline constexpr double kTopRollerInertia = 0.0008;
inline constexpr double kSideRollerInertia = 0.0006;
/**
 * Wheel surface speed (m/s) that moves a coral. The coral only moves when the top roller AND the
 * side wheels push it the same way, so a dead or backwards motor shows up in sim.
 */
inline constexpr double kMoveSurfaceSpeed = 0.4;
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
