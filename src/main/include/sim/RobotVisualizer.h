// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

/**
 * Logs 3D poses of the robot's moving parts so AdvantageScope can animate our CAD model.
 *
 * How it works: in AdvantageScope's custom robot config.json, each moving part ("component") is
 * its own .glb file, listed in a fixed order. Every loop we log a Pose3d[] in that SAME order.
 * Each pose says how that part has moved from where it sits in the CAD export. A pose of all
 * zeros means "exactly where it is in CAD".
 *
 * The CAD export ("00. Main Unconstrained") has the elevator lying FLAT, pointing forward, fully
 * retracted, with the wrist at its CAD angle. All numbers are in robot coordinates (X forward,
 * Y left, Z up, origin at frame center on the floor), measured from that export.
 *
 * This is a pivotvator, so each part stacks on the one before it:
 *   1. Fixed elevator stage: rotates with the pivot about the pivot axle.
 *   2. 1st stage: fixed stage, then slides h/2 along the elevator (continuous rigging).
 *   3. Carriage: fixed stage, then slides h along the elevator.
 *   4. Wrist + end effector: carriage, then rotates about the wrist axle.
 *
 * Runs on the real robot too, so log files from matches can be watched the same way.
 */
namespace RobotVisualizer {

/** Index of each component in the Pose3d[]. MUST match the "components" order in config.json. */
inline constexpr int kFixedStageIndex = 0;
inline constexpr int kStage1Index = 1;
inline constexpr int kCarriageIndex = 2;
inline constexpr int kWristIndex = 3;
inline constexpr int kNumComponents = 4;

/**
 * Pivot angle (as the pivot code reports it) at which the CAD was drawn. The CAD has the elevator
 * flat, so if the pivot code calls "flat" 0 rad, this stays 0. If the code calls "straight up" 0,
 * set this to -pi / 2.
 */
inline constexpr double kPivotAngleAtCadRad = 0.0;

/** Wrist angle (as the wrist code reports it) at which the CAD was drawn. TODO: confirm. */
inline constexpr double kWristAngleAtCadRad = 0.0;

/** Elevator only (pivot and wrist held at their CAD angles). */
void Update(double elevatorMeters);

/**
 * @param pivotRad pivot angle; positive tilts the elevator UP from flat
 * @param elevatorMeters carriage travel from fully retracted (Elevator::GetPositionMeters)
 * @param wristRad wrist angle; positive tips the end effector UP (same sense as the pivot)
 */
void Update(double pivotRad, double elevatorMeters, double wristRad);

}  // namespace RobotVisualizer
