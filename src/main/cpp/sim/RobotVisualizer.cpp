// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "sim/RobotVisualizer.h"

#include <array>

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation3d.h>
#include <units/angle.h>
#include <units/length.h>

#include "util/Log.h"

using namespace units::literals;

namespace {

// ---------------- Geometry from CAD (robot frame, meters) ----------------
/** Pivot axle: runs left-right (robot Y), through the 64T pivot sprockets / X-contact bearings. */
const frc::Translation3d kPivotAxle{-0.24_m, 0.0_m, 0.34_m};

/** Direction the carriage slides when the pivot is at its CAD angle (flat, pointing forward). */
const frc::Translation3d kElevatorDirection{1.0_m, 0.0_m, 0.0_m};

/** Wrist axle (Thrifty Cycloidal output) when the elevator is flat and fully retracted. */
const frc::Translation3d kWristAxle{0.395_m, 0.0_m, 0.20_m};

const std::array<frc::Pose3d, RobotVisualizer::kNumComponents> kZeroPoses{};

/**
 * There's no drivetrain code yet, so park the robot at a fixed spot on the field. AdvantageScope
 * needs a robot pose to hang the components on. Replace with the real odometry pose later.
 */
const frc::Pose3d kParkedRobotPose{2.0_m, 4.0_m, 0.0_m, frc::Rotation3d{}};

/** Apply `inner` first, then `outer`. */
frc::Pose3d Compose(const frc::Pose3d& outer, const frc::Pose3d& inner) {
  return outer.TransformBy(frc::Transform3d{inner.Translation(), inner.Rotation()});
}

/** A pose that rotates things by `rot` about an axle passing through `point`. */
frc::Pose3d RotateAbout(const frc::Translation3d& point, const frc::Rotation3d& rot) {
  // Move the axle to the origin, rotate, move it back: x -> R(x - p) + p = Rx + (p - Rp).
  return frc::Pose3d{point - point.RotateBy(rot), rot};
}

/** `base`, then slid `meters` along the elevator (in the elevator's tilted frame). */
frc::Pose3d Slide(const frc::Pose3d& base, double meters) {
  return Compose(base, frc::Pose3d{kElevatorDirection * meters, frc::Rotation3d{}});
}

/** Tilting UP about the left-pointing Y axis is a NEGATIVE pitch (right-hand rule). */
frc::Rotation3d PitchUp(double radians) {
  return frc::Rotation3d{0_rad, units::radian_t{-radians}, 0_rad};
}

}  // namespace

void RobotVisualizer::Update(double elevatorMeters) {
  Update(kPivotAngleAtCadRad, elevatorMeters, kWristAngleAtCadRad);
}

void RobotVisualizer::Update(double pivotRad, double elevatorMeters, double wristRad) {
  frc::Pose3d pivot = RotateAbout(kPivotAxle, PitchUp(pivotRad - kPivotAngleAtCadRad));
  frc::Pose3d stage1 = Slide(pivot, elevatorMeters / 2.0);
  frc::Pose3d carriage = Slide(pivot, elevatorMeters);
  frc::Pose3d wrist =
      Compose(carriage, RotateAbout(kWristAxle, PitchUp(wristRad - kWristAngleAtCadRad)));

  std::array<frc::Pose3d, kNumComponents> poses;
  poses[kFixedStageIndex] = pivot;
  poses[kStage1Index] = stage1;
  poses[kCarriageIndex] = carriage;
  poses[kWristIndex] = wrist;
  Log::Record("RobotVisualizer/ComponentPoses", poses);
  Log::Record("RobotVisualizer/RobotPose", kParkedRobotPose);

  // All-zero poses: drag this onto the robot instead when lining up config.json. Every part
  // should then sit exactly where it does in the CAD export.
  Log::Record("RobotVisualizer/ZeroedComponentPoses", kZeroPoses);
}
