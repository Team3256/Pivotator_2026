// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <array>

/** Everything the pivot reads from hardware each loop. Angles in degrees above horizontal. */
struct PivotIOInputs {
  bool leaderConnected = false;
  bool followersConnected = false;
  double positionDeg = 0.0;
  double velocityDegPerSec = 0.0;
  double leaderVoltage = 0.0;
  /** Stator current of all 4 motors: [leader, same side, other side, other side follower]. */
  std::array<double, 4> statorCurrents{};
  std::array<double, 4> supplyCurrents{};
  std::array<double, 4> tempsCelsius{};
  double closedLoopErrorDeg = 0.0;
};

/** Hardware abstraction for the main pivot. Angles in degrees above horizontal. */
class PivotIO {
 public:
  virtual ~PivotIO() = default;

  virtual void UpdateInputs(PivotIOInputs& inputs) {}

  /** Motion Magic to an angle, plus a gravity feedforward computed by the Pivot subsystem. */
  virtual void SetPosition(double degrees, double gravityFeedforwardVolts) {}

  virtual void SetVoltage(double volts) {}

  virtual void Off() {}

  virtual void ResetPosition(double degrees) {}
};
