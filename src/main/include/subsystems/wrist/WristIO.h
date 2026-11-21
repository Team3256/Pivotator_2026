// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

/** Everything the wrist reads from hardware each loop. Angles in degrees relative to elevator. */
struct WristIOInputs {
  bool leaderConnected = false;
  bool followerConnected = false;
  double positionDeg = 0.0;
  double velocityDegPerSec = 0.0;
  double leaderVoltage = 0.0;
  double leaderStatorCurrent = 0.0;
  double followerStatorCurrent = 0.0;
  double leaderTempCelsius = 0.0;
  double followerTempCelsius = 0.0;
};

/** Hardware abstraction for the wrist. Angles in degrees relative to the elevator. */
class WristIO {
 public:
  virtual ~WristIO() = default;

  virtual void UpdateInputs(WristIOInputs& inputs) {}
  virtual void SetPosition(double degrees, double gravityFeedforwardVolts) {}
  virtual void SetVoltage(double volts) {}
  virtual void Off() {}
  virtual void ResetPosition(double degrees) {}
};
