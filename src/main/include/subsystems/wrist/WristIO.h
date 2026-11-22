// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

/** Everything the wrist reads from hardware each loop. Angles in degrees relative to elevator. */
struct WristIOInputs {
  bool motorConnected = false;
  double positionDeg = 0.0;
  double velocityDegPerSec = 0.0;
  double voltage = 0.0;
  double statorCurrent = 0.0;
  double tempCelsius = 0.0;
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
