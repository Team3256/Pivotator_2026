// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

/** Everything the elevator reads from hardware each loop. Positions in meters, velocities m/s. */
struct ElevatorIOInputs {
  bool leaderConnected = false;
  double leaderPositionMeters = 0.0;
  double leaderVelocityMetersPerSec = 0.0;
  double leaderVoltage = 0.0;
  double leaderStatorCurrent = 0.0;
  double leaderSupplyCurrent = 0.0;
  double leaderTempCelsius = 0.0;

  bool followerConnected = false;
  double followerPositionMeters = 0.0;
  double followerVelocityMetersPerSec = 0.0;
  double followerVoltage = 0.0;
  double followerStatorCurrent = 0.0;
  double followerSupplyCurrent = 0.0;
  double followerTempCelsius = 0.0;

  double closedLoopReferenceMeters = 0.0;
  double closedLoopErrorMeters = 0.0;
};

/** Hardware abstraction for the elevator. All positions in meters, velocities in m/s. */
class ElevatorIO {
 public:
  virtual ~ElevatorIO() = default;

  virtual void UpdateInputs(ElevatorIOInputs& inputs) {}

  /**
   * Closed-loop (Motion Magic or plain position) to a height in meters.
   *
   * @param gravityFeedforwardVolts extra volts to hold the carriage against gravity. Computed by
   *     Elevator from the pivot angle, because a tilted elevator feels less gravity.
   */
  virtual void SetPosition(double meters, double gravityFeedforwardVolts) {}

  /** Open-loop voltage. Soft limits still apply. */
  virtual void SetVoltage(double volts) {}

  /** Open-loop voltage that ignores soft limits. Only for homing. */
  virtual void SetVoltageIgnoringLimits(double volts) {}

  virtual void Off() {}

  /** Tell the encoders the carriage is currently at this height. */
  virtual void ResetPosition(double meters) {}
};
