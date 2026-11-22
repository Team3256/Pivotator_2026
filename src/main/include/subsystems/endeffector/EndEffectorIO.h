// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

/** One roller motor's readings. Velocity is roller (not rotor) rotations per second. */
struct RollerMotorInputs {
  bool connected = false;
  double velocityRps = 0.0;
  double voltage = 0.0;
  double statorCurrent = 0.0;
  double supplyCurrent = 0.0;
  double tempCelsius = 0.0;
};

/** Everything the end effector reads from hardware each loop. */
struct EndEffectorIOInputs {
  RollerMotorInputs top;
  RollerMotorInputs sideLeader;
  RollerMotorInputs sideFollower;

  bool sensorConnected = false;
  double sensorDistanceMeters = 0.0;
  /** The CANrange sees a coral in the end effector. */
  bool hasCoral = false;
};

/** Hardware abstraction for the end effector rollers and coral sensor. */
class EndEffectorIO {
 public:
  virtual ~EndEffectorIO() = default;

  virtual void UpdateInputs(EndEffectorIOInputs& inputs) {}
  /** Top roller and side wheels. Positive pulls coral in. */
  virtual void SetVoltage(double topVolts, double sideVolts) {}
  virtual void Off() {}
};
