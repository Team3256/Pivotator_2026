// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

/** Everything the end effector reads from hardware each loop. */
struct EndEffectorIOInputs {
  bool motorConnected = false;
  double velocityRps = 0.0;
  double voltage = 0.0;
  double statorCurrent = 0.0;
  double supplyCurrent = 0.0;
  double tempCelsius = 0.0;

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
  /** Positive pulls coral in. */
  virtual void SetVoltage(double volts) {}
  virtual void Off() {}
};
