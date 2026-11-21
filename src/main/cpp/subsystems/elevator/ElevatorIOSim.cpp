// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "subsystems/elevator/ElevatorIOSim.h"

#include <cmath>
#include <utility>

#include <frc/RobotController.h>
#include <frc/TimedRobot.h>

#include "sim/SimBattery.h"
#include "subsystems/elevator/ElevatorConstants.h"
#include "util/Log.h"

using namespace ctre::phoenix6;
using namespace units::literals;
namespace EC = ElevatorConstants;

ElevatorIOSim::ElevatorIOSim(std::function<double()> pivotAngleRad)
    : ElevatorIOTalonFX(),
      m_leaderSim{m_leader.GetSimState()},
      m_followerSim{m_follower.GetSimState()},
      m_pivotAngleRad{std::move(pivotAngleRad)},
      m_physics{EC::kGearbox,
                EC::kGearRatio,
                units::kilogram_t{EC::Sim::kCarriageMassKg},
                units::meter_t{EC::kDrumRadiusMeters * EC::kStageMultiplier},
                units::meter_t{EC::kMinHeightMeters},
                units::meter_t{EC::kMaxHeightMeters},
                false,  // gravity handled below, it depends on the pivot angle
                units::meter_t{EC::Sim::kStartingHeightMeters}} {
  // Must match MotorOutput.Inverted in ElevatorConstants so +V moves the sim carriage up.
  m_leaderSim.Orientation = sim::ChassisReference::CounterClockwise_Positive;
  m_followerSim.Orientation = EC::kFollowerAlignment == signals::MotorAlignmentValue::Aligned
                                  ? sim::ChassisReference::CounterClockwise_Positive
                                  : sim::ChassisReference::Clockwise_Positive;
}

void ElevatorIOSim::UpdateInputs(ElevatorIOInputs& inputs) {
  units::volt_t battery = frc::RobotController::GetBatteryVoltage();
  m_leaderSim.SetSupplyVoltage(battery);
  m_followerSim.SetSupplyVoltage(battery);

  // Gravity pulls along the elevator by g * sin(pivot angle).
  double gravityForceN = EC::Sim::kCarriageMassKg * 9.81 * std::sin(m_pivotAngleRad());
  double gravityMotorTorque =
      gravityForceN * EC::kDrumRadiusMeters * EC::kStageMultiplier / EC::kGearRatio;
  units::volt_t gravityVolts =
      EC::kGearbox.Voltage(units::newton_meter_t{gravityMotorTorque}, 0_rad_per_s);

  m_physics.SetInputVoltage(m_leaderSim.GetMotorVoltage() - gravityVolts);
  m_physics.Update(frc::TimedRobot::kDefaultPeriod);

  double positionMeters = m_physics.GetPosition().value();
  double velocityMetersPerSec = m_physics.GetVelocity().value();
  units::turn_t rotorPos{positionMeters * EC::kRotorRotationsPerMeter};
  units::turns_per_second_t rotorVel{velocityMetersPerSec * EC::kRotorRotationsPerMeter};
  m_leaderSim.SetRawRotorPosition(rotorPos);
  m_leaderSim.SetRotorVelocity(rotorVel);
  m_followerSim.SetRawRotorPosition(rotorPos);
  m_followerSim.SetRotorVelocity(rotorVel);

  SimBattery::Report("Elevator", m_physics.GetCurrentDraw().value());

  ElevatorIOTalonFX::UpdateInputs(inputs);

  Log::Record("ElevatorSim/PositionMeters", positionMeters);
  Log::Record("ElevatorSim/VelocityMetersPerSec", velocityMetersPerSec);
  Log::Record("ElevatorSim/CurrentDrawAmps", m_physics.GetCurrentDraw().value());
}
