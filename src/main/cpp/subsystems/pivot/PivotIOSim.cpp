// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "subsystems/pivot/PivotIOSim.h"

#include <cmath>
#include <numbers>
#include <utility>

#include <frc/RobotController.h>
#include <frc/TimedRobot.h>
#include <units/angle.h>

#include "sim/SimBattery.h"
#include "subsystems/pivot/PivotConstants.h"
#include "util/Log.h"

using namespace ctre::phoenix6;
using namespace units::literals;
namespace PC = PivotConstants;

namespace {
sim::ChassisReference OrientationFor(signals::MotorAlignmentValue alignment) {
  return alignment == signals::MotorAlignmentValue::Aligned
             ? sim::ChassisReference::CounterClockwise_Positive
             : sim::ChassisReference::Clockwise_Positive;
}
}  // namespace

PivotIOSim::PivotIOSim(std::function<double()> elevatorMeters)
    : PivotIOTalonFX(),
      m_elevatorMeters{std::move(elevatorMeters)},
      m_physics{PC::kGearbox,
                PC::kGearRatio,
                // MOI is fixed in WPILib's model; use the half-extended value as a middle ground.
                units::kilogram_square_meter_t{PC::MomentOfInertia(0.45)},
                units::meter_t{PC::kCarriageCoMRetractedMeters},
                units::degree_t{PC::kMinAngleDeg},
                units::degree_t{PC::kMaxAngleDeg},
                false,  // gravity handled below, it depends on elevator extension
                units::degree_t{PC::kBootAngleDeg}} {
  for (int i = 0; i < 4; i++) {
    m_simStates[i] = &m_motors[i]->GetSimState();
  }
  // Must match MotorOutput.Inverted (CCW+) and each follower's alignment.
  m_simStates[0]->Orientation = sim::ChassisReference::CounterClockwise_Positive;
  m_simStates[1]->Orientation = OrientationFor(PC::kSameSideAlignment);
  m_simStates[2]->Orientation = OrientationFor(PC::kOtherSideAlignment);
  m_simStates[3]->Orientation = OrientationFor(PC::kOtherSideAlignment);
}

void PivotIOSim::UpdateInputs(PivotIOInputs& inputs) {
  units::volt_t battery = frc::RobotController::GetBatteryVoltage();
  for (sim::TalonFXSimState* s : m_simStates) {
    s->SetSupplyVoltage(battery);
  }

  double angle = m_physics.GetAngle().value();
  double gravityTorque = PC::GravityTorqueHorizontalNm(m_elevatorMeters()) * std::cos(angle);
  units::volt_t gravityVolts =
      PC::kGearbox.Voltage(units::newton_meter_t{gravityTorque / PC::kGearRatio}, 0_rad_per_s);

  m_physics.SetInputVoltage(m_simStates[0]->GetMotorVoltage() - gravityVolts);
  m_physics.Update(frc::TimedRobot::kDefaultPeriod);

  // Raw rotor position is measured from boot, because the Talon was told "you're at bootAngle"
  // at startup (ResetPosition). Feeding the absolute angle would double-count it.
  double bootRad = units::radian_t{units::degree_t{PC::kBootAngleDeg}}.value();
  units::turn_t rotorRot{(m_physics.GetAngle().value() - bootRad) / (2 * std::numbers::pi) *
                         PC::kGearRatio};
  units::turns_per_second_t rotorVel{m_physics.GetVelocity().value() / (2 * std::numbers::pi) *
                                     PC::kGearRatio};
  for (sim::TalonFXSimState* s : m_simStates) {
    s->SetRawRotorPosition(rotorRot);
    s->SetRotorVelocity(rotorVel);
  }

  SimBattery::Report("Pivot", m_physics.GetCurrentDraw().value());
  PivotIOTalonFX::UpdateInputs(inputs);

  Log::Record("PivotSim/AngleDeg", units::degree_t{m_physics.GetAngle()}.value());
  Log::Record("PivotSim/CurrentDrawAmps", m_physics.GetCurrentDraw().value());
}
