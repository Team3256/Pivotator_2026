// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "RobotContainer.h"

#include <memory>
#include <utility>

#include <frc/RobotBase.h>
#include <frc2/command/Commands.h>

#include "sim/RobotVisualizer.h"
#include "subsystems/elevator/ElevatorIOSim.h"
#include "subsystems/elevator/ElevatorIOTalonFX.h"
#include "subsystems/pivot/PivotIOSim.h"
#include "subsystems/pivot/PivotIOTalonFX.h"
#include "subsystems/wrist/WristIOSim.h"
#include "subsystems/wrist/WristIOTalonFX.h"

namespace {

// The lambdas passed in here are only called once the robot is running, after all three
// subsystems exist, so it's safe for them to point at subsystems constructed later.

std::unique_ptr<ElevatorIO> MakeElevatorIO(std::function<double()> pivotAngleRad) {
  if (frc::RobotBase::IsReal()) {
    return std::make_unique<ElevatorIOTalonFX>();
  }
  return std::make_unique<ElevatorIOSim>(std::move(pivotAngleRad));
}

std::unique_ptr<PivotIO> MakePivotIO(std::function<double()> elevatorMeters) {
  if (frc::RobotBase::IsReal()) {
    return std::make_unique<PivotIOTalonFX>();
  }
  return std::make_unique<PivotIOSim>(std::move(elevatorMeters));
}

std::unique_ptr<WristIO> MakeWristIO(std::function<double()> pivotAngleRad) {
  if (frc::RobotBase::IsReal()) {
    return std::make_unique<WristIOTalonFX>();
  }
  return std::make_unique<WristIOSim>(std::move(pivotAngleRad));
}

}  // namespace

RobotContainer::RobotContainer()
    : m_elevator{true, MakeElevatorIO([this] { return m_pivot.GetAngleRad(); })},
      m_pivot{true, MakePivotIO([this] { return m_elevator.GetPositionMeters(); })},
      m_wrist{true, MakeWristIO([this] { return m_pivot.GetAngleRad(); })} {
  // Each mechanism's gravity feedforward depends on the others.
  m_elevator.SetPivotAngleSupplier([this] { return m_pivot.GetAngleRad(); });
  m_pivot.SetElevatorExtensionSupplier([this] { return m_elevator.GetPositionMeters(); });
  m_wrist.SetPivotAngleSupplier([this] { return m_pivot.GetAngleRad(); });

  ConfigureTestBindings();
}

/**
 * Temporary test buttons for driving the elevator. In the sim GUI, drag "Keyboard 0" onto
 * Joysticks[0]; then Z/X/C/V are buttons 1-4 = A/B/X/Y.
 */
void RobotContainer::ConfigureTestBindings() {
  m_testController.A().OnTrue(m_elevator.GoToStow());
  m_testController.B().OnTrue(m_elevator.GoToL2());
  m_testController.X().OnTrue(m_elevator.GoToL3());
  m_testController.Y().OnTrue(m_elevator.GoToL4());
}

void RobotContainer::Periodic() {
  RobotVisualizer::Update(m_pivot.GetAngleRad(), m_elevator.GetPositionMeters(),
                          m_wrist.GetAngleRad());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::None();
}
