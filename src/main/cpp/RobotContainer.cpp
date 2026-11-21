// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "RobotContainer.h"

#include <memory>

#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/event/EventLoop.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>

#include "sim/PhysicsWorld.h"
#include "sim/RobotVisualizer.h"
#include "subsystems/elevator/ElevatorIOSim.h"
#include "subsystems/elevator/ElevatorIOTalonFX.h"
#include "subsystems/pivot/PivotIOSim.h"
#include "subsystems/pivot/PivotIOTalonFX.h"
#include "subsystems/wrist/WristIOSim.h"
#include "subsystems/wrist/WristIOTalonFX.h"

namespace {

template <typename Real, typename Sim, typename IO>
std::unique_ptr<IO> MakeIO() {
  if (frc::RobotBase::IsReal()) {
    return std::make_unique<Real>();
  }
  return std::make_unique<Sim>();
}

}  // namespace

RobotContainer::RobotContainer()
    : m_elevator{true, MakeIO<ElevatorIOTalonFX, ElevatorIOSim, ElevatorIO>()},
      m_pivot{true, MakeIO<PivotIOTalonFX, PivotIOSim, PivotIO>()},
      m_wrist{true, MakeIO<WristIOTalonFX, WristIOSim, WristIO>()},
      m_superstructure{m_elevator, m_pivot, m_wrist},
      m_autos{m_superstructure} {
  // Each mechanism's gravity feedforward depends on the others.
  m_elevator.SetPivotAngleSupplier([this] { return m_pivot.GetAngleRad(); });
  m_pivot.SetElevatorExtensionSupplier([this] { return m_elevator.GetPositionMeters(); });
  m_wrist.SetPivotAngleSupplier([this] { return m_pivot.GetAngleRad(); });

  // Unless something else is running on them (manual jog, homing), the mechanisms follow the
  // Superstructure's targets.
  m_elevator.SetDefaultCommand(
      m_elevator.SetPosition([this] { return m_superstructure.ElevatorTargetMeters(); }));
  m_pivot.SetDefaultCommand(
      m_pivot.SetAngle([this] { return m_superstructure.PivotTargetDeg(); }));
  m_wrist.SetDefaultCommand(
      m_wrist.SetAngle([this] { return m_superstructure.WristTargetDeg(); }));

  frc::SmartDashboard::PutData("Auto Chooser", &m_autos.Chooser());
  ConfigureBindings();
}

RobotContainer::~RobotContainer() {
  // Tests build several robots in one process: drop this robot's commands, subsystems and button
  // bindings (they point at objects about to be destroyed), and start the next one from a fresh
  // simulated robot.
  auto& scheduler = frc2::CommandScheduler::GetInstance();
  scheduler.CancelAll();
  scheduler.UnregisterAllSubsystems();
  scheduler.GetDefaultButtonLoop()->Clear();
  sim::PhysicsWorld::Destroy();
}

/**
 * Driver controls (also the "Superstructure" machine's operator inputs in robot.mayhem.json). In
 * the sim GUI, drag "Keyboard 0" onto Joysticks[0]; then Z/X/C/V are A/B/X/Y.
 */
void RobotContainer::ConfigureBindings() {
  using Goal = Superstructure::Goal;
  auto request = [this](Goal goal) {
    return frc2::cmd::RunOnce([this, goal] { m_superstructure.Request(goal); });
  };
  m_driverController.A().OnTrue(request(Goal::STOW));
  m_driverController.B().OnTrue(request(Goal::L2));
  m_driverController.X().OnTrue(request(Goal::L3));
  m_driverController.Y().OnTrue(request(Goal::L4));
  m_driverController.LeftBumper().OnTrue(request(Goal::L1));
  m_driverController.RightBumper().OnTrue(request(Goal::CORAL_INTAKE));
}

void RobotContainer::Periodic() {
  RobotVisualizer::Update(m_pivot.GetAngleRad(), m_elevator.GetPositionMeters(),
                          m_wrist.GetAngleRad());
}
