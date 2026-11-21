// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "sim/PhysicsWorld.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <variant>

#include <frc/DriverStation.h>
#include <frc/simulation/RoboRioSim.h>
#include <units/angle.h>
#include <units/voltage.h>

#include "mayhem/sim/phoenix6.hpp"
#include "sim/RobotVisualizer.h"
#include "subsystems/elevator/ElevatorConstants.h"
#include "subsystems/pivot/PivotConstants.h"
#include "subsystems/wrist/WristConstants.h"
#include "util/Log.h"

namespace sim {
namespace {

namespace EC = ElevatorConstants;
namespace PC = PivotConstants;
namespace WC = WristConstants;

constexpr double kDt = 0.001;
constexpr double kGravity = 9.81;
/** roboRIO brownout: motor outputs turn off below this (RobotController's default 6.75 V). */
constexpr double kBrownoutVolts = 6.75;

double Rad(double degrees) {
  return degrees * std::numbers::pi / 180.0;
}

double Deg(double radians) {
  return radians * 180.0 / std::numbers::pi;
}

double PivotTotalMassKg() {
  return PC::kFixedStageMassKg + PC::kStage1MassKg + PC::kCarriageMassKg;
}

/** Distance from the pivot axle to the centre of mass of everything the pivot carries. */
double PivotComMeters(double elevatorMeters) {
  return PC::GravityTorqueHorizontalNm(elevatorMeters) / (PivotTotalMassKg() * kGravity);
}

std::unique_ptr<PhysicsWorld> g_world;

}  // namespace

struct PhysicsWorld::MotorEntry {
  ctre::phoenix6::hardware::TalonFX* device;
  int id;
  Mechanism mechanism;
  mayhem::sim::TalonFx emulator;
  bool follower = false;
};

PhysicsWorld& PhysicsWorld::Get() {
  if (!g_world) {
    g_world.reset(new PhysicsWorld());
  }
  return *g_world;
}

bool PhysicsWorld::Exists() {
  return g_world != nullptr;
}

void PhysicsWorld::Destroy() {
  g_world.reset();
}

PhysicsWorld::PhysicsWorld() {
  // Gearing is rotor rotations per mechanism rotation (the configs' SensorToMechanismRatio), so
  // each emulator's position is in the robot code's mechanism units.
  m_pivot = {PC::MomentOfInertia(0.0),
             PC::kGearRatio,
             PivotTotalMassKg(),
             PivotComMeters(0.0),
             Rad(PC::kMinAngleDeg),
             Rad(PC::kMaxAngleDeg),
             0,  // rigid hard stops
             Rad(PC::kBootAngleDeg),
             0,
             0};
  m_elevator = {EC::Sim::kCarriageMassKg,
                EC::kDrumRadiusMeters * EC::kStageMultiplier,
                EC::kGearRatio,
                EC::kMinHeightMeters,
                EC::kMaxHeightMeters,
                Rad(PC::kBootAngleDeg),  // updated every step from the pivot angle
                0,
                EC::Sim::kStartingHeightMeters,
                0,
                0};
  m_wrist = {WC::kMomentOfInertia,
             WC::kGearRatio,
             WC::kMassKg,
             WC::kComMeters,
             Rad(WC::kMinAngleDeg),
             Rad(WC::kMaxAngleDeg),
             0,
             Rad(WC::kBootAngleDeg),
             0,
             0};
}

void PhysicsWorld::AttachMotor(Mechanism mechanism, ctre::phoenix6::hardware::TalonFX& device,
                               const ctre::phoenix6::configs::TalonFXConfiguration& config,
                               const mayhem::sim::DcMotor& trapezoidal,
                               const mayhem::sim::DcMotor& foc) {
  m_motors.push_back(std::make_unique<MotorEntry>(MotorEntry{
      &device, device.GetDeviceID(), mechanism,
      mayhem::sim::TalonFx{trapezoidal, foc, mayhem::sim::FromPhoenix(config)}}));
}

PhysicsWorld::MotorEntry* PhysicsWorld::Find(
    const ctre::phoenix6::hardware::TalonFX& device) const {
  for (const auto& m : m_motors) {
    if (m->device == &device) {
      return m.get();
    }
  }
  return nullptr;
}

const mayhem::sim::TalonFx& PhysicsWorld::Motor(
    const ctre::phoenix6::hardware::TalonFX& device) const {
  const auto* m = Find(device);
  if (m == nullptr) {
    throw std::logic_error("PhysicsWorld: motor was never attached");
  }
  return m->emulator;
}

void PhysicsWorld::SetMechanismPosition(const ctre::phoenix6::hardware::TalonFX& device,
                                        double rotations) {
  if (auto* m = Find(device)) {
    m->emulator.SetMechanismPosition(rotations);
  }
}

double PhysicsWorld::PivotAngleRad() {
  return m_pivot.Angle();
}

double PhysicsWorld::ElevatorMeters() {
  return m_elevator.Position();
}

double PhysicsWorld::WristAngleRad() {
  return m_wrist.Angle();
}

frc::Pose2d PhysicsWorld::TruePose() const {
  return RobotVisualizer::ParkedRobotPose().ToPose2d();
}

frc::Pose3d PhysicsWorld::TruePose3d() const {
  return RobotVisualizer::ParkedRobotPose();
}

void PhysicsWorld::Step(double loopSeconds) {
  const int steps = std::max(1, static_cast<int>(std::lround(loopSeconds / kDt)));
  ReadControls();
  // A disabled robot's TalonFXs output neutral (brake or coast, as configured), whatever they
  // were last told.
  const bool enabled = frc::DriverStation::IsEnabled();
  for (int k = 0; k < steps; k++) {
    // The bus the motors see: the battery through the wiring, low-passed (Battery::BusVoltage).
    const double bus = m_battery.BusVoltage();
    if (bus < kBrownoutVolts) {
      m_brownedOut = true;
      m_metrics.brownedOut = true;
    } else if (bus > kBrownoutVolts + 0.5) {
      m_brownedOut = false;
    }
    if (m_brownedOut || !enabled) {
      for (auto& m : m_motors) {
        m->emulator.SetControl(mayhem::sim::NeutralOut{});
      }
    }
    StepMechanisms(kDt, bus);
    // Signed: regeneration (a mechanism being lowered) charges the battery.
    double current = 0;
    for (const auto& m : m_motors) {
      current += m->emulator.SupplyCurrent();
    }
    m_battery.Step(current, kDt);
    m_metrics.maxTotalCurrent = std::max(m_metrics.maxTotalCurrent, current);
    m_metrics.minBatteryVolts = std::min(m_metrics.minBatteryVolts, m_battery.BusVoltage());
    m_time += kDt;
  }
  frc::sim::RoboRioSim::SetVInVoltage(units::volt_t{m_battery.BusVoltage()});
  Publish();
}

void PhysicsWorld::ReadControls() {
  for (auto& m : m_motors) {
    const auto applied = m->device->GetAppliedControl();
    if (!applied) {
      continue;
    }
    const auto request = mayhem::sim::FromApplied(*applied);
    if (!request) {
      continue;
    }
    int leaderId = -1;
    if (const auto* f = std::get_if<mayhem::sim::Follower>(&*request)) {
      leaderId = f->leaderId;
    } else if (const auto* s = std::get_if<mayhem::sim::StrictFollower>(&*request)) {
      leaderId = s->leaderId;
    }
    m->follower = leaderId >= 0;
    if (m->follower) {
      for (auto& other : m_motors) {
        if (other->id == leaderId && other.get() != m.get()) {
          m->emulator.SetLeader(&other->emulator);
        }
      }
    }
    m->emulator.SetControl(*request);
  }
}

void PhysicsWorld::StepMechanisms(double dt, double bus) {
  auto step = [&](Mechanism which, auto& plant) {
    double torque = 0, damping = 0;
    const double position = plant.RotorPosition(), velocity = plant.RotorVelocity();
    // Leaders first: a follower copies its leader's output from this step.
    for (bool followers : {false, true}) {
      for (auto& m : m_motors) {
        if (m->mechanism == which && m->follower == followers) {
          torque += m->emulator.Step(dt, bus, position, velocity);
          damping += m->emulator.RotorDamping();
        }
      }
    }
    plant.Step(dt, torque, damping);
  };

  // Couple the mechanisms: the pivot carries the extended elevator, the elevator and wrist ride
  // on the pivot.
  const double extension = m_elevator.Position();
  m_pivot.inertia = PC::MomentOfInertia(extension);
  m_pivot.comLengthM = PivotComMeters(extension);
  step(Mechanism::kPivot, m_pivot);

  const double pivotAngle = m_pivot.Angle();
  m_elevator.angleFromHorizontal = pivotAngle;
  step(Mechanism::kElevator, m_elevator);

  m_wrist.SetPitchOffset(pivotAngle);
  step(Mechanism::kWrist, m_wrist);
}

void PhysicsWorld::Publish() {
  const double pivot = m_pivot.Angle(), elevator = m_elevator.Position(),
               wrist = m_wrist.Angle();
  Log::Record("Sim/RobotPose", TruePose3d());
  Log::Record("Sim/Components", RobotVisualizer::ComponentPoses(pivot, elevator, wrist));
  Log::Record("Sim/PivotAngleDeg", Deg(pivot));
  Log::Record("Sim/ElevatorMeters", elevator);
  Log::Record("Sim/WristAngleDeg", Deg(wrist));
  Log::Record("Sim/BatteryVolts", m_battery.BusVoltage());
  Log::Record("Sim/TotalCurrentAmps", m_battery.TotalCurrent());
  Log::Record("Sim/BrownedOut", m_brownedOut);
}

}  // namespace sim
