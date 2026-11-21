// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "sim/PhysicsWorld.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <stdexcept>
#include <variant>

#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/simulation/RoboRioSim.h>
#include <units/angle.h>
#include <units/voltage.h>

#include "commands/MayhemProject.h"
#include "mayhem/sim/phoenix6.hpp"
#include "mayhem/sim/swerve_requests.hpp"
#include "mayhem/sim/wpilib_sim.hpp"
#include "sim/Reefscape2025.h"
#include "sim/RobotVisualizer.h"
#include "subsystems/elevator/ElevatorConstants.h"
#include "subsystems/pivot/PivotConstants.h"
#include "subsystems/swerve/generated/TunerConstants.h"
#include "subsystems/wrist/WristConstants.h"
#include "util/Log.h"

namespace sim {
namespace {

namespace EC = ElevatorConstants;
namespace PC = PivotConstants;
namespace WC = WristConstants;

constexpr double kDt = 0.001;
constexpr int kTicksPerControl = 4;  // Phoenix's odometry/control thread: 250 Hz
constexpr double kGravity = 9.81;
/** roboRIO brownout: motor outputs turn off below this (RobotController's default 6.75 V). */
constexpr double kBrownoutVolts = 6.75;
/** A contact step with more impulse than this (N s per 1 ms, i.e. 300 N) counts as a hit. */
constexpr double kHitImpulse = 0.3;

/**
 * The robot's mass properties and tread, for the swerve physics. Estimates: TODO measure (weigh
 * the robot with battery and bumpers; CAD mass properties for the centre of mass and inertia).
 */
namespace robot {
constexpr double kMassKg = 135 * 0.45359237;  // with battery and bumpers
constexpr double kMoiKgM2 = 6.5;
constexpr double kCgHeightM = 0.24;  // elevator stowed upright; it rises as the elevator extends
constexpr double kBumperLengthM = 0.85;
constexpr double kBumperWidthM = 0.85;
constexpr double kFrameLengthM = 0.686;
constexpr double kFrameClearanceM = 0.041;
constexpr double kMuStatic = 1.15, kMuKinetic = 1.0;
constexpr double kWheelInertia = 0.0035, kSteerInertia = 0.01;
}  // namespace robot

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

mayhem::sim::SwerveParams MakeSwerveParams() {
  mayhem::sim::SwerveParams s;
  const auto& m = TunerConstants::FrontLeft;
  s.massKg = robot::kMassKg;
  s.moiKgM2 = robot::kMoiKgM2;
  s.cgHeightM = robot::kCgHeightM;
  for (const auto* module : {&TunerConstants::FrontLeft, &TunerConstants::FrontRight,
                             &TunerConstants::BackLeft, &TunerConstants::BackRight}) {
    s.modulePositions.push_back({module->LocationX.value(), module->LocationY.value()});
  }
  s.wheelRadius = m.WheelRadius.value();
  s.driveGearing = m.DriveMotorGearRatio;
  s.steerGearing = m.SteerMotorGearRatio;
  s.couplingRatio = m.CouplingGearRatio;
  s.driveMotor = mayhem::sim::DcMotor::KrakenX60();
  s.driveMotorFoc = mayhem::sim::DcMotor::KrakenX60Foc();
  s.steerMotor = mayhem::sim::DcMotor::KrakenX60();
  s.steerMotorFoc = mayhem::sim::DcMotor::KrakenX60Foc();
  s.wheelInertia = robot::kWheelInertia;
  s.steerInertia = robot::kSteerInertia;
  s.muStatic = robot::kMuStatic;
  s.muKinetic = robot::kMuKinetic;
  s.bumperLengthX = robot::kBumperLengthM;
  s.bumperLengthY = robot::kBumperWidthM;
  s.frameLengthX = robot::kFrameLengthM;
  s.frameLengthY = robot::kFrameLengthM;
  s.frameClearanceM = robot::kFrameClearanceM;
  s.robotHeightM = 1.0;
  return s;
}

std::unique_ptr<PhysicsWorld> g_world;
bool g_nextField = true;

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

void PhysicsWorld::SetNextFieldEnabled(bool enabled) {
  g_nextField = enabled;
}

PhysicsWorld::PhysicsWorld() {
  // Mechanism plants. Gearing is rotor rotations per mechanism rotation (the configs'
  // SensorToMechanismRatio), so each emulator's position is in the robot code's mechanism units.
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

  // Drivetrain, configured the way Phoenix configures the real modules.
  m_swerve = std::make_unique<mayhem::sim::SwerveSim>(MakeSwerveParams());
  const auto& m = TunerConstants::FrontLeft;
  auto constants = mayhem::sim::swerve::DrivetrainConstants::From(
      MakeSwerveParams(), TunerConstants::kSpeedAt12Volts.value());
  mayhem::sim::swerve::ConfigureLikePhoenix(
      *m_swerve, constants, mayhem::sim::FromPhoenix(m.SteerMotorGains),
      mayhem::sim::FromPhoenix(m.DriveMotorGains), m.SlipCurrent.value(),
      mayhem::sim::FromPhoenix(m.DriveMotorInitialConfigs),
      mayhem::sim::FromPhoenix(m.SteerMotorInitialConfigs));

  m_fieldEnabled = g_nextField;
  if (m_fieldEnabled) {
    reefscape::BuildField(m_obstacles);
  }
  // Start where the Mayhem project says the robot starts.
  const auto& start = mayhem_generated::startPose;
  SetTruePose(frc::Pose2d{units::meter_t{start.x}, units::meter_t{start.y},
                          frc::Rotation2d{units::radian_t{start.heading}}});
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
  return mayhem::sim::ToPose2d(m_swerve->Pose());
}

frc::Pose3d PhysicsWorld::TruePose3d() const {
  return mayhem::sim::ToPose3d(m_swerve->Pose3());
}

void PhysicsWorld::SetTruePose(const frc::Pose2d& pose) {
  m_swerve->SetPose(pose.X().value(), pose.Y().value(), pose.Rotation().Radians().value(),
                    &m_terrain);
}

void PhysicsWorld::OnOdometryReset(const frc::Pose2d& pose) {
  if (!m_keepPoseOnReset) {
    SetTruePose(pose);
  }
}

double PhysicsWorld::RobotHeight() {
  // The elevator's top along its axis from the pivot axle (0.34 m up), at the pivot angle.
  const double reach = 0.78 + m_elevator.Position();
  return std::max(0.55, 0.34 + reach * std::sin(m_pivot.Angle()));
}

void PhysicsWorld::Step(double loopSeconds) {
  const int steps = std::max(1, static_cast<int>(std::lround(loopSeconds / kDt)));
  const double start = frc::Timer::GetFPGATimestamp().value();
  ReadControls();
  // A disabled robot's TalonFXs output neutral (brake or coast, as configured), whatever they
  // were last told; the swerve request is re-applied every tick, so it is overridden there too.
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
    if (k % kTicksPerControl == 0 && m_driveHooks) {
      m_driveHooks->ControlTick(start + k * kDt);
    }
    if (m_brownedOut || !enabled) {
      for (std::size_t i = 0; i < m_swerve->ModuleCount(); i++) {
        m_swerve->DriveMotor(i).SetControl(mayhem::sim::NeutralOut{});
        m_swerve->SteerMotor(i).SetControl(mayhem::sim::NeutralOut{});
      }
      for (auto& m : m_motors) {
        m->emulator.SetControl(mayhem::sim::NeutralOut{});
      }
    }
    const auto pose = m_swerve->Pose3();
    StepMechanisms(kDt, bus, -pose.pitch);
    m_swerve->SetRobotHeight(RobotHeight());
    m_swerve->Step(kDt, bus, &m_terrain, &m_obstacles);
    // Signed: regeneration (a robot braking, a mechanism lowered) charges the battery.
    double current = m_swerve->TotalSupplyCurrent();
    for (const auto& m : m_motors) {
      current += m->emulator.SupplyCurrent();
    }
    m_battery.Step(current, kDt);
    m_metrics.maxTotalCurrent = std::max(m_metrics.maxTotalCurrent, current);
    m_metrics.minBatteryVolts = std::min(m_metrics.minBatteryVolts, m_battery.BusVoltage());
    UpdateMetrics(kDt);
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

void PhysicsWorld::StepMechanisms(double dt, double bus, double robotPitch) {
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
  // on the pivot, and the robot's own pitch on the field tilts everything.
  const double extension = m_elevator.Position();
  m_pivot.inertia = PC::MomentOfInertia(extension);
  m_pivot.comLengthM = PivotComMeters(extension);
  m_pivot.SetPitchOffset(robotPitch);
  step(Mechanism::kPivot, m_pivot);

  const double pivotAngle = m_pivot.Angle();
  m_elevator.angleFromHorizontal = pivotAngle + robotPitch;
  step(Mechanism::kElevator, m_elevator);

  m_wrist.SetPitchOffset(pivotAngle + robotPitch);
  step(Mechanism::kWrist, m_wrist);
}

void PhysicsWorld::UpdateMetrics(double dt) {
  m_metrics.maxTiltDeg = std::max(m_metrics.maxTiltDeg, Deg(m_swerve->TiltAngle()));
  m_metrics.tippedOver = m_metrics.tippedOver || m_swerve->TippedOver();
  if (m_swerve->Beached()) {
    m_metrics.beachedSeconds += dt;
  }
  // A new contact with an obstacle (counted once per touch, not once per millisecond).
  std::set<std::string> touching;
  for (const auto& e : m_swerve->Contacts()) {
    if (e.impulse < kHitImpulse) {
      continue;
    }
    const std::string name{e.name};
    touching.insert(name);
    if (m_touching.count(name) == 0) {
      const bool wall = name.rfind("perimeter", 0) == 0;
      (wall ? m_metrics.wallHits : m_metrics.obstacleHits)++;
      const auto p = m_swerve->Pose();
      char text[160];
      std::snprintf(text, sizeof text, "%s at t=%.2f robot (%.2f, %.2f, %.0f deg)", name.c_str(),
                    m_time, p.x, p.y, Deg(p.yaw));
      m_metrics.hits.emplace_back(text);
    }
  }
  m_touching = std::move(touching);
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
  Log::Record("Sim/TiltDegrees", Deg(m_swerve->TiltAngle()));
  Log::Record("Sim/RobotHeightMeters", RobotHeight());
  Log::Record("Sim/WallHits", static_cast<double>(m_metrics.wallHits));
  Log::Record("Sim/ObstacleHits", static_cast<double>(m_metrics.obstacleHits));
}

}  // namespace sim
