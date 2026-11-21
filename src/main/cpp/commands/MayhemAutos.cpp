// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "commands/MayhemAutos.h"

#include <algorithm>
#include <array>

#include <frc/DriverStation.h>
#include <frc/Errors.h>
#include <frc/Timer.h>
#include <wpi/sendable/SendableBuilder.h>

#include "commands/MayhemProject.h"
#include "util/Log.h"

namespace {

double Now() {
  return frc::Timer::GetFPGATimestamp().value();
}

std::string_view StatusName(mayhem::RunStatus status) {
  switch (status) {
    case mayhem::RunStatus::Idle:
      return "Idle";
    case mayhem::RunStatus::Running:
      return "Running";
    case mayhem::RunStatus::Completed:
      return "Completed";
    case mayhem::RunStatus::Failed:
      return "Failed";
    case mayhem::RunStatus::Cancelled:
      return "Cancelled";
  }
  return "Unknown";
}

frc::Pose2d ToWpi(const mayhem::Pose& p) {
  return {units::meter_t{p.x}, units::meter_t{p.y}, frc::Rotation2d{units::radian_t{p.heading}}};
}

}  // namespace

// ---------------- AutoChooser ----------------

AutoChooser::AutoChooser() {
  m_options.push_back(kNoneName);
  for (const auto& routine : mayhem_generated::routines) {
    m_options.emplace_back(routine.name);
  }
}

void AutoChooser::Select(const std::string& name) {
  if (std::find(m_options.begin(), m_options.end(), name) != m_options.end()) {
    m_selected = name;
  }
}

void AutoChooser::InitSendable(wpi::SendableBuilder& builder) {
  // Same layout as frc::SendableChooser, so dashboards show it as a dropdown.
  builder.SetSmartDashboardType("String Chooser");
  builder.AddStringArrayProperty("options", [this] { return m_options; }, nullptr);
  builder.AddStringProperty("default", [] { return std::string{kNoneName}; }, nullptr);
  builder.AddStringProperty("active", [this] { return m_selected; }, nullptr);
  builder.AddStringProperty("selected", [this] { return m_selected; },
                            [this](std::string_view name) { Select(std::string{name}); });
}

// ---------------- MayhemAutos ----------------

MayhemAutos::MayhemAutos(Drivetrain& drivetrain, Superstructure& superstructure)
    : m_drivetrain{drivetrain}, m_superstructure{superstructure} {
  // Fail loudly at boot (not mid-match) if a routine uses an action the robot doesn't handle.
  for (const auto& routine : mayhem_generated::routines) {
    const auto report = Build(routine.routine)->UnboundReport();
    if (!report.empty()) {
      FRC_ReportError(frc::err::Error, "Auto '{}' uses actions the robot doesn't handle:\n{}",
                      routine.name, report);
    }
  }
  LogStatus();
}

std::unique_ptr<mayhem::AutoRunner> MayhemAutos::Build(mayhem::RoutineView routine) {
  using Goal = Superstructure::Goal;
  auto runner = std::make_unique<mayhem::AutoRunner>(routine);
  auto goTo = [this](Goal goal) {
    return mayhem::Action{
        [this, goal] { m_superstructure.Request(goal); },
        [this, goal] {
          return m_superstructure.IsAt(goal) ? mayhem::ActionStatus::Succeeded
                                             : mayhem::ActionStatus::Running;
        },
        {}};
  };
  runner->Bind("superstructure.stow", goTo(Goal::STOW))
      .Bind("superstructure.intake", goTo(Goal::CORAL_INTAKE))
      .Bind("superstructure.scoreL1", goTo(Goal::L1))
      .Bind("superstructure.scoreL2", goTo(Goal::L2))
      .Bind("superstructure.scoreL3", goTo(Goal::L3))
      .Bind("superstructure.scoreL4", goTo(Goal::L4));
  runner->Attach(m_observer, mayhem_generated::autonomous::manifest, Now);
  return runner;
}

mayhem::Pose MayhemAutos::MeasuredPose() {
  const auto& p = m_drivetrain.Pose();
  return {p.X().value(), p.Y().value(), p.Rotation().Radians().value()};
}

mayhem::Velocity MayhemAutos::MeasuredVelocity() {
  const auto v = m_drivetrain.GetFieldRelativeSpeeds();
  return {v.vx.value(), v.vy.value(), v.omega.value()};
}

void MayhemAutos::Start() {
  m_runner.reset();
  m_routine.reset();
  m_running = m_chooser.Selected();
  if (const auto* info = mayhem::FindRoutine(mayhem_generated::routines, m_running)) {
    m_routine = std::make_unique<mayhem::AllianceRoutine>(*info, mayhem_generated::fieldLength,
                                                          mayhem_generated::fieldWidth);
    const bool red = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed;
    m_routine->Apply(red ? mayhem::FieldTransform::Flip : mayhem::FieldTransform::None);
    // The robot is placed at the routine's start pose before the match.
    m_drivetrain.ResetPose(ToWpi(m_routine->StartPose()));
    m_runner = Build(m_routine->View());
    m_runner->Start(MeasuredVelocity());
  }
  LogStatus();
}

void MayhemAutos::Periodic() {
  if (m_runner && m_runner->Status() == mayhem::RunStatus::Running) {
    const mayhem::Velocity v = m_runner->Tick(MeasuredPose(), MeasuredVelocity());
    m_drivetrain.ApplyMayhemSpeeds(frc::ChassisSpeeds{units::meters_per_second_t{v.vx},
                                                      units::meters_per_second_t{v.vy},
                                                      units::radians_per_second_t{v.omega}});
    Log::Record("Mayhem/Auto/FieldSpeeds", std::array<double, 3>{v.vx, v.vy, v.omega});
  } else {
    // Done (or no routine): hold still until autonomous ends.
    m_drivetrain.Stop();
  }
  LogStatus();
}

void MayhemAutos::Stop() {
  // Only a routine that's still going is cancelled. (AutoRunner::Cancel() would also overwrite a
  // Completed/Failed status, and the sim proof reads the final status.)
  if (m_runner && m_runner->Status() == mayhem::RunStatus::Running) {
    m_runner->Cancel();
  }
  LogStatus();
}

void MayhemAutos::LogStatus() {
  Log::Record("Mayhem/Auto/Name", m_running);
  Log::Record("Mayhem/Auto/Status",
              m_runner ? StatusName(m_runner->Status()) : std::string_view{"Idle"});
}
