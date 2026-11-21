// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <functional>
#include <string_view>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

/**
 * A subsystem that can be turned off from RobotContainer. When disabled, every command it builds
 * is an empty "do nothing" command, so a mechanism that isn't on the robot yet can't move.
 *
 * Subsystems should build commands with these Run/RunOnce/... (not frc2::cmd::Run directly) so
 * the disable switch works.
 */
class DisableSubsystem : public frc2::SubsystemBase {
 public:
  DisableSubsystem(bool enabled, std::string_view name);

  const bool disabled;

  // These hide SubsystemBase's versions so they return a no-op command when disabled.
  frc2::CommandPtr Run(std::function<void()> action);
  frc2::CommandPtr RunOnce(std::function<void()> action);
  frc2::CommandPtr StartEnd(std::function<void()> start, std::function<void()> end);
  frc2::CommandPtr RunEnd(std::function<void()> run, std::function<void()> end);

  /** Does nothing when disabled (a default command must require its subsystem). */
  void SetDefaultCommand(frc2::CommandPtr&& defaultCommand);

  /** Logs "<Name>/CurrentCommand". Subclasses must call DisableSubsystem::Periodic(). */
  void Periodic() override;
};
