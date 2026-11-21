// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#include "util/DisableSubsystem.h"

#include <string>
#include <utility>

#include <frc2/command/Commands.h>

#include "util/Log.h"

DisableSubsystem::DisableSubsystem(bool enabled, std::string_view name)
    : frc2::SubsystemBase(name), disabled(!enabled) {}

frc2::CommandPtr DisableSubsystem::Run(std::function<void()> action) {
  if (disabled) {
    return frc2::cmd::None();
  }
  return frc2::SubsystemBase::Run(std::move(action));
}

frc2::CommandPtr DisableSubsystem::RunOnce(std::function<void()> action) {
  if (disabled) {
    return frc2::cmd::None();
  }
  return frc2::SubsystemBase::RunOnce(std::move(action));
}

frc2::CommandPtr DisableSubsystem::StartEnd(std::function<void()> start,
                                            std::function<void()> end) {
  if (disabled) {
    return frc2::cmd::None();
  }
  return frc2::SubsystemBase::StartEnd(std::move(start), std::move(end));
}

frc2::CommandPtr DisableSubsystem::RunEnd(std::function<void()> run,
                                          std::function<void()> end) {
  if (disabled) {
    return frc2::cmd::None();
  }
  return frc2::SubsystemBase::RunEnd(std::move(run), std::move(end));
}

void DisableSubsystem::SetDefaultCommand(frc2::CommandPtr&& defaultCommand) {
  if (!disabled) {
    frc2::SubsystemBase::SetDefaultCommand(std::move(defaultCommand));
  }
}

void DisableSubsystem::Periodic() {
  frc2::Command* current = GetCurrentCommand();
  Log::Record(GetName() + "/CurrentCommand",
              current != nullptr ? current->GetName() : std::string{"none"});
}
