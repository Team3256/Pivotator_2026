// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <memory>
#include <string>
#include <string_view>

#include <networktables/NetworkTableInstance.h>

#include "mayhem/networktables_observer.hpp"
#include "mayhem/wpilib_log.hpp"
#include "util/Log.h"

/**
 * Reports one Mayhem state machine to both places the Mayhem app reads it from: NetworkTables
 * (its live view) and the .wpilog (log replay and sim proof). Create it after Log::Start().
 */
class MayhemObserver final : public mayhem::StateObserver {
 public:
  explicit MayhemObserver(std::string machine)
      : m_nt(nt::NetworkTableInstance::GetDefault(), machine) {
    if (auto* log = Log::GetLog()) {
      m_log = std::make_unique<mayhem::WpiLogObserver>(*log, machine);
    }
  }

  void Describe(std::string_view json) {
    m_nt.Describe(json);
    if (m_log) {
      m_log->Describe(json);
    }
  }
  void RegisterGuard(std::string transition, std::string input) {
    m_nt.RegisterGuard(transition, input);
    if (m_log) {
      m_log->RegisterGuard(transition, input);
    }
  }
  void RegisterInput(std::string_view machine, const mayhem::MachineInput& input) override {
    m_nt.RegisterInput(machine, input);
    if (m_log) {
      m_log->RegisterInput(machine, input);
    }
  }
  void Input(std::string_view machine, std::string_view key, const mayhem::SignalValue& value,
             double time) override {
    m_nt.Input(machine, key, value, time);
    if (m_log) {
      m_log->Input(machine, key, value, time);
    }
  }
  void Enter(std::string_view machine, std::string_view state, double time) override {
    m_nt.Enter(machine, state, time);
    if (m_log) {
      m_log->Enter(machine, state, time);
    }
  }
  void Guard(std::string_view machine, std::string_view transition, std::string_view input,
             bool value, double time) override {
    m_nt.Guard(machine, transition, input, value, time);
    if (m_log) {
      m_log->Guard(machine, transition, input, value, time);
    }
  }
  void Transition(std::string_view machine, std::string_view transition, std::string_view from,
                  std::string_view to, double time) override {
    m_nt.Transition(machine, transition, from, to, time);
    if (m_log) {
      m_log->Transition(machine, transition, from, to, time);
    }
  }
  void GoalProgress(std::string_view goal, double translationError, double rotationError,
                    bool reached, double time) override {
    m_nt.GoalProgress(goal, translationError, rotationError, reached, time);
    if (m_log) {
      m_log->GoalProgress(goal, translationError, rotationError, reached, time);
    }
  }

 private:
  mayhem::NtStateObserver m_nt;
  std::unique_ptr<mayhem::WpiLogObserver> m_log;
};
