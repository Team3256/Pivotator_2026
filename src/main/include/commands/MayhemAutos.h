// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>

#include "commands/MayhemObserver.h"
#include "mayhem/auto_runner.hpp"
#include "subsystems/Superstructure.h"

/**
 * Dashboard chooser for the autonomous routines in robot.mayhem.json (names and order come from
 * the project). Mayhem's sim proof picks an auto with Select(name), as the dashboard would.
 */
class AutoChooser : public wpi::Sendable, public wpi::SendableHelper<AutoChooser> {
 public:
  static constexpr const char* kNoneName = "Nothing";

  AutoChooser();

  /** Selects an option by name. Unknown names are ignored. */
  void Select(const std::string& name);
  const std::string& Selected() const { return m_selected; }
  const std::vector<std::string>& Options() const { return m_options; }

  void InitSendable(wpi::SendableBuilder& builder) override;

 private:
  std::vector<std::string> m_options;
  std::string m_selected = kNoneName;
};

/**
 * Runs the selected Mayhem routine with mayhem::AutoRunner. Each action ID a routine can use is
 * bound to the Superstructure here; a routine that uses an ID the robot doesn't handle is
 * reported as a Driver Station error at boot.
 *
 *   superstructure.stow / .intake / .scoreL1 .. .scoreL4: request that goal; the step succeeds
 *   once the Motion machine says the mechanisms got there.
 *   superstructure.score: score the held coral (at a reef level); succeeds once it's out.
 *
 * The runner reports as the "Autonomous" machine (live and in logs), and its status goes to
 * Mayhem/Auto/Status, which Mayhem's sim proof reads to decide whether an auto completed.
 */
class MayhemAutos {
 public:
  explicit MayhemAutos(Superstructure& superstructure);

  AutoChooser& Chooser() { return m_chooser; }

  /** autonomousInit: build and start the selected routine. */
  void Start();
  /** Every autonomous loop. */
  void Periodic();
  /** Leaving autonomous: cancel a routine that's still running. */
  void Stop();

  /** The current (or last) routine's status; Idle when none ran. */
  mayhem::RunStatus Status() const {
    return m_runner ? m_runner->Status() : mayhem::RunStatus::Idle;
  }

 private:
  std::unique_ptr<mayhem::AutoRunner> Build(const mayhem::RoutineInfo& routine);
  void LogStatus();

  Superstructure& m_superstructure;
  AutoChooser m_chooser;
  MayhemObserver m_observer{"Autonomous"};
  std::unique_ptr<mayhem::AutoRunner> m_runner;
  std::string m_running = AutoChooser::kNoneName;
};
