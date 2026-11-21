// Copyright (c) 2026 FRC 3256
// https://github.com/Team3256

#pragma once

#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <thread>

#include <frc/simulation/DriverStationSim.h>
#include <frc/simulation/SimHooks.h>
#include <frc/simulation/XboxControllerSim.h>
#include <hal/DriverStationTypes.h>

#include "Robot.h"

/** The robot program with its loop function exposed so a test can step it. */
class SimRobot : public Robot {
 public:
  using frc::IterativeRobotBase::LoopFunc;
};

/**
 * Runs the whole robot program (Robot -> RobotContainer -> every subsystem, simulated) one 20 ms
 * loop at a time on stepped simulated time: the HAL clock is paused and advanced 20 ms per loop,
 * and every mechanism runs on sim::PhysicsWorld (Mayhem's physics engine, which doesn't use the
 * wall clock), so a loop takes only as long as the code. Set ROBOT_SIM_REALTIME=1 to pace loops at
 * 20 ms of wall time instead (to watch a test live in AdvantageScope or Mayhem).
 *
 * Mayhem's sim proof uses this class: it copies its probe test next to this file and calls
 * RobotHarness(red, field), Step(), Teleop()/Autonomous()/Disable(), Driver() and
 * C().GetAutoChooser().Select(name). Keep those working.
 */
class RobotHarness {
 public:
  /** {@code field}: accepted for Mayhem's probe; this robot has no field simulation yet. */
  explicit RobotHarness(bool red = false, bool field = false) {
    (void)field;
    if (std::getenv("ROBOT_LOG_DIR") == nullptr) {
      setenv("ROBOT_NO_LOG", "1", 0);  // plain tests don't leave log files around
    }
    m_stepped = std::getenv("ROBOT_SIM_REALTIME") == nullptr;
    if (m_stepped) {
      // The first robot in the process starts the clock at exactly 0 (repeatable runs); later
      // robots continue from where the last one stopped, because NetworkTables ignores values
      // older than the one it holds.
      static bool clockStarted = false;
      frc::sim::PauseTiming();
      if (!clockStarted) {
        frc::sim::RestartTiming();
        clockStarted = true;
      }
    }
    frc::sim::DriverStationSim::ResetData();
    frc::sim::DriverStationSim::SetDsAttached(true);
    SetAlliance(red);
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
    m_robot = std::make_unique<SimRobot>();
    m_driver = std::make_unique<frc::sim::XboxControllerSim>(0);
    m_next = std::chrono::steady_clock::now();
  }

  ~RobotHarness() {
    Disable();
    Step(1);
    m_robot.reset();
  }

  /** Runs {@code loops} robot loops. */
  void Step(int loops = 1) {
    for (int i = 0; i < loops; i++) {
      if (m_stepped) {
        // Async: TimedRobot's own notifier is never waited on (the harness calls LoopFunc), so
        // StepTiming, which waits for every active notifier, would hang.
        frc::sim::StepTimingAsync(20_ms);
      } else {
        m_next += std::chrono::milliseconds(20);
        std::this_thread::sleep_until(m_next);
      }
      frc::sim::DriverStationSim::NotifyNewData();
      m_robot->LoopFunc();
    }
  }

  /** Steps until {@code condition} is true; false if it isn't true within {@code maxLoops}. */
  bool StepUntil(const std::function<bool()>& condition, int maxLoops) {
    for (int i = 0; i < maxLoops; i++) {
      if (condition()) {
        return true;
      }
      Step();
    }
    return condition();
  }

  void Teleop() { SetMode(true, false); }
  void Autonomous() { SetMode(true, true); }
  void Disable() { SetMode(false, false); }

  void SetAlliance(bool red) {
    frc::sim::DriverStationSim::SetAllianceStationId(red ? HAL_AllianceStationID_kRed1
                                                         : HAL_AllianceStationID_kBlue1);
    frc::sim::DriverStationSim::NotifyNewData();
  }

  /** Presses a driver button for one loop, then releases it. */
  void Tap(const std::function<void(frc::sim::XboxControllerSim&, bool)>& button) {
    button(*m_driver, true);
    Step();
    button(*m_driver, false);
    Step();
  }

  frc::sim::XboxControllerSim& Driver() { return *m_driver; }
  RobotContainer& C() { return m_robot->Container(); }
  SimRobot& R() { return *m_robot; }

 private:
  void SetMode(bool enabled, bool autonomous) {
    frc::sim::DriverStationSim::SetAutonomous(autonomous);
    frc::sim::DriverStationSim::SetTest(false);
    frc::sim::DriverStationSim::SetEnabled(enabled);
    frc::sim::DriverStationSim::NotifyNewData();
  }

  std::unique_ptr<SimRobot> m_robot;
  std::unique_ptr<frc::sim::XboxControllerSim> m_driver;
  std::chrono::steady_clock::time_point m_next;
  bool m_stepped = false;
};
