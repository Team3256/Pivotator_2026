# Mayhem integration

[Mayhem](https://github.com/ryanabraham1/Mayhem) is where we review this robot: its state
machines live, in logs and in simulation, sim proof on every commit, and a 3D replay of what the
robot did.

| File | What it is |
| --- | --- |
| `robot.mayhem.json` | The Mayhem project: the field (2025 REEFSCAPE), the autos, and the `Superstructure` and `Motion` state machines. Open it in the Mayhem app. |
| `tools/mayhem/machines.json` | The two state machines as a compact spec (edit this, then apply it, below). |
| `tools/mayhem/scenarios.json` | Sim proof regression checks. Add one for every behavior a driver relies on; never loosen one to make a run pass. |
| `tools/mayhem/explainer.json` | Plain-language description of each part (Overview → How the robot works). |
| `src/main/include/commands/MayhemProject.h` | Generated from `robot.mayhem.json`. Don't edit by hand. |
| `src/main/include/mayhem/` | Mayhem's C++ library, vendored. Don't edit; re-vendor. Commit pinned in `docs/mayhem/vendor-snapshot.json`. |
| `src/test/cpp/RobotHarness.h` | Runs the whole robot on stepped simulated time. Mayhem's sim proof drives the robot through it. |

## Driver controls

| Button | Superstructure goal |
| --- | --- |
| A | STOW |
| RB | CORAL_INTAKE |
| LB / B / X / Y | L1 / L2 / L3 / L4 |

The `Motion` machine then moves the mechanisms in a safe order. A pivot move of more than 5° first
pulls the elevator in below 0.15 m (RETRACTING), then swings the pivot (PIVOTING), then extends
(EXTENDING) until everything is within tolerance (AT_GOAL). The pivot, elevator and wrist targets
for each goal are in `src/main/include/subsystems/SuperstructureConstants.h` (placeholders: tune
them on the robot).

## Editing the state machines

```sh
node <Mayhem>/scripts/mayhem.mjs machine apply robot.mayhem.json tools/mayhem/machines.json
tools/mayhem/export.sh            # regenerate MayhemProject.h (MAYHEM_DIR=<Mayhem> to skip the clone)
```

`tools/mayhem/export.sh --check` fails if the header or vendored headers are stale (for CI).
Edits saved from the Mayhem app regenerate the header themselves.

## Simulation

`sim::PhysicsWorld` (`src/main/cpp/sim/PhysicsWorld.cpp`) runs Mayhem's physics engine. The
robot code commands its real Phoenix TalonFXs, and each loop the world feeds every motor's applied
request to Mayhem's TalonFX emulator (same configs, so Motion Magic, gains, limits and followers
run as on the motor controller). The pivot, elevator and wrist are coupled the way a pivotvator
is. Nothing runs on the wall clock, so a test or a sim proof scenario runs much faster than real
time.

- Our tests: `./gradlew build` (or the WPILib "Build Robot Code" command) runs
  `src/test/cpp/SuperstructureTest.cpp`.
- Sim proof: `node <Mayhem>/scripts/mayhem.mjs sim . HEAD --base main` (MCP `run_scenarios`), or
  Changes → Sim proof in the app. It taps and holds every button, runs every auto, and checks
  `tools/mayhem/scenarios.json`. Run it on a commit so the result is recorded for reviewers.
- Watch it live: `ROBOT_SIM_REALTIME=1` paces the harness at real time; or run the robot sim from
  WPILib VS Code and connect Mayhem's live view to localhost.

## 3D replay

Every sim log has the robot pose (`/RealOutputs/Sim/RobotPose`, parked: no drivetrain yet) and the
four moving parts (`/RealOutputs/Sim/Components`: fixed stage, 1st stage, carriage, wrist), in the
order of the CAD model's `config.json` (`ascope_assets/`). Mayhem's 3D replay reads the robot model
and the field model from AdvantageScope's asset folders:

- Robot: copy `Robot_Pivotvator2026/` (config.json + the five .glb files from the Onshape export,
  see `ascope_assets/README.md`) into `~/Library/Application Support/AdvantageScope/userAssets/`.
- Field: install AdvantageScope (it comes with the WPILib installer) and open it once, so it
  downloads the 2025 field.

## Logs

| Channel | Meaning |
| --- | --- |
| `Mayhem/Schema/<machine>` | Machine manifest |
| `Mayhem/States/<machine>/{state,transition,...}` | State entries, transitions, guard results |
| `RealOutputs/Mayhem/States/<machine>/inputs/*` | Every input each machine reads |
| `RealOutputs/Mayhem/Auto/{Status,Name}` | Auto runner (sim proof reads Status) |
| `RealOutputs/Mayhem/Build` | Which commit this robot code was built from |
| `RealOutputs/Superstructure/*` | Goal, motion phase and the three targets |
| `RealOutputs/Sim/*` | Simulated truth: robot pose, components, mechanism positions, battery |
