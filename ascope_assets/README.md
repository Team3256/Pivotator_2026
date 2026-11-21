# AdvantageScope 3D robot model

This folder holds our robot's CAD model for AdvantageScope's 3D Field view, so we can watch the
elevator move in sim (or replay a real match log).

```
ascope_assets/
  Robot_Pivotvator2026/   <- folder name MUST start with "Robot_"
    config.json           <- how AdvantageScope lines the pieces up (already written)
    model.glb             <- base: everything that never moves (drivebase, electronics,
                             pivot gearbox/towers bolted to the frame)
    model_0.glb           <- component 0: everything the pivot tilts that does NOT slide
                             (fixed elevator stage + Elevator GB, if it rides on the elevator)
    model_1.glb           <- component 1: elevator 1st stage
    model_2.glb           <- component 2: carriage (without the wrist)
    model_3.glb           <- component 3: wrist + end effector (EE Block CAD)
```

## How the pieces fit together

This robot is a **pivotvator**: a pivot at the base tilts the whole elevator, the elevator slides
along its own (tilted) axis, and a wrist turns on the carriage. So each moving part's pose stacks
on the one before it:

| Index | Part | Pose |
|---|---|---|
| 0 | Fixed elevator stage | rotate by the pivot angle about the pivot axle |
| 1 | 1st stage | pose 0, then slide **h / 2** along the elevator's axis |
| 2 | Carriage | pose 0, then slide **h** along the elevator's axis |
| 3 | Wrist + end effector | pose 2, then rotate by the wrist angle about the wrist axle |

h is the carriage height from `Elevator::GetPositionMeters()`. The 1st stage goes half as far
because the rigging is continuous. The robot code logs these as `RobotVisualizer/ComponentPoses` (measured) and `Sim/Components` (simulated; what Mayhem draws),
a `Pose3d[]` in exactly this order (see `src/main/cpp/sim/RobotVisualizer.cpp`).
The pivot and wrist parts of that math need three numbers from CAD: where the pivot axle is,
which direction the elevator slides, and where the wrist axle is.

- A logged pose of all zeros means "the part sits exactly where it is in CAD". So every .glb must
  be exported **from the same assembly, in the same coordinate frame, with the pivot, elevator
  and wrist all at their zero positions** (whatever the code calls 0: elevator fully down, and the
  pivot/wrist angles the code calls 0). Then the only thing `config.json` has to do is turn
  Onshape's frame into the robot's frame, and it's the same fix for every file.
- `rotations` (base) and `zeroedRotations` (components) turn the export into AdvantageScope's frame
  (X = robot front, Y = left, Z up). Onshape already exports Z-up, so only a `z: 90` is needed:
  in CAD the robot's front (the end-effector side) points toward -Y.
- `position` / `zeroedPosition` are 0 because the CAD origin is already the frame center on the
  floor.

## Export from Onshape (one export + a script)

1. Open the "00. Main Unconstrained" assembly. Leave the pivot, elevator and wrist where the CAD
   draws them (elevator flat and fully retracted).
2. Right-click the assembly's tab at the bottom of the screen -> **Export...** -> Format
   **GLTF** -> **Export**. Don't suppress anything. The file lands in Downloads.
3. Split it into the five files:

   ```bash
   python3 ascope_assets/split_onshape_glb.py "~/Downloads/00. Main Unconstrained.glb"
   ```

   The script decides which part goes in which file by its name (Stage 1 Rail -> 1st stage,
   Carriage Tube -> carriage, the "03. Wrist Assembly" -> wrist, ...). It drops bolts, nuts,
   spacers and the cable chain to keep the files small. If a new part lands in the wrong file,
   add a name rule in `classify()` / `elevator_bucket()` in the script.

## Load it in AdvantageScope

1. AdvantageScope -> **Help -> Use Custom Assets Folder** (on macOS it's in the app menu /
   Help menu) and pick this `ascope_assets` folder. (Or copy `Robot_Pivotvator2026` into the
   folder that **Help -> Show Assets Folder** opens.) Restart AdvantageScope if the robot doesn't
   show up in the list.
2. Start the sim: `./gradlew simulateJava` (or WPILib: Simulate Robot Code). In the sim GUI,
   set the robot to **Teleoperated**, and drag **Keyboard 0** from "System Joysticks" onto
   **Joysticks[0]**.
3. AdvantageScope -> **File -> Connect to Simulator**.
4. Add a **3D Field** tab. Pick a field (the coral game field) and in the robot dropdown pick
   **3256 Pivotvator 2026**.
5. From the sidebar, drag `AdvantageKit/RealOutputs/RobotVisualizer/RobotPose` onto the 3D
   Field's Poses area. It shows up as a "Robot".
6. Drag `AdvantageKit/RealOutputs/RobotVisualizer/ComponentPoses` **onto that robot** so it
   becomes a "Component" child of it.
7. Press keys in the sim GUI's Keyboard 0 window: **Z** = A = Stow, **X** = B = L2,
   **C** = X = L3, **V** = Y = L4. The elevator should rise and fall.

Also handy: `AdvantageKit/RealOutputs/Elevator/Mechanism2d` in a **Mechanism** tab (the 2D side
view), and a **Line Graph** of `Elevator/ReqPositionMeters` vs `Elevator/LeaderPositionMeters`.

## Calibration (do this once after exporting)

1. Instead of `ComponentPoses`, drag `RobotVisualizer/ZeroedComponentPoses` onto the robot.
   Everything should look like the CAD with the elevator down. If the stage or carriage floats
   somewhere else, the files weren't exported from the same assembly/frame.
2. **Lying on its side?** Change the `x` rotation (try `-90`, or `0`) in **every** rotation
   lists. They should always be the same, because all the files share one export frame.
3. **Facing the wrong way?** The robot's front should point along the field's +X (toward the
   red alliance wall at the default robot pose). Change the `z` rotation to `90`, `-90` or `180`,
   again in every list.
4. **Not centered / sunk into the floor?** Set `position` to the shift (meters) from the CAD origin
   to the frame center on the floor, **in robot axes** (X forward, Y left, Z up). Copy the same
   numbers into every `zeroedPosition`. Example: if the layout origin is 2 in above the floor at
   frame center, use `[0, 0, 0.0508]`. Check the "00. Layout and Block" sketch for where the
   origin really is.
5. Switch back to `ComponentPoses` and run L4. At L4 (0.89 m) the carriage should be near the top
   of the 1st stage, and the 1st stage near the top of the fixed stage.

Edits to `config.json` show up after restarting AdvantageScope (or re-picking the robot).
