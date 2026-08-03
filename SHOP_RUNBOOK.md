# Shop runbook — first robot session

Branch `StuartRevisions`. Nothing here has touched hardware. Work top to bottom; each step
gates the next.

Console output from every routine is prefixed, so you can follow along in the driver station
Console tab without AdvantageScope: `[validate]`, `[calib]`, `[maneuver]`, `[state]`.

---

## 0. Before power-on — two numbers only you can supply

Both live in `src/main/java/frc/robot/common/subsystems/vision/VisionConstants.java` and are
marked `MEASURE`. Vision cannot be trusted until they are right, and a wrong camera transform
produces *confidently wrong* poses, which is worse than none.

| Constant | What it is | How to get it |
|---|---|---|
| `CAMERA_NAME` | Camera name exactly as PhotonVision shows it | PhotonVision web UI. A mismatch fails **silently** — no error, vision just never contributes |
| `ROBOT_TO_CAMERA` | Lens position and angle relative to robot centre on the floor | Tape measure. +x forward, +y left, +z up. Pitch is negative when tilted **up** |

Field layout is already set to **AndyMark** (`k2026RebuiltAndymark`). If you ever run on a
welded field, change it — the two layouts place tags up to **3.6 cm** apart, which is larger
than your 1″ budget.

Also check `FieldRegions.BUMP_NEAR_EDGE_METERS` / `BUMP_FAR_EDGE_METERS` — currently a
placeholder mid-field band. The bump-reverse state fires off these.

---

## 1. Deploy

```
cd ~/FRC/FRC26-Rebuilt-worktrees/StuartRevisions
./gradlew deploy
```

Expect `startup complete` in the console. If the program dies at boot, read the stack trace —
a binding composition error kills it before teleop and is the most likely cause.

---

## 2. On blocks — mechanical self-test

**Robot on blocks, wheels clear.** Select **Test** mode.

Fourteen checks run automatically: gyro reporting, module positions readable, drive motors
actually turn the wheels, steering responds, shooter reaches its idle setpoint, intake deploy
moves its encoder, feeder and spindexer accept demands.

- Watch: `Rebuilt/Validation/AllPassed`
- A failed drive check names the offending corner and how far it actually travelled

**Do not proceed past a failure here.** This is the step that catches a swapped CAN ID or an
unplugged encoder before it becomes a broken mechanism.

---

## 3. On the floor — verify drive directions

**This is the likeliest surprise of the day.** The original code had no negations and a comment
reading *"LeftY is the xRequest and LeftX is the yRequest for some reason"*. Because the
drivetrain never actually moved, **there is no evidence any sign was correct.** Standard WPILib
convention has been applied; expect to flip some.

Enable teleop and check, gently, in this order:

1. Push stick forward → robot drives **forward**
2. Push stick left → robot drives **left**
3. Right stick left → robot rotates **counter-clockwise**

If any is inverted, flip the sign of the matching lambda in `RebuiltContainer.createContainer()`.

Then watch `Expectations/AllOK` while driving. `DriveRespondsToStick` trips if the sticks are
not reaching the modules.

---

## 4. Vision commissioning

Practice field, tags in view. Schedule `getVisionValidationCommand()`.

Five checks: camera connected, field layout loaded, a tag sighting accepted within 5 s, fused
pose agrees with wheel-only pose to within half a metre, latency under 200 ms.

- If `Rejected` is climbing fast, suspect `ROBOT_TO_CAMERA` or the field layout — the gates are
  probably throwing out poses that land off the field
- `SwerveDrive/VisionCorrectionMeters` shows how hard vision is pulling. Large and sustained
  means one of the two is wrong

While commissioning, `VISION_SUBSYSTEM.setFuseIntoPoseEstimate(false)` lets you gather every
calibration figure without vision moving the robot.

---

## 5. Drivetrain auto-calibration

**Needs ~4 m clear ahead and tags in view.** Schedule `getCalibrationCommand()`.

Aborts immediately with a clear message if no tag measurements exist — none of it means
anything without ground truth.

Runs in dependency order:

| Routine | Measures | Notes |
|---|---|---|
| Straight run, open loop | **Wheel scale**, **steering misalignment** | Open loop deliberately — closed loop would correct the error being measured |
| Spin in place | **Gyro scale**, **effective drive radius** | Several turns, so scale error beats tag noise |
| Stepped duty-cycle sweep | **kS**, **kV** | Watch `R²`; a poor fit means wheel slip or not enough run-up |
| 10 ft, open loop | Dead-reckoning accuracy | The honest odometry number |
| 10 ft, closed loop | Corrected accuracy | Should be substantially better |

Ends with a paste-ready report. **Nothing is written to source** — you paste what you accept.

### What to expect on your 1″ / 10 ft spec

From the datasheets, the budget is 0.833%, and two terms dominate:

| Source | Spec | Over 10 ft |
|---|---|---|
| Wheel diameter, uncalibrated | 1–3% tread compression | **30–91 mm (1.2–3.6″)** |
| Steering offset | ±0.5° (Through Bore V2) | **26.6 mm (1.05″)** |
| Gyro drift during run | 0.5°/min → 0.017° in 2 s | 0.9 mm |
| Drive encoder quantization | 7168 cpr × 4.714 | negligible |

**You cannot hit 1″ open loop with nominal constants** — wheel diameter alone blows it. And
±0.5° of steering misalignment can consume the whole budget by itself, which is why the
calibrator measures it.

Encoder quantization and gyro drift are nowhere near mattering. Do not spend time there.

**Closed loop changes the game.** Terminating on the tag-corrected pose removes wheel scale
from the endpoint entirely — it then only shapes the velocity profile. Expect closed loop to
pass comfortably while open loop may not. Accuracy then depends on the *vision* calibration
being right, which is why step 4 gates this one.

---

## 6. Manoeuvre suite

Space permitting, run in increasing order of appetite:

| Command | Contents | Space |
|---|---|---|
| `getPermutationManeuversCommand()` | 16 drive-turn-drive permutations: {10 ft, 5 ft} × {90°, 270°} × {left, right} × {fwd, rev} | Large |
| `getSamePathReturnCommand()` | 4 out-and-back retracing the outbound path | Moderate |
| `getDifferentPathReturnCommand()` | 5 loops home by a different route: squares both ways, rectangle, mixed turns, triangle | Large |
| `getAllManeuversCommand()` | All 25, ~100 m of driving | Very large |

Each is scored against its analytically expected finishing pose, decomposed into along-track,
cross-track and heading so the *cause* is identifiable, not just the magnitude. Loops also
report **closure error**, which needs no absolute reference at all.

**Read the two families differently.** A wheel-scale error largely *cancels* on a same-path
return — both legs are wrong by the same proportion in opposite directions — so small closure
error there does **not** mean the wheels are calibrated. It isolates heading error, backlash and
hysteresis. The different-path loops are where scale and heading compound instead, and they are
the closest analogue to a real autonomous path.

Summary prints worst-error-per-metre first. Compare the left and right squares: turn error is
often direction-dependent.

---

## 7. Localization states

Two assists, both supplying **heading and shooter speed only** — translation always stays with
the driver, and touching the rotation stick overrides instantly.

- **`AIM_AT_HUB`** — own half, hub open: faces the hub and sets the flywheel from a
  distance-interpolated curve built from your own four tuned presets. Outside 2.0–6.1 m it
  clamps rather than extrapolating.
- **`BUMP_REVERSE`** — approaching or on the bump: turns so the back crosses first, committing
  ~1 m early because turning halfway up is worse than not turning.

Bump outranks aiming. Both fall back to `MANUAL` when the pose is not trustworthy.

Watch `States/Active` and `States/Reason`. Verify the bump band matches the real field before
trusting it.

---

## 8. Tuning session

Set `TunableNumber.TUNING_ENABLED = true`, redeploy. Shooter `kP`/`kI`/`kD` appear under
`Tuning/` and reconfigure only on change (no flash wear, no CAN flood).

Highest-value targets, in order:

1. **PathPlanner gains** — translation `P` is 14.0 and the rotation constants are byte-identical
   to the module steering PID, which is a copy-paste signature. More importantly, until this
   branch every path ran flat out regardless of profile, so **these have never been evaluated
   against correct behaviour.** Re-tune from scratch.
2. **Shooter PID** — spin-up and recovery dominate cycle time; effect visible within a second on
   `Shooter/Activity/RPMError`.
3. **Max speed** — currently capped at 4.8 m/s, which coincidentally matched the *wrong* motor's
   free speed. Physical capability with NEO Vortex is ~5.74 m/s, so ~19% is unused. Left alone
   deliberately: that is a driveability call.

**Turn tuning off before competition.**

---

## Known-unverified list

Everything below is reasoned or measured in simulation, never on hardware:

- Drive direction signs (step 3)
- Camera transform and camera name (step 0)
- Bump band position (`FieldRegions`)
- Module angular offsets — never measured, and worth ±1″ over 10 ft on their own
- PathPlanner gains — never run against correct velocities
- Whether the intake deploy soft limits (0–11 rotations) match real travel
- All calibration figures — the routines are tested against synthetic data with known answers,
  but have never seen a real robot
