# Shop runbook — what to do with physical robot access

Branch `StuartRevisions`. **Nothing here has touched hardware.** Work top to bottom; each step
gates the next, and step 2 is a hard stop if it fails.

Console output from every routine is prefixed, so you can follow along in the driver station
Console tab without AdvantageScope: `[validate]`, `[calib]`, `[maneuver]`, `[state]`.

Background, decisions and history: `passdowns/2026-08-04_claude_frc26-rebuilt-hardening.md`.

---

## Session plan at a glance

| Step | What | Needs | Rough time |
| --- | --- | --- | --- |
| 0 | Measure and enter three values | Tape measure, PhotonVision UI | 20 min |
| 1 | Deploy | Laptop | 5 min |
| 2 | **On blocks** — 14-check self-test | Blocks, wheels clear | 10 min |
| 3 | **Verify drive directions** | ~3 m clear floor | 10 min |
| 4 | Vision commissioning | Practice field, tags visible | 20 min |
| 5 | Auto-calibration | ~4 m clear, tags visible | 20 min |
| 6 | Manoeuvre suite | Large clear space | 20–60 min |
| 7 | Localisation states | Practice field | 20 min |
| 8 | **Traction / drive current limit** | Wall, carpet, good battery | 10 min |
| 9 | **Load thresholds** — piece and jam detection | Blocks, ~20 game pieces, a helper | 15 min |
| 10 | **SysId feedforward** — the only source of kA | 28 ft of carpet, robot at one end | 10 min |
| 11 | Tuning | Time and patience | open-ended |

Steps 8 and 9 are independent of everything above them. If the field is busy, do them
first — step 8 needs only a wall and step 9 only blocks.

### Bring to the shop

- **Tape measure** — module spacing (decision 1), camera position, bump band edges
- **Scale** if you want to re-check the 47.6272 kg in `settings.json`
- **Blocks** — steps 2 and 9 will not be skipped
- **~20 game pieces** for step 9, and **a second person** to feed them
- **A good battery** for step 8 — a sagging pack under-reports the traction limit
- **The full 28 ft of carpet clear** for step 10, robot starting at one end
- **A notebook**, or willingness to pull the WPILOG off the USB stick afterwards

### Data to capture before you leave

The calibrators print paste-ready blocks; keep all of them. Also worth recording:
`Calibration/Auto/WheelScale`, `Calibration/Auto/SteerOffsetDeg`,
`Calibration/Auto/ModuleKvSpreadPercent`, both `Acceptance/*/TotalErrorMm` figures,
`Calibration/Maneuvers/Summary/*`, `TractionCalibration/*`, `LoadCalibration/*/*`,
`SysId/Mean/*` and `SysId/WorstRunMeters`.

**`SysId/WorstRunMeters` is worth a note of its own.** It says how much of your 28 ft each run
actually needed. If it comes back near the 6.0 m abort, the drivetrain is faster than the nominal
constants predict and the ramp wants shortening before the next session.

**Also grab `Shooter/Sensors/AnalogRPM`.** It should read 0 — that channel was being used as
the flywheel's velocity source and there is no analog sensor on either shooter motor. If it
reads anything but 0, someone fitted one and that needs knowing.

---

## 0. Before power-on — three values only you can supply

Both live in `src/main/java/frc/robot/common/subsystems/vision/VisionConstants.java` and are
marked `MEASURE`. Vision cannot be trusted until they are right, and a wrong camera transform
produces *confidently wrong* poses, which is worse than none.

| Constant | What it is | How to get it |
| --- | --- | --- |
| `CAMERA_NAME` | Camera name exactly as PhotonVision shows it | PhotonVision web UI. A mismatch fails **silently** — no error, vision just never contributes |
| `ROBOT_TO_CAMERA` | Lens position and angle relative to robot centre on the floor | Tape measure. +x forward, +y left, +z up. Pitch is negative when tilted **up** |
| Module spacing | Distance between module centres, both axes | Tape measure the frame. See decision below — the code and PathPlanner currently disagree by 6.5 mm per side |

**Measuring `ROBOT_TO_CAMERA` properly is worth the ten minutes.** Measure from the robot's centre
of rotation, at floor level, to the camera *lens* — not the case. Pitch matters most: a few degrees
of error becomes tens of centimetres of pose error at the far end of the field, and it will look
like a calibration problem rather than a measurement one.

Field layout is already set to **AndyMark** (`k2026RebuiltAndymark`). If you ever run on a
welded field, change it — the two layouts place tags up to **3.6 cm** apart, which is larger
than your 1″ budget.

Also check `FieldRegions.BUMP_NEAR_EDGE_METERS` / `BUMP_FAR_EDGE_METERS` — currently a
placeholder mid-field band. The bump-reverse state fires off these.

### Two decisions: PathPlanner settings disagree with the code

PathPlanner *plans* using `src/main/deploy/pathplanner/settings.json`; the robot *executes* using
`CommonConstants`. Two properties disagree, so PathPlanner is planning for a slightly different
robot than the one that runs. Both are pinned by `PathPlannerSettingsConsistencyTest` so they
cannot grow, but both need a human decision:

| Property | PathPlanner | Code | Decision |
| --- | --- | --- | --- |
| Module offset | ±0.343 m (27.01″ spacing) | ±0.3366 m (26.50″) | **Measure the frame.** 6.5 mm per side skews the kinematics, so commanded rotation and translation bleed into each other |
| Drive current limit | 60 A | 50 A applied in `Configs.java` | **Electrical call.** PathPlanner plans acceleration assuming 20% more torque than the drivetrain will deliver, so the robot falls behind on hard acceleration |

Whichever value is right in each row, change the other to match. Once they agree, tighten the two
`KNOWN_*_DIVERGENCE` constants in that test to zero — a test in the suite will tell you to.

Two things that *do* agree and are worth knowing: wheel radius (0.038 vs 0.0381, a 0.26% difference
that does not matter) and the motor type, recorded as `vortex`, which independently confirms the
drive free-speed fix.

Also note `settings.json` already carries **measured** mass (47.6272 kg) and MOI (3.733 kg·m²).
Those are now used by the code fallback rather than guessed at.

---

## 1. Deploy

```bash
cd ~/FRC/FRC26-Rebuilt-worktrees/StuartRevisions
./gradlew deploy
```

Expect `startup complete` in the console.

Boot-killing wiring faults are now caught by `ContainerWiringTest` rather than only by the
simulator, and that test is proven to catch them — reintroducing the original illegal composition
makes it fail with the exact WPILib message. The PathPlanner half is covered too, so the full
`createContainer()` path now runs in tests.

A missing or malformed `settings.json` no longer stops the robot booting. It used to throw a
`RuntimeException` out of `robotInit()` *and* discard the original exception, so the console said
only that loading had failed. It now prints the real cause and falls back to `PathPlannerConfig`.
Check `RobotUtils.isUsingFallbackConfig()` if paths follow unexpectedly badly — the fallback works
but is less accurate than the tuned settings.

Running `./gradlew simulateJava` before a deploy is still worth it as a whole-program smoke test,
but it is no longer the *only* thing covering container wiring.

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

### If a self-test check fails

| Check | Most likely cause |
| --- | --- |
| `DriveMotorsTurnWheels` — names the corner and how far it moved | Swapped CAN ID, unplugged encoder, or a motor wired backwards on that corner |
| `SteeringResponds` | Through Bore encoder unplugged, or a steering CAN ID swapped |
| `ShooterSpinsUp` — prints target vs measured | Follower fighting the leader (check `follow()` inversion), or the interlock reporting the hub closed |
| `IntakeDeployMoves` — prints rotations travelled | Deploy motor not moving, encoder not counting, or the soft limits are stopping it early |
| `GyroReporting` | NavX not seated on the MXP port |

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

If any is inverted, flip the sign of the matching lambda in
`RebuiltContainer.setDriveDefaultCommand()`.

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

> **This step cannot produce kA.** Its feedforward sweep waits for steady state, where acceleration
> is zero, so the data contains no information about it. Step 10 is the only source of kA. Both fit
> kS and kV, and comparing them is a real cross-check — see step 10.

**Needs ~4 m clear ahead and tags in view.** Schedule `getCalibrationCommand()`.

Aborts immediately with a clear message if no tag measurements exist — none of it means
anything without ground truth.

Runs in dependency order:

| Routine | Measures | Notes |
| --- | --- | --- |
| Straight run, open loop | **Wheel scale**, **steering misalignment** | Open loop deliberately — closed loop would correct the error being measured |
| Spin in place | **Gyro scale**, **effective drive radius** | Several turns, so scale error beats tag noise |
| Stepped duty-cycle sweep | **kS**, **kV** — *per module* | Watch `R²`; a poor fit means wheel slip or not enough run-up |
| 10 ft, open loop | Dead-reckoning accuracy | The honest odometry number |
| 10 ft, closed loop | Corrected accuracy | Should be substantially better |

Ends with a paste-ready report. **Nothing is written to source** — you paste what you accept.

### What to expect on your 1″ / 10 ft spec

From the datasheets, the budget is 0.833%, and two terms dominate:

| Source | Spec | Over 10 ft |
| --- | --- | --- |
| Wheel diameter, uncalibrated | 1–3% tread compression | **30–91 mm (1.2–3.6″)** |
| Steering offset | ±0.5° (Through Bore V2) | **26.6 mm (1.05″)** |
| Gyro drift during run | 0.5°/min → 0.017° in 2 s | 0.9 mm |
| Drive encoder quantization | 7168 cpr × 4.714 | negligible |

**You cannot hit 1″ open loop with nominal constants** — wheel diameter alone blows it. And
±0.5° of steering misalignment can consume the whole budget by itself, which is why the
calibrator measures it.

Encoder quantization and gyro drift are nowhere near mattering. Do not spend time there.

### Per-module motor variance

Published motor specs are *typical* values — REV's own 5676 RPM is an empirical average — and
individual motors vary. So the sweep fits **each module separately** rather than averaging
first, which would produce one tidy number and hide the useful one.

Read `Calibration/Auto/ModuleKvSpreadPercent`:

- **Under ~8%** — normal manufacturing variation, absorbed by the velocity loop. One robot-wide
  kV is fine.
- **Over ~8%** — one corner is materially weaker. The report names it. Check gearing, wheel
  wear, and whether that module has a different motor than you think before accepting an
  averaged kV.

This matters for the 1″ spec directly: a weak corner pulls the robot off a straight line, which
shows up as cross-track error rather than distance error.

**Closed loop changes the game.** Terminating on the tag-corrected pose removes wheel scale
from the endpoint entirely — it then only shapes the velocity profile. Expect closed loop to
pass comfortably while open loop may not. Accuracy then depends on the *vision* calibration
being right, which is why step 4 gates this one.

---

## 6. Manoeuvre suite

Space permitting, run in increasing order of appetite:

| Command | Contents | Space |
| --- | --- | --- |
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

## 7. Localisation states

Practice field, tags visible, alliance set on the driver station.

**`AIM_AT_HUB`** — drive onto your own half with the hub open. The robot should turn to face the hub
and spin the flywheel to a range-appropriate speed. Watch `States/Active`, `States/Reason` and
`States/DistanceToHub`. Touch the rotation stick: the assist must yield instantly.

**`BUMP_REVERSE`** — this one needs the bump band measured first, or it will fire in the wrong place.
Drive towards the bump; the robot should turn so its back leads, committing about a metre out.
`States/Active` should read `BUMP_REVERSE` and `States/OnBump` should go true as you cross.

Both fall back to `MANUAL` if the pose is untrustworthy, so if neither ever engages, check
`Vision/SecondsSinceAccepted` and `Field/HasAlliance` before suspecting the states themselves.

---

## 8. Traction — the drive current limit

**Robot square against a wall, on carpet, on a good battery.** Schedule
`RebuiltContainer.getTractionCalibrationCommand()`.

It pushes at full output while stepping the drive current limit from 20 A upward in 5 A steps,
stopping the moment the wheels break loose. Each step is 0.75 s of pushing with 2.5 s of
cooldown, so about a minute in total.

**What it is measuring.** Against a wall the robot cannot move, so the wheels either grip and
stay still or slip and spin. Wheels turning while the chassis stays put is slip. That second
half matters: without it, driving away from the wall would read as the most convincing slip in
the sweep, which is why the run **aborts and reports no number** if the pose moves more than
10 cm.

**Why the limit belongs below the traction limit.** Not only to protect motors. Below it the
wheels physically cannot slip under their own torque, so encoder distance always corresponds to
ground travelled — which is the same error budget as the 1″ / 10 ft spec in step 5, arriving
through the throttle instead of through the wheel diameter. Above it, hard acceleration spins
the wheels and the pose estimate gains error nothing knows to expect.

Paste the recommendation into `SwerveConstants.DRIVE_MOTOR_CURRENT_LIMIT`. The limits applied
during the run **do not persist** — a power cycle restores whatever the code says, so an
interrupted run cannot leave the robot on a limit nobody chose.

### Reading the report

| Line says | Meaning |
| --- | --- |
| `gripped` | Limit was binding, wheels held. Keep going |
| `SLIPPED` | Traction limit found. Sweep stops here |
| `gripped, but limit not binding` | Current never reached the limit that was set, so the limit is not what held it down. **Check the battery** |
| `INVALID, robot moved …` | Not against the wall. Reposition and re-run |

**Watch the breaker warning.** The recommendation is *per motor* and there are four of them.
70 A per motor is 280 A against a 120 A main breaker. The breaker is thermal so short pushes
survive, but it is not a limit the drivetrain can hold — if the robot browns out in pushing
matches, this is the first number to lower.

**Expect a low-ish number.** On carpet with 3″ wheels and this robot's weight, traction is
likely to break somewhere in the 35–55 A range. If it never slips up to 80 A, the report says
so and recommends the cap.

---

## 9. Load thresholds — piece and jam detection

**Robot on blocks, ~20 game pieces, and a second person.** Schedule
`RebuiltContainer.getLoadCalibrationCommand()` for all four mechanisms, or
`getIntakeLoadCalibrationCommand()` for just the intake.

Every value in `RebuiltConstants.LoadConstants` is currently reasoned, not measured. This
replaces them with numbers off the actual robot. Each mechanism runs three phases, announced on
the console as they start:

| Phase | Duration | What you do |
| --- | --- | --- |
| Empty | 4 s | Nothing. Keep hands and pieces clear |
| Loaded | 8 s | **Feed pieces through continuously.** Gaps are fine and expected |
| Obstructed | 2 s | Hold the mechanism so it cannot move product |

Order is intake → spindexer → feeder → shooter, which is the order pieces travel, so a piece
fed for one phase is roughly where it needs to be for the next.

**The shooter has no obstructed phase.** Obstructing a flywheel by hand is how people lose
fingers, and a wheel carrying that much momentum will throw or shear whatever holds it. Its jam
threshold is inferred from the loaded phase instead, and the report says `INFERRED` rather than
implying it was measured.

### Reading the report

Each mechanism gets one line. The verdicts that matter:

- **A pasteable line** — `WORK_EXCESS_AMPS = …; EXPECTED_RPM = …; jam fraction …`. Paste all
  three into `LoadConstants`.
- **`NOT VIABLE`** — a piece does not load that mechanism enough to see in current. **Do not
  paste anything.** No threshold exists that separates loaded from empty, and one invented
  anyway would misfire all match. This is a real possibility for a lightly-loaded roller, and
  it is much better to learn it here.
- **`INCOMPLETE`** — not enough samples, usually because too few pieces went through. Re-run
  that mechanism and feed harder.

The report also states whether each jam threshold was **measured** (obstructed phase run,
threshold sits between two measured populations) or **inferred** (derived only from how far a
working mechanism slows, which says nothing about how far a stuck one does).

### While you are there

`JamClearing` automates the jostling that is currently done by hand — Back clears the whole
path, Start jostles the intake. Both are bounded and escalating: three attempts at increasing
amplitude, then it gives up rather than pumping a mechanism that is not going to free. Try each
with a piece deliberately wedged and confirm it stops on its own.

---

## 10. SysId — the feedforward, including kA

**Start at one end of the carpet, facing down its length.** Schedule
`RebuiltContainer.getSysIdCommand()`.

Runs a quasistatic ramp forward and reverse, then four short alternating voltage steps, then
prints kS, kV and kA per module plus the mean. **No log transfer and no desktop analyser** — the
regression the SysId GUI performs is done on the robot.

### Why this exists when step 5 already fits a feedforward

Step 5's sweep waits for steady state, where acceleration is zero. So kA is not merely unmeasured
there, it is **unmeasurable** — the data contains no information about it. kA is what
second-order kinematics needs.

Run both and compare: kS and kV agreeing between two different excitations and two different
regressions is real evidence. Disagreeing means one run was bad, which is much better learned
from two printed numbers than from a robot that follows paths oddly.

### It is sized for your 28 ft

Half a field is 8.53 m. **The textbook SysId ramp — 1 V/s for 6 s — covers 8.10 m on this
drivetrain**, which is the whole carpet before allowing for the robot's length or stopping
distance. So the stock configuration would drive into the wall on its first test.

The configuration here reaches 5.25 V in about **4.1 m** by ramping faster (1.5 V/s for 3.5 s).
That works because for a given final voltage, distance is inversely proportional to ramp rate —
and a faster ramp is only a problem for the classical two-stage analysis, which assumes the ramp
has no acceleration. The on-robot fit solves for kA at the same time, so that acceleration is
signal rather than contamination.

The dynamic tests are four short alternating steps of about 0.9 m each rather than one long one.
All the kA information is in the first few time constants (~0.16 s here), so a long step just
drives; alternating short ones capture four transients and end up where they started.

**Every test also aborts at 6.0 m regardless of the clock.** The distance predictions above use
the nominal kV, and measured kV is what this routine produces — so the prediction is circular and
the abort is the guard that does not depend on it. A cut-short run still contributes its samples;
the report says which were cut.

### Reading the report

| Line says | Meaning |
| --- | --- |
| `kS = … kV = … kA = … (R2 …) — OK` | Paste the MEAN line into `CommonConstants.DriveFeedforwardConstants` |
| `SINGULAR` | The dynamic steps did not run, so acceleration was constant and kS and kA are not separable at all |
| `REJECT: kV is not positive` | A wiring or inversion fault, not a fit problem. No amount of extra data fixes it |
| `REJECT: kA is negative` | Unphysical — check that run for wheel slip or a collision |
| `SUSPECT: R2 … below 0.95` | Go and look at the log. The standard SysId log is still written, so the desktop analyser and its residual plots are available |

Also check **kA spread across modules**. Over 25% means one corner accelerates differently from
the other three, and a chassis feedforward built on the mean will under-drive it — which shows up
as the robot yawing under hard acceleration rather than as anything obviously feedforward-related.

---

## 11. Tuning

Set `TunableNumber.TUNING_ENABLED = true`, redeploy. Shooter `kP`/`kI`/`kD` appear under `Tuning/`
and reconfigure only on change — no flash wear, no CAN flood.

Highest-value targets, in order:

1. **PathPlanner gains.** Translation `P` is 14.0, and the rotation constants are byte-identical to
   the module steering PID — a copy-paste signature. More importantly, until this branch every path
   ran flat out regardless of profile, so **these have never been evaluated against correct
   behaviour.** Re-tune from scratch.
2. **Shooter PID.** Spin-up and recovery dominate cycle time; effect visible within a second on
   `Shooter/Activity/RPMError`.
3. **Vision standard deviations.** Replace the guessed `SINGLE_TAG_XY_STD_DEV_BASE` with the
   measured `Calibration/VisionNoise/MeasuredXyStdDevMeters` from a stationary run.
4. **Wheel diameter.** Multiply `kWheelDiameterMeters` by `Calibration/Auto/WheelScale`. Remember to
   update `driveWheelRadius` in `settings.json` to match.
5. **Drive feedforward.** Paste kS/kV/kA from step 10 into `DriveFeedforwardConstants`. Note that
   `Configs.java` currently derives its velocity feedforward from free speed as
   `12 V / kDriveWheelFreeSpeedRps` — a theoretical kV, correct only for a datasheet-perfect motor
   on a fresh battery. A measured kV should replace that derivation.

**kA has nowhere it is used yet**, and that is deliberate rather than an oversight. It is what
second-order kinematics needs, so it is measured and stored first; wiring it into a controller is a
separate change that should be made against a measured number rather than a guessed one.

**Turn tuning off before competition.**

---

## Before you pack up

- [ ] Paste-ready calibration reports saved somewhere — drivetrain, traction, and load
- [ ] `LoadConstants` updated, or the NOT VIABLE mechanisms noted as such
- [ ] `SwerveConstants.DRIVE_MOTOR_CURRENT_LIMIT` updated from the traction sweep
- [ ] `DriveFeedforwardConstants` kS/kV/kA pasted from the SysId report
- [ ] SysId kS/kV cross-checked against the auto-calibrator's figures
- [ ] `Shooter/Sensors/AnalogRPM` confirmed reading 0
- [ ] Decisions 1 and 2 resolved, or at least measured
- [ ] `settings.json` and `CommonConstants` reconciled — then tighten the two
      `KNOWN_*_DIVERGENCE` constants in `PathPlannerSettingsConsistencyTest` to zero
- [ ] Bump band measured and entered in `FieldRegions`
- [ ] Motor inventory spot-checked against step 9's measured free speeds — ~6,500 for the four
      Vortex mechanisms, ~5,400 for the spindexer. A mechanism near the wrong figure means the
      controller-to-motor rule has an exception nobody has mentioned
- [ ] `TunableNumber.TUNING_ENABLED` back to `false`
- [ ] `./gradlew build` still green after any constant changes
- [ ] Update the known-unverified list below — cross off what you verified

---

## Known-unverified list

Everything below is reasoned or measured in simulation, never on hardware. **Cross items off as you
verify them** — this list is the honest measure of how much of this code has met a robot.

- Drive direction signs (step 3)
- Camera transform and camera name (step 0)
- Bump band position (`FieldRegions`)
- Module angular offsets — never measured, and worth ±1″ over 10 ft on their own
- Module spacing — PathPlanner says 27.01″, code says 26.50″; measure the frame (section 0)
- Drive current limit — PathPlanner assumes 60 A, code applies 50 A; pick one (section 0)
- PathPlanner gains — never run against correct velocities
- Whether the intake deploy soft limits (0–11 rotations) match real travel
- All calibration figures — the routines are tested against synthetic data with known answers,
  but have never seen a real robot
- Every value in `LoadConstants` — reasoned, not measured (step 9)
- Whether a game piece is visible in current at all on each mechanism. It is assumed, and step 9
  is what proves or disproves it per mechanism
- The traction limit, and therefore whether the applied 50 A can already break traction (step 8)
- The motor inventory — derived from the controller-to-motor rule rather than read off labels. Controller types are read from code and
  certain, the motors behind them are not
- That `Shooter/Sensors/AnalogRPM` reads 0. The flywheel's velocity source was reading an analog
  sensor that is almost certainly not fitted; it now reads the encoder, but the old channel is
  still logged so this can be confirmed rather than assumed
- **kS, kV and kA** — the SysId routine and its regression are tested against synthetic data with
  known answers, but no real drivetrain has been characterised (step 10)
- **Whether the SysId runs actually fit in 28 ft.** The distances are computed from the nominal kV,
  and measured kV is what step 10 produces, so the prediction is circular. A 6.0 m abort covers it,
  but the first run is the one that finds out
- Whether the auto-calibrator's kV and SysId's kV agree. They use different excitations and
  different regressions, so agreement is evidence and disagreement means one run was bad

---

## Simulation: no drivetrain physics

Worth knowing before you rely on the simulator: **the robot cannot be driven in simulation.**
Commanding a module does nothing, encoders stay at zero, and the closed loops are never exercised.

That is not a misconfiguration and there is no setting for it. WPILib simulation models nothing
unless you write the model — physics is opt-in code, per
[the WPILib physics-simulation docs](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html).
So `simulateJava` validates wiring, bindings, container construction and telemetry, but not motion.

An attempt to add it was reverted. REVLib does ship the pieces (`SparkFlexSim`, `SparkMaxSim`,
`DCMotorSim`, and the NavX exposes a sim device for yaw), but three iterations produced velocities
49x, 10x and then roughly half the correct value, plus a gyro reading that disagreed with the model
in both sign and magnitude. A physics model that looks plausible and is wrong is worse than none —
a calibration dry-run against it would "measure" a wheel scale of about 0.44 and send someone
chasing a mechanical fault that does not exist.

`RevSimUnitContractTest` records what was established by measurement, so a future attempt starts
from fact rather than from method names:

- Encoder sim setters take **converted units** (m/s and metres here), not RPM
- `SparkSim.iterate(velocity, vbus, dt)` also takes **converted units**, not motor RPM — position
  advances by exactly `velocity * dt`. This was the main error: `DCMotorSim` reports RPM, and
  passing that straight through is wrong by the conversion factor
- Velocity readback is **stateful and filtered** — observed at 30.6 and 635.6 after a single
  `iterate(100)` depending on prior state. Trust position, not immediate velocity
- Conversion factors round-trip only to float precision, about 1.2e-9 of error
- Correct target for a working model: full throttle should approach **5.74 m/s**, matching
  `kDriveWheelFreeSpeedRps`

Still outstanding for a working model: the NavX sim yaw sign and scale, and the velocity filter's
transient. If drivetrain sim becomes a priority, `maple-sim` is purpose-built for swerve and worth
evaluating against finishing this by hand.