# Passdown — FRC26-Rebuilt hardening, calibration and localisation states

**Date:** 2026-08-04
**Author:** Claude (with Stuart Rampy)
**Branch:** `StuartRevisions` @ `840b2c5` — 14 commits off `main` (`23bf8b1`)
**Worktree:** `~/FRC/FRC26-Rebuilt-worktrees/StuartRevisions`
**Status:** Not pushed. `main` untouched. Nothing has run on hardware.

---

## One-paragraph summary

Two code reviews of `main` found 25 defects, including three that made the robot
non-functional: teleop drive was completely dead, AdvantageKit was never started so all
telemetry was discarded, and every autonomous path ran at full throttle regardless of its
velocity profile. All 25 are fixed. On top of that: AprilTag localisation is wired up, an
AprilTag-driven drivetrain auto-calibrator was built around the 10 ft / 1 inch accuracy
requirement, a 25-manoeuvre validation catalogue was added, and two localisation-driven robot
states were implemented. Then game piece and jam detection from motor current, with calibration
routines for both those thresholds and for the drive current limit. Test count went from 3 to
365. **No part of it has touched a robot** — that is tomorrow's job, and `SHOP_RUNBOOK.md` is the
script for it.

| | |
| --- | --- |
| Commits | 14 |
| Files changed | 78 (+15,790 / −659) |
| Source | 70 files |
| Tests | 29 files / **365 tests, 0 failing** |
| Build | clean, zero warnings |
| Simulation | reaches `startup complete` |
| Hardware verification | **none** |

---

## The three defects that mattered most

**Teleop drive was dead.** `driveCommand` took `double` rather than `DoubleSupplier`, and was
called from a static initialiser. The lambda captured the stick values once, at class-load time —
zero — and replayed them for the whole match. Nothing in `teleopPeriodic` drove the chassis
either, so there was no second path that saved it. The robot could not be driven at all.

**AdvantageKit was never started.** A dozen `Logger.recordOutput` calls, no `Logger.start()`, no
data receiver. Every subsystem faithfully logged into a void. AdvantageScope showed nothing, and
there were no on-robot logs — which is precisely the tool that would have exposed the drive bug in
minutes.

**All autonomous ran flat out.** `driveRobotRelative` passed metres per second into a method
expecting fractions of maximum speed, multiplying every PathPlanner request by 4.8 before
desaturation clamped it to maximum. Every path's velocity profile was discarded. Related:
`getChassisSpeeds()` called `toChassisSpeeds()` with no module states, so PathPlanner's velocity
feedback never described the robot.

The rest — string identity on the FMS message, integer division in the feedforward, a 0.6
deadband that was never applied, `stop()` methods that didn't stop, commented-out soft limits,
duplicate CAN ID declarations — are all in the artifact and the commit messages.

### A fourth, found later while wiring up shooter current

**The flywheel's velocity source was a sensor that isn't installed.**
`Shooter.getMeasuredRPM()` read `motor1.getInputs().analogVelocity`. In PurpleLib that field is
populated from `SparkBase.getAnalog().getVelocity()` — the analog sensor on the SPARK's **data
port**, not the motor encoder. There is no analog sensor on either shooter motor, so on the real
robot it returned 0.

The flywheel still span correctly: the SPARK runs its velocity loop against its own primary
encoder internally and never consults that field. What broke was everything built on top of it.
`isAtTargetRPM()` compared the target against 0, so it could never be true, and any
wait-until-up-to-speed gate would have hung forever. `Shooter/Activity/RPMError` logged the full
setpoint as error for the whole match.

Now reads `encoderVelocity`. **Both fields are still logged** under `Shooter/Sensors/` so the
shop session can confirm the analog channel really is dead rather than take this write-up's word
for it — the claim rests on reading decompiled library bytecode, not on measuring the robot.

---

## The single most consequential find

**The drive feedforward was characterised for the wrong motor.**

`kDrivingMotorFreeSpeedRps` read `5676 / 60`. That is the free speed of the **NEO 2.0 and NEO 1.1**
(both 5676 RPM), and it is WPILib's MAXSwerve template default. This robot drives on **NEO Vortex
at 6784 RPM**.

What makes it insidious rather than obvious: **the robot does have NEO 2.0s on it** — just not on
the drive shaft. So 5676 is a real figure for a real motor here, and nothing about it looks wrong
on inspection. It was only wrong because it named the wrong motor for the drivetrain.

| | Old | Corrected |
| --- | --- | --- |
| Free speed | 94.6 rev/s | **113.07 rev/s** |
| Wheel free speed | 4.804 m/s | **5.741 m/s** |
| Feedforward `kV` | 2.498 | **2.090 V/(m/s)** |

`kV` was **19.5% too high**, so the feedforward over-commanded voltage on every request and the
closed loop spent the match fighting it. Also: the 4.8 m/s cap coincidentally matched the *wrong*
motor's free speed, so roughly 19% of top speed is unused. **The cap was deliberately left alone** —
raising it is a driveability decision, not a correctness one.

There is now a motor inventory beside the CAN IDs in `RebuiltConstants` so this cannot recur. Four
superstructure motors are marked `CONFIRM` rather than guessed.

The team rule is that **a SPARK Flex always has a NEO Vortex on it** — nothing else is ever
connected to one. Since controller types are read from code and certain, that fixes the motor for
the intake rollers (CAN 13/15) too.

**CAN 18 is an unresolved conflict, deliberately left that way.** It was stated to be a NEO 2.0,
and it is a SPARK Flex, so the rule says Vortex. Both cannot be true. The candidates are 5676 and
6784 RPM — 19.5% apart, the same magnitude and the same failure mode as the bug this inventory
exists to prevent. Picking one to finish the table is how it happened the first time. The load
calibration measures unloaded speed directly, so step 9 settles it as a side effect.

---

## The 10 ft / 1 inch requirement

1 inch over 120 inches is 0.833%, or 0.477° of heading error. From the datasheets:

| Error source | Datasheet | Over 10 ft |
| --- | --- | --- |
| **Wheel diameter, uncalibrated** | 1–3% tread compression | **30–91 mm** |
| **Steering misalignment** | ±0.5° (Through Bore V2) | **26.6 mm** |
| Gyro drift during run | 0.5°/min → 0.017° in ~2 s | 0.9 mm |
| Drive encoder quantisation | 7168 cpr × 4.714 | 0.007 mm/count |

Two conclusions that shaped everything:

1. **The requirement is unreachable open-loop with nominal constants.** Wheel diameter alone
   exceeds the budget.
2. **±0.5° of steering misalignment can consume the entire budget by itself.** That was not in the
   original plan; doing the budget is what surfaced it, and it is why the calibrator measures
   common-mode steering offset.

Encoder resolution and gyro drift are nowhere near mattering. Do not spend time there.

**Closing the loop changes the picture.** Terminating on the AprilTag-corrected pose removes wheel
scale from the endpoint entirely — it then only shapes the velocity profile. Heading is held on the
gyro (fast, 0.02° of drift in a 2 s run), cross-track is corrected from the fused pose, and distance
remaining is measured absolutely. Expect closed-loop to pass comfortably where open-loop may not.

The honest caveat: this trades odometry error for *vision calibration* error. A wrong camera
transform puts the robot precisely on a wrong target.

---

## What was built

**Vision.** `VisionSubsystem` runs a PhotonVision multi-tag solve with single-tag fallback, feeding
the pose estimator with per-measurement standard deviations scaled by distance² and tag count.
Implausible poses are rejected rather than fused: off-field, off-floor, too distant, too ambiguous,
or an implausible single-tag jump. No-op with no camera connected. Note `AssumedPoseSubsystem`
already contained a complete unused AprilTag estimator — it was superseded and deleted.

**Field layout is AndyMark** (`k2026RebuiltAndymark`), pinned by test. The welded and AndyMark
layouts place tags up to **3.6 cm** apart, measured — larger than the whole 1″ budget. This is a
travel checklist item: official events are usually welded.

**Auto-calibrator.** Five routines measuring wheel scale, steering misalignment, gyro scale,
effective drive radius and per-module kS/kV, then the 10 ft acceptance run both open and closed
loop. Aborts if no tags are visible. Prints a paste-ready report; **writes nothing to source**.

**Per-module feedforward.** Fits each module separately rather than averaging first, because
published motor specs are typical values and individual motors vary. Reports peak-to-peak kV spread
and names the outlier. Under ~8% is normal; more means a corner is materially weaker, which pulls
the robot off a straight line — cross-track error, not distance error.

**Manoeuvre catalogue.** 25 runs in three families, each scored against its analytically expected
finishing pose: 16 drive-turn-drive permutations ({10 ft, 5 ft} × {90°, 270°} × {left, right} ×
{fwd, rev}), 4 same-path returns, 5 different-route loops. Read the two return families
differently — a wheel-scale error largely *cancels* on a same-path return, so small closure error
there does **not** mean the wheels are calibrated. It isolates heading error, backlash and
hysteresis. The different-route loops are where scale and heading compound.

**Localisation states.** `AIM_AT_HUB` faces the hub on our own half during our shift and sets the
flywheel from a distance-interpolated curve built out of the team's own four tuned presets and the
scoring-location geometry already in the code. `BUMP_REVERSE` turns so the back crosses first,
committing ~1 m early. Bump outranks aiming. Both supply **heading and shooter speed only** —
translation always stays with the driver, and touching the rotation stick overrides instantly. Both
fall back to `MANUAL` when the pose is untrustworthy.

**Game piece and jam detection from current.** The robot has no game piece sensor —
`EasyBreakBeam` exists in the framework and is never instantiated — so current is the sensor
already fitted. `MotorLoadMonitor` covers intake rollers, spindexer, feeder and flywheel.

The central point: **current alone is ambiguous.** A roller drawing 25 A might be ingesting a
piece, might be jammed, or might just be on a fresh battery. Two things make it work — a baseline
learned continuously while running *unloaded*, so what is measured is the excess; and velocity to
disambiguate, because high current with healthy speed is work while high current with collapsed
speed is a jam. No amount of current filtering achieves that second distinction alone.

`GamePieceCounter` adds sustain and refractory guards so one piece counts once. `JamClearing`
automates the jostling currently done by hand: three attempts at escalating amplitude, re-checking
between each, then giving up rather than pumping a mechanism that is not going to free.
`Intake.jiggleItALittleCommand()` was an unbounded `repeatingSequence` — acceptable behind a held
button, wrong for anything a sensor can trigger, so it is bounded now.

**Three calibrators for the thresholds**, because every one of them was reasoned rather than
measured and shipping guesses in a detector is how a piece count drifts mid-match:

- `LoadCalibrator` / `LoadCalibrationRoutine` — empty phase, loaded phase, obstructed phase, per
  mechanism. The interesting problem is knowing which loaded samples had a piece in them without
  the sensor being calibrated; samples above the empty phase's noise floor are attributed to a
  piece. That bootstrap guarantees the selected population clears the floor, so **viability is
  judged on the unselected phase mean instead** — otherwise every mechanism would report as
  viable, including one where pieces are invisible. A mechanism that cannot be detected is
  reported `NOT VIABLE` rather than given a number.
- `TractionCalibrator` — pushes into a wall, steps the drive current limit up until the wheels
  break loose, caps at 80 A. Slip is wheels turning while the chassis stays put; without that
  second half, driving away from the wall would read as slip, so a run where the pose moves
  aborts with **no recommendation at all**. The limit belongs below the traction limit not only
  for the motors: below it the wheels cannot slip under their own torque, so encoder distance
  always corresponds to ground travelled. Same error budget as the 1″ / 10 ft spec, arriving
  through the throttle instead of the wheel diameter.

**Diagnostics.** `ExpectationMonitor` evaluates six invariants every loop under `Expectations/` —
including one that fails if the driver commands motion and no module moves, which is the automated
form of the defect that started all this. `ValidationSuite` + `RebuiltValidation` provide a
14-check on-blocks self-test. `TunableNumber` allows live gain tuning, inert unless enabled.

---

## Open decisions — these need a human, not more code

| # | Decision | Why it matters |
| --- | --- | --- |
| 1 | **Module spacing**: PathPlanner says 27.01″, code says 26.50″ | 6.5 mm/side skews the kinematics so rotation and translation bleed together. Measure the frame; change whichever is wrong |
| 2 | **Drive current limit**: PathPlanner assumes 60 A, code applies 50 A | PathPlanner plans ~20% more torque than the drivetrain delivers, so the robot falls behind on hard acceleration. Electrical call |
| 3 | **Camera name + `ROBOT_TO_CAMERA`** | Vision is untrustworthy until measured. A wrong transform produces confidently wrong poses |
| 4 | **Bump band position** (`FieldRegions`) | Currently a placeholder mid-field band; `BUMP_REVERSE` fires off it |
| 5 | **Max speed cap** — 4.8 m/s vs ~5.74 physical | ~19% of top speed unused. Left alone deliberately |
| 6 | **CAN 18 motor: NEO 2.0 or Vortex?** Plus two `CONFIRM` SPARK MAX motors | 5676 vs 6784 RPM is 19.5%, and it feeds `FEEDER_EXPECTED_RPM` and therefore the jam threshold. Read the label or read step 9's measured free speed |
| 7 | **`docs/` tracked** — 49 generated files rewritten every build | Dirties the tree on every build. Untracking loses GitHub SVG previews; your call |

Decisions 1 and 2 are pinned by `PathPlannerSettingsConsistencyTest` as named allowances so they
cannot silently grow. A further test **fails if you ever reconcile them**, prompting the allowances
to be tightened to zero rather than left tolerating a regression.

---

## Things worth knowing about this codebase

- **`simulateJava` cannot drive the robot.** WPILib models no physics unless you write it. Three
  attempts were made and reverted; `RevSimUnitContractTest` records the measured unit contract so
  the next attempt starts from fact. `maple-sim` is purpose-built for swerve and probably the better
  route. See the runbook section.
- **`AutoBuilder` is a global singleton** that refuses to be configured twice.
  `configureAutoBuilder()` now guards on `isConfigured()`.
- **`RobotConfig.hasValidConfig()` is not a self-validity check.** It re-reads the GUI settings and
  compares — so it is always false without a deploy directory. It cost an hour; do not assert on it
  in tests.
- **REVLib rejects two controller objects on one CAN ID**, and Gradle runs all tests in one JVM.
  Hence `SharedSubsystems` shares the production drivetrain (swerve IDs are fixed) and gives the
  superstructure test-only IDs in the 40s.
- **A missing `settings.json` used to stop the robot booting** — it threw from `robotInit()` and
  discarded the original exception. Now prints the cause and falls back to `PathPlannerConfig`.
  Check `RobotUtils.isUsingFallbackConfig()` if paths follow badly.
- **Error Prone now errors** on `ReferenceEquality`, `NarrowCalculation` and `ShortCircuitBoolean` —
  each caught a real shipped bug here.

---

## Remaining test gaps, ranked

1. **Vision plausibility gates** — `isPlausible()` is private and untested. It decides which
   measurements get fused, so a wrong gate either rejects everything or admits garbage. Same
   extract-and-test treatment as `RotationAccumulator`.
2. **`ValidationSuite` pass/fail reporting** — if it mis-reports, a shop session gives false
   confidence.
3. **`FieldRegions`** — covered only indirectly through the state machine; the exact bump edges and
   the midfield boundary deserve direct tests.
4. **`ManeuverRunner` scoring** — expected-pose maths is tested, the along/cross decomposition of
   the result is not.
5. **Drivetrain physics simulation** — see above.
6. **`applyDriveGains` / `applyTurnGains` are wired to nothing.** Known-open since review #3. The
   new `applyDriveCurrentLimit` alongside them *is* used, by the traction sweep, so the pattern is
   now proven — connecting the gain setters to `TunableNumber` is a small job someone should do.
7. **`LoadCalibrationRoutine` phase routing** — the analysis is well covered, and composition and
   subsystem ownership are both asserted, but nothing verifies that each phase's samples land in the
   right accumulator, or that a mechanism's own current supplier is the one wired to it. A
   copy-paste of `feeder::getFeederCurrent` into the spindexer calibrator would pass every test
   here. Closing it needs a fake subsystem returning scripted currents.

---

## Three mistakes I made, for the record

**A boot-killing regression.** Adding subsystem requirements to the intake commands put two
commands requiring `INTAKE` into one parallel composition, which WPILib rejects at construction —
killing the robot program before teleop. Every test passed; only the simulator caught it. That is
why `ContainerWiringTest` now exists, and why it was verified by reintroducing the defect and
confirming the test fails with the exact WPILib message.

**A statistically biased estimator.** The wheel-scale calculation summed distances between noisy
positions, which always over-reports — a random walk is never shorter than the straight line. My
tests missed it because they injected noise-free synthetic samples. Now noise-corrected using the
measured standard deviation, with the raw ratio still exposed for comparison, and a test that
injects real noise.

**A baseline that lied during warm-up.** `MotorLoadMonitor` learned its idle current with a
`LinearFilter.movingAverage(100)`. That ramps up from zero, so for its first window the baseline
reads far too low — measured at 4.6 A against a true 8 A idle after 60 samples. Excess is
baseline-relative, so an artificially low baseline inflates it and the monitor reports game pieces
that are not there for the first seconds of every run. Replaced with an exponential average seeded
from the first raw reading, which starts at the right value and has no warm-up artefact. My own
tests caught this one before it left the branch.

All three are the same lesson: a test that cannot fail proves nothing, and synthetic or
zero-initialised data hides the errors that only appear in real data.

---

## Where things are

```
~/FRC/FRC26-Rebuilt/                                  main @ 23bf8b1 (untouched)
~/FRC/FRC26-Rebuilt-worktrees/StuartRevisions/         this work @ 404cbe2
    SHOP_RUNBOOK.md                                    ← the script for robot access
    passdowns/2026-08-04_claude_...md                  ← this file
~/FRC/FRC26-Rebuilt-worktrees/<11 other branches>      pre-existing
```

Artifact with the full review, diagrams and CAN map:
<https://claude.ai/code/artifact/73c05dc7-121a-4233-a280-1d340e3bbc97>

**Nothing pushed.** When you want it reviewed: push `StuartRevisions` and open a PR against `main`.
