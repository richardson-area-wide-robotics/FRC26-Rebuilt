# Shop runbook — what to do with physical robot access

Branch `StuartRevisions`. **Nothing here has touched hardware.** Work top to bottom; each step
gates the next, and step 2 is a hard stop if it fails.

Console output from every routine is prefixed, so you can follow along in the driver station
Console tab without AdvantageScope: `[validate]`, `[calib]`, `[maneuver]`, `[state]`,
`[Traction]`, `[LoadCalibration]`, `[sysid]`, `[bump]`.

**Steps 0, 2, 3, 8, 9 and 10 are written as numbered procedures.** Step 0 in particular says what
each measurement is taken *from* and *to* — a wrong datum there produces confidently wrong poses,
which is worse than no vision at all.

**Read 0-CAD first.** With the CAD you can skip 0a, 0b and 0d entirely — and CAD does the camera
better than a tape measure can. 0-CAD also lists what CAD *cannot* give, which is what is left of the
session, and includes the largest single term in the 10 ft error budget.

**Then start with 0c.** Two minutes, no tools, and it is a convention question CAD cannot settle.

**Every measurement assumes bumpers on**, since that is how this robot is always tested. Step 0b
explains why that changes nothing about the numbers and everything about where you take them from.

**Live telemetry is available during every step below** — see 0-LIVE. Everything the robot logs is
readable over NT4 as it happens, so the capture list can be gathered live rather than reconstructed
from a WPILOG afterwards.

**Four review passes so far, 38 defects closed.** The most recent found three defects in the arm
profiling within an hour of it being written, all of them boundary errors rather than mistakes inside a
component: wrong units across an interface, a controller running in a mode it should not, and a
calibration step longer than the travel it was measuring. Two of them would have presented as
mechanical faults. Section 12 of the review artifact has the detail.

Background, decisions and history: `passdowns/2026-08-04_claude_frc26-rebuilt-hardening.md`.

---

## Session plan at a glance

| Step | What | Needs | Rough time |
| --- | --- | --- | --- |
| 0-NET | **Check robot traffic is not going down the VPN** | Both NICs up | 5 min |
| 0-CAD | **Pull what you can from CAD first** — worksheet included | Onshape | 20–40 min |
| 0 | Whatever CAD cannot give — see 0-CAD | Tape, square, angle finder, PhotonVision UI | 10–45 min |
| 1 | Deploy | Laptop | 5 min |
| 2 | **On blocks** — 14-check self-test | Blocks, wheels clear | 10 min |
| 3 | **Verify drive directions** | ~3 m clear floor | 10 min |
| 4 | **Camera calibration (PhotonVision)** then vision commissioning | ChArUco board, practice field | 45 min |
| 5 | Auto-calibration | ~4 m clear, tags visible | 20 min |
| 6 | Manoeuvre suite | Large clear space | 20–60 min |
| 7 | Localisation states | Practice field | 20 min |
| 7b | **Rotational inertia** — CAD cannot give it | ~2 m clear all round | 10 min |
| 8 | **Traction limit** and **ramp bog-down diagnosis** | Wall, carpet, good battery, a ramp, a tag | 20 min |
| 9 | **Load thresholds** — piece and jam detection | Blocks, ~20 game pieces, a helper | 15 min |
| 9b | **Intake arm travel** and hard-stop detection | Blocks, **no** game pieces | 5 min |
| 9c | **Intake arm motion profile** — are the limits achievable? | Blocks, **no** game pieces | 10 min |
| 10 | **SysId feedforward** — the only source of kA | 28 ft of carpet, robot at one end | 10 min |
| 11 | Tuning | Time and patience | open-ended |

Steps 8 and 9 are independent of everything above them. If the field is busy, do them
first — step 8 needs only a wall and step 9 only blocks.

**Step 4a is not optional and cannot be reordered.** Step 6 measures wheel scale against AprilTag
ground truth, so an uncalibrated camera puts a systematic error straight into the wheel scale. A 5%
intrinsics error becomes a 5% wheel-scale error against a total budget of 0.833%.

### Bring to the shop

- **The CAD open on a laptop.** See 0-CAD; it removes most of step 0
- **Tape measure**, **masking tape**, a **carpenter's square**, a **marker**, and **string or a
  chalk line** — for whatever CAD does not cover, and for the diagonal squareness check
- A **digital angle finder or phone inclinometer** for camera pitch
- **Bumpers on**, as always. Step 0b explains why that changes nothing about the numbers but
  everything about where you measure them from
- **Scale** if you want to re-check the 47.6272 kg in `settings.json`
- **Blocks** — steps 2 and 9 will not be skipped
- **~20 game pieces** for step 9, and **a second person** to feed them
- **A good battery** for step 8 — a sagging pack under-reports the traction limit
- **The full 28 ft of carpet clear** for step 10, robot starting at one end
- **The 9×11″ 8×8 ChArUco board**, taped flat to something rigid, plus a laptop on the robot's
  network for PhotonVision. Measure the square and marker sizes across several squares rather than
  trusting the nominal figures
- **Decide the match camera resolution before calibrating** — the calibration only applies to the
  resolution it was taken at
- **wpical** on the laptop as well, if you want the optional field-layout calibration in 4c
- **A notebook**, or willingness to pull the WPILOG off the USB stick afterwards

### Data to capture before you leave

The calibrators print paste-ready blocks; keep all of them. Also worth recording:
`Calibration/Auto/WheelScale`, `Calibration/Auto/SteerOffsetDeg`,
`Calibration/Auto/ModuleKvSpreadPercent`, both `Acceptance/*/TotalErrorMm` figures,
`Calibration/Maneuvers/Summary/*`, `TractionCalibration/*`, `LoadCalibration/*/*`,
`SysId/Mean/*`, `SysId/WorstRunMeters` and `Inertia/*/MomentOfInertia` for both intake states.

**`SysId/WorstRunMeters` is worth a note of its own.** It says how much of your 28 ft each run
actually needed. If it comes back near the 6.0 m abort, the drivetrain is faster than the nominal
constants predict and the ramp wants shortening before the next session.

**Also grab `Vision/Layout/Provenance`.** It says whether the robot is using the official layout or
a wpical-calibrated one. A calibrated practice-field layout is wrong at an event, and nothing else
will tell you it is active.

**Also grab `Shooter/Sensors/AnalogRPM`.** It should read 0 — that channel was being used as
the flywheel's velocity source and there is no analog sensor on either shooter motor. If it
reads anything but 0, someone fitted one and that needs knowing.

---

## 0-CAD. What to take from CAD instead of measuring

Having the CAD removes most of step 0 and does a **better** job of the hardest parts — nobody measures
a lens pitch with a tape measure as well as a model does. But it is not a clean substitute, because
**CAD describes the design and the robot is the build.** Three categories, and the third is the one
worth reading carefully.

> **On this robot the mass-properties tool is unusable** — the assembly is large enough to crash it.
> So moment of inertia and centre of mass, which are normally the strongest reasons to reach for CAD,
> are not available. Step 7b measures inertia on the robot instead. If the tool will run on
> **subassemblies one at a time**, that is worth trying: note each one's mass and centre of mass, then
> combine them by hand with the parallel axis theorem. Many small computations instead of one large
> one is often the difference between crashing and finishing.

### 0-CAD worksheet — fill these in

**Six numbers to start with, in priority order.** If more of the model is available, the ranked list
further down goes as deep as you have time for. Each row says what the code believes now, so a disagreement is
visible immediately rather than after you have entered it.

| # | Number | Code says now | From CAD | Tolerance |
| --- | --- | --- | --- | --- |
| 1 | **Camera pitch** | −15.0° | ______ | **±0.5°** |
| 2 | **Camera yaw sign** | +90° *(unconfirmed)* | ______ | left = +90, right = −90 |
| 3 | **Wheel base / track width** | 26.500″ *(PathPlanner says 27.01″)* | ______ | **±2 mm** |
| 4 | **Camera x** (forward of centre) | 12.0″ | ______ | ±5 mm |
| 5 | **Camera y** (left of centre) | 0.0″ | ______ | ±5 mm |
| 6 | **Camera z** (floor to lens) | 8.0″ | ______ | ±5 mm |

---

#### 1. Camera pitch — do this one most carefully

The angle of the camera's **optical axis** from horizontal.

- **In CAD:** measure the angle of the camera's mounting face, or of a datum on the camera part that
  is parallel to the lens axis, relative to the chassis' horizontal plane.
- **Sign: negative is tilted UP.** WPILib's frame is right-handed with +y left, so a positive rotation
  about +y drops the nose. `PitchConventionTest` asserts this so it cannot be misremembered.
- **Goes in:** `VisionConstants.ROBOT_TO_CAMERA`, the middle argument of `Rotation3d`.

**Why ±0.5°:** pitch error grows with range, unlike every other number here.

| Range | 0.5° of pitch error |
| --- | --- |
| 2 m | 17 mm |
| 3 m | 26 mm |
| 5 m | 44 mm |

At 5 m half a degree already costs more than the entire 25 mm budget for the 10 ft run. This is why
step 0d offers a crosshair method as a physical cross-check — worth doing even with good CAD.

---

#### 2. Camera yaw — the sign is the whole question

The magnitude is known: **90°**, because the camera is in line with the shooter and the shooter is 90°
from the intake. What nobody has confirmed is which way.

- **In CAD:** which side of the chassis does the camera look out of, relative to the direction the
  robot drives forward?
- **+90 = out of the robot's LEFT.** −90 = out of the right. (+y is left in WPILib.)
- **Goes in two places, and they must match:** `VisionConstants.CAMERA_YAW_DEGREES` and
  `RebuiltConstants.GeometryConstants.SHOOTER_YAW_OFFSET_DEGREES`. `GeometryConsistencyTest` fails the
  build if they disagree, since the camera and shooter share a physical axis.

**Careful — CAD alone cannot fully settle this.** "Forward" is a *convention* tied to which module was
wired as front-left, not a fact about the geometry. Take the side from CAD, then confirm with the
two-minute check in 0c: stand behind the robot looking the way it drives forward.

**If the sign is wrong:** `AIM_AT_HUB` turns the robot 180° from the hub, and every tag pose is
rotated 180°. Both fail loudly rather than subtly, which is the one mercy here.

---

#### 3. Wheel base and track width — settles a live disagreement

Distance between the **module rotation axes**, not the frame and definitely not the bumpers.

- **In CAD:** distance between the swerve module mate connectors or part origins. Front-left to
  front-right is track width; front-left to rear-left is wheel base. On a square chassis they match.
- **Goes in:** `DriveConstants.kTrackWidth` and `kWheelBase`, **and** the four module offsets in
  `settings.json`, which are **half** the spacing.

**This resolves an open decision.** The two sources currently disagree:

| Source | Spacing | Offset |
| --- | --- | --- |
| `CommonConstants` | 673.1 mm (26.500″) | ±336.5 mm |
| `settings.json` | 686.0 mm (27.010″) | ±343.0 mm |
| **Difference** | **12.9 mm** | **6.5 mm per side** |

6.5 mm per side is a **1.9% kinematics error**, which makes commanded rotation bleed into translation
and vice versa — the robot drifts slightly sideways while spinning and yaws slightly while driving
straight. It looks like a tuning problem and no amount of tuning fixes it.

Whichever CAD says, change the other to match, then set the two `KNOWN_*_DIVERGENCE` constants in
`PathPlannerSettingsConsistencyTest` to zero. A test will tell you to.

**Still worth doing on the robot:** measure both floor diagonals (0a step 5). Equal diagonals mean the
frame is square; unequal means it is racked, and CAD cannot know that. That is a build check, not a
measurement.

---

#### 4–6. Camera x, y, z — three numbers from one place

All measured from the **robot origin: the centre of the four wheel contact patches, at floor level.**
Not the bumper centre, not the frame centre, not the CAD assembly origin.

- **x** — forward of the origin, positive forward
- **y** — left of the origin, positive left. Currently 0.0″, i.e. on the centreline
- **z** — floor to the **centre of the lens glass**, always positive

**Measure to the lens, not the camera body origin or the mount face.** Find the lens in the model; the
optical centre is a few mm inside the front face of the glass and the front face is close enough.

**x and y matter more than z for the 2D pose**, so do not stop at height. An error in x or y puts a
fixed offset of that size straight into every pose. z mostly affects the tag-elevation geometry, which
matters for single-tag solves and washes out for multi-tag.

> **The one cross-check worth keeping even with perfect CAD:** put a tape on the lens height and
> compare it to the model's z. Thirty seconds, and it catches every frame-conversion error at once — a
> +y sign flip, a wrong origin, an inch/mm slip. If z agrees, the conversion is probably right. If it
> does not, none of the other five numbers can be trusted either.

---

### If you can get more — everything else CAD can give, ranked

Split by whether it changes what the robot does today or unlocks a check. That distinction is worth
having: the first group is worth interrupting someone for, the second is worth doing while you are
already in the model.

#### Group A — wrong values change robot behaviour

| # | Number | Code says | What a wrong value does |
| --- | --- | --- | --- |
| 7 | **Drive pinion teeth** | 14 | 12T/13T/14T all bolt on and differ by **17% in free speed**. Scales the feedforward and the top speed — same magnitude and same failure mode as the wrong-motor bug. `MechanismRatios.DRIVE_PINION_TEETH` |
| 8 | ~~Moment of inertia about Z~~ | 3.733 kg·m² | **Cannot be had from this CAD** — the assembly crashes the mass-properties tool. Step 7b measures it on the robot instead, which also captures the wire and tape a model never has |
| 9 | **Module mounting orientation** | FL −90°, FR 0°, RL 180°, RR +90° | These are the standard MAXSwerve pattern, where each module is mounted a quarter turn from its neighbour. If they were all mounted the same way all four would be 0. A wrong one points that module 90° off. Confirm the pattern from CAD — the *fine* encoder zero is still a physical calibration |
| 10 | **Design mass** | 47.6272 kg *(looks weighed)* | Only if the mass-properties tool will do it on subassemblies one at a time. Not to replace the weighed figure — to **compare** |

#### Group B — currently changes nothing, but unlocks a check

| # | Number | Code says | What it buys |
| --- | --- | --- | --- |
| 11 | **Intake deploy reduction** | 1.0 *(placeholder)* | Turns `DEPLOY_POSITION_ROTATIONS = 10` and soft limits of 0–11 from opaque motor rotations into **arm degrees**. "Do the soft limits match real travel" has been unverified since the first review; this is what closes it |
| 12 | **Intake deploy arm travel, stowed → deployed** | *(unknown)* | With row 11, checks the two against each other directly. If 10 motor rotations does not equal the CAD travel, one of them is wrong |
| 13 | **Shooter reduction** | 1.0 *(placeholder)* | With row 14, predicts ball exit speed from motor RPM. If that comes out implausible for the distances the team actually makes, one of the three is wrong |
| 14 | **Flywheel diameter** | 0.1016 m *(4″, assumed)* | As above |
| 15 | **Bumper thickness** | implied 3.25″ per side | `settings.json` says 0.838 m across, consistent with 26.5″ + 2×3.25″. Feeds the SysId runway reserve and the robot footprint |
| 16 | **CG height** | *(not in code)* | Same tool, same problem. If subassembly-at-a-time works it is worth having, because it governs weight transfer going up a ramp — the mechanism behind step 8b |
| 17 | **Camera roll** | 0.0 | Assumed level. Only matters if the camera is deliberately canted |
| 18 | **Camera FOV / lens spec** | *(unknown)* | Step 4a checks the calibrated FOV against the spec sheet, within about ±10°. Having the spec to hand makes that check possible |

#### Not worth asking CAD for

Because CAD's answer would be wrong, not merely absent:

- **Effective wheel diameter.** CAD gives nominal 3.00″; loaded tread compresses 1–3% smaller, which is
  **30–91 mm over 10 ft against a 25 mm budget**. The largest single error term, and CAD is confidently
  wrong about it. Only the AprilTag run measures it
- **Steering encoder zeros.** A calibration, not a dimension
- **`wheelCOF`** and the traction limit. A property of the carpet
- **kS, kV, kA.** Friction and inertia through gearing — CAD's MOI helps a simulation, not a feedforward
- **Camera intrinsics.** Properties of the lens as manufactured

---

### CAD replaces these outright

| Number | Where it goes | How to get it |
| --- | --- | --- |
| Module positions | `DriveConstants.kTrackWidth` / `kWheelBase`, `settings.json` module offsets | Distance between the four module rotation axes. Settles the 26.50″ vs 27.01″ disagreement outright |
| **Camera position and angle** | `VisionConstants.ROBOT_TO_CAMERA` | See the coordinate-frame note below. This is where CAD wins hardest |
| ~~Moment of inertia about Z~~ | — | **Not available.** The assembly is too large for the mass-properties tool to finish. Measured on the robot instead — step 7b |
| ~~Centre of gravity height~~ | — | **Not available**, same reason |
| Bumper perimeter | `settings.json` `robotWidth` / `robotLength` | Currently 0.838 m = 33.0″, consistent with a 26.5″ frame plus 3.25″ bumpers each side |
| **All gear and pulley reductions** | `MechanismRatios` | Count teeth. See below — this is the other place CAD is authoritative and a tape measure is useless |
| Nominal wheel diameter | `ModuleConstants.kWheelDiameterMeters` | 3.00″ = 0.0762 m. Note *nominal*, see category three |

Skip **0a, 0b and 0d** if you take these from CAD. 0a only existed as a datum for measuring the
camera, so if the camera comes from CAD the floor marks are unnecessary.

### The coordinate frame is where CAD numbers go wrong

CAD gives exact numbers in **CAD's** frame. The code wants them in **WPILib's**, and the conversion is
the step that quietly ruins otherwise perfect data.

1. The robot origin is the **centre of the four wheel contact patches, at floor level**. An Onshape
   assembly origin is wherever the first part landed — almost certainly not that.
2. WPILib is **+x forward, +y LEFT, +z up**. CAD may well be +y right, or z along the length. Check,
   do not assume.
3. Measure to the **lens**, not the camera body origin or the mount face. Find the lens in the model.
4. **Pitch is positive downward**, so a camera tilted up is negative. Asserted by
   `PitchConventionTest` so it cannot drift.

> **Do one physical cross-check even with perfect CAD.** Measure lens height above the floor with a
> tape and compare it against the model's z. It takes thirty seconds and it catches every
> frame-conversion error at once — a sign flip, a wrong origin, a mm/inch slip. If z agrees, the frame
> conversion is probably right; if it does not, none of the other five numbers are trustworthy either.

### CAD gives these, but check the build anyway

| Number | Why CAD is not the last word |
| --- | --- |
| Module spacing | CAD gives the design. **Measure both diagonals on the floor** — equal diagonals mean the frame is square, unequal means it is racked and the kinematics' rectangle assumption is already violated. That is a build check, not a measurement |
| **Drive pinion teeth** | The BOM says what was ordered; the robot has what was fitted. 12T, 13T and 14T all bolt on, and they differ by **17% in free speed** — the same magnitude and the same failure mode as the wrong-motor bug. **Count the teeth on the robot** |
| Robot mass | CAD mass is design mass and is usually optimistic — wire, tape, zip ties and bumpers all go missing. `settings.json` has 47.6272 kg which looks weighed. **A scale beats CAD here** |
| Shooter and camera side | CAD shows the geometry, but which face is +x is a *convention* tied to how the module CAN IDs were assigned. Take it from CAD, then do the two-minute check in 0c |

### CAD cannot give you these at all

This is the list that matters, because it is what remains of the shop session — and it includes the
single largest term in the 10 ft error budget.

- **Effective wheel diameter.** CAD gives the nominal 3.00″. Under load the tread compresses and the
  rolling diameter is *smaller*, by 1–3%. That is **30–91 mm over 10 ft**, against a budget of 25 mm.
  It is the largest single error term and only the AprilTag wheel-scale run in step 6 measures it.
  CAD is confidently wrong here in a way that looks right.
- **Steering angular offsets.** The absolute encoder zero for each module is a calibration, not a
  dimension. ±0.5° of misalignment is worth 26.6 mm over 10 ft on its own.
- **Camera intrinsics.** Focal length, principal point and distortion are properties of the lens as
  manufactured. Step 4a, and everything distance-derived inherits it.
- **kS, kV, kA.** Friction, and inertia reflected through gearing. CAD's MOI helps a simulation; it
  does not give you the feedforward. Step 10.
- **Traction limit and `wheelCOF`.** A property of the carpet as much as the robot. Step 8.
- **Current thresholds for piece and jam detection.** Step 9.
- **The bump band.** A fact about the field, not the robot. Robot CAD says nothing; use the field
  drawings.

### Three inconsistencies in `settings.json` that CAD settles

Reading the file turned these up. All three are PathPlanner disagreeing with the code or with itself.

| Field | Value | Problem |
| --- | --- | --- |
| `flModuleX` etc. | ±0.343 m | Implies 27.01″ spacing; the code says 26.50″. **CAD settles it** |
| `robotTrackwidth` | 0.546 m | 21.50″ — a *third* value, agreeing with neither. Probably unused in `holonomicMode: true`, since PathPlanner uses the module offsets for holonomic kinematics, but it should not be left describing a robot that does not exist |
| `maxDriveSpeed` | 4.879 m/s | The constants give a physical **5.741 m/s**, so this is 15% low. PathPlanner will plan conservatively — not dangerous, but it is leaving speed unused and it is not a number anything derived |

---

## 0. Before power-on — the five measurements only you can make

Nothing in the code can derive these, and a wrong camera transform produces *confidently wrong*
poses, which is worse than no vision at all — the estimator fuses the error in and reports high
confidence while doing it.

**Everything below is measured with bumpers on**, since that is how this robot is always tested.
That matters most in 0b: bumpers hide the frame entirely, and the frame is not the datum anyway.

| # | What | Where it goes | Time |
| --- | --- | --- | --- |
| 0a | Robot origin marked on the floor | *(needed for 0d)* | 10 min |
| 0b | Module spacing, wheel centre to wheel centre | `CommonConstants.DriveConstants` + `settings.json` | 10 min |
| 0c | **Which side the shooter fires from** | `GeometryConstants` + `VisionConstants` | 2 min |
| 0d | Camera position and angle | `VisionConstants.ROBOT_TO_CAMERA` | 20 min |
| 0e | Bump band position | `FieldRegions` | 5 min |

You need: tape measure, masking tape, a carpenter's square, a marker, a digital angle finder or
phone inclinometer, and string or a chalk line.

---

### 0a. Mark the robot origin on the floor

Everything in 0d is measured **from the robot's centre of rotation, at floor level**. That is not
the centre of the bumper perimeter and not the centre of the frame — it is the centre of the square
formed by the four wheel contact patches. Find it once and mark it.

1. Put the robot on flat, hard floor. Not carpet — you need marks that stay put.
2. Point all four wheels **straight ahead**. Run step 2's self-test if the robot is already
   deployed, or push each wheel round by hand until it is square to the frame.
3. Lay a strip of masking tape on the floor beside each wheel.
4. For each wheel, hold the carpenter's square vertically against the **outer face of the wheel**
   and mark on the tape where the wheel's contact patch centre falls. On REV MAXSwerve the wheel is
   coaxial with the steering axis, so the contact patch centre sits directly below the module's
   turning axis — sighting down the turning motor is a good cross-check.
5. You now have four marks. **Measure both diagonals** — front-left to rear-right, and front-right
   to rear-left.
   - They should be equal. For a 26.5″ module square, each diagonal is **37.48″** (0.952 m).
   - If they differ by more than about ¼″ the frame is racked. Note it and carry on, but know that
     the kinematics assumes a perfect rectangle, so a racked frame puts a floor under how good the
     10 ft / 1 inch result can get.
6. Snap a chalk line or run string along each diagonal. **Where they cross is the robot origin.**
   Mark it with a cross and label it.
7. Extend a line forward from the origin, parallel to the robot's centreline, and mark it. This is
   the **+x axis** and you will measure along it in 0d.

---

### 0b. Module spacing — wheel centre to wheel centre

This is the open decision from the summary table: PathPlanner's `settings.json` says the modules sit
at ±0.343 m (27.01″ apart), the code says ±0.3366 m (26.50″ apart). One of them is wrong.

> **Do not measure the frame, and do not measure the bumpers.** With bumpers on, the frame is not
> even visible — and the number the code wants is the distance between wheel *centres*, which is
> unaffected by bumpers. Measuring across bumpers would give you roughly 32–33″ and quietly wreck the
> kinematics.

1. Use the four floor marks from 0a — they already are the wheel centres.
2. **Track width**: distance between the two *front* marks. Repeat for the two rear marks; they
   should match.
3. **Wheel base**: distance between the two *left* marks. Repeat on the right.
4. If you would rather measure on the robot than on the floor, measure **outside-to-outside across
   both front tyres, then subtract one tyre width** (each tyre contributes half a width). Measure the
   tyre width rather than assuming it.
5. Record all four figures. On a square chassis all four should agree.

Then resolve the disagreement:

- Both `kTrackWidth` and `kWheelBase` in `CommonConstants.DriveConstants` are `Units.inchesToMeters(26.5)`.
- `src/main/deploy/pathplanner/settings.json` carries `moduleOffset`, which is **half** the spacing.
- Change whichever is wrong so both describe the robot you measured, then set the two
  `KNOWN_*_DIVERGENCE` constants in `PathPlannerSettingsConsistencyTest` to zero. A test in the suite
  will tell you to.

**Why 6.5 mm per side matters:** the kinematics converts chassis motion into module motion using
these distances. Get them wrong and commanded rotation bleeds into translation and vice versa — the
robot drifts sideways slightly while spinning, and yaws slightly while driving straight. It looks
like a tuning problem and no amount of tuning fixes it.

---

### 0c. Which side the shooter fires from — two minutes, and everything else leans on it

The intake and shooter are 90 degrees apart, and the camera is in line with the shooter. The code
knows the magnitude; **it does not know which way round**, and two separate things depend on it.

1. Stand **behind** the robot, looking the direction it drives on a forward stick.
2. Find the shooter. Is it firing out of your **left** or your **right**?
3. If **left**: both offsets are `+90`. If **right**: both are `-90`.
4. Set `RebuiltConstants.GeometryConstants.SHOOTER_YAW_OFFSET_DEGREES`.
5. Set `VisionConstants.CAMERA_YAW_DEGREES` to the same value. A test fails the build if they
   disagree, since the camera and shooter share a physical axis.

**What each one breaks if the sign is wrong:**

| Constant | Wrong sign does this |
| --- | --- |
| `SHOOTER_YAW_OFFSET_DEGREES` | `AIM_AT_HUB` turns the robot 180° from where it should be. Fails obviously, at least |
| `CAMERA_YAW_DEGREES` | Every tag pose is rotated 180° instead of being correct. The estimator fuses it and reports confidence |

> Also worth knowing: **while these were both 0, `AIM_AT_HUB` aimed the intake at the goal.** The
> heading it commanded was the bearing to the hub, which points the chassis nose there — and the
> shooter is a quarter turn off the nose, so every shot would have gone sideways across the field.
> Fixed, and `RobotStateMachineTest` now asserts the *shooter* ends up on the hub rather than the nose.

---

### 0d. Camera position and angle → `ROBOT_TO_CAMERA`

This is a `Transform3d` from the robot origin to the **camera lens**. Six numbers: three distances
and three angles.

**Measure to the lens glass, not the case, not the mount, not the USB connector.** The optical
centre is a few millimetres inside the front face of the glass; the front face is close enough and
is something you can actually put a square against.

Robot coordinates, all from the origin you marked in 0a:

- **+x is forward**, the direction the robot drives on a forward stick
- **+y is LEFT** — this catches people out; it is left, not right
- **+z is up**, from the floor

#### The three distances

1. Hold the carpenter's square vertically, one edge flat on the floor, and slide it until the
   vertical edge just touches the **centre of the front face of the lens**.
2. Mark the floor at the base of the square's vertical edge. That mark is the lens projected straight
   down onto the floor.
3. **x** = distance from the origin cross to the lens mark, measured **along the +x centreline you
   marked in 0a**. Positive if the camera is forward of the origin.
4. **y** = perpendicular distance from that centreline to the lens mark. **Positive if the camera is
   to the robot's left.** A camera on the centreline is 0.
5. **z** = vertical height from the floor to the centre of the lens glass. Always positive.

Enter all three in inches via `Units.inchesToMeters(...)`, matching the existing style.

#### Roll and yaw

6. **Roll** is rotation about the forward axis — the camera being tilted sideways. Normally **0**.
   Check by looking at the PhotonVision stream: if the horizon is level, roll is 0. Leave it 0 unless
   the camera is deliberately canted.
7. **Yaw** is rotation about the vertical axis, positive counterclockwise seen from above — which
   means rotated toward the robot's **left**.

> **On this robot the yaw is 90 degrees, not 0.** The camera is mounted in line with the shooter, and
> the shooter is 90 degrees from the intake. Since the chassis forward axis is the intake direction,
> the camera looks out of the side.
>
> **A yaw of 0 here rotates every tag-derived pose by a quarter turn.** Vision becomes confidently,
> enormously wrong — worse than no vision, because the estimator fuses it and reports high confidence
> while doing it. This was the placeholder value and it is now 90; `GeometryConsistencyTest` keeps it
> tied to the shooter offset so the two cannot drift apart.
>
> **Confirm the sign, which is the one thing nobody has yet.** Stand behind the robot looking the way
> it drives forward on a forward stick:
>
> - Shooter and camera out of **your left** → both offsets are **+90**
> - Out of **your right** → both are **−90**
>
> Set `RebuiltConstants.GeometryConstants.SHOOTER_YAW_OFFSET_DEGREES` and
> `VisionConstants.CAMERA_YAW_DEGREES` to match. Getting the sign backwards aims the robot 180
> degrees from the hub, which at least fails obviously.

#### Pitch — measure this one twice

Pitch is the number that costs you the most if it is wrong. **A few degrees of pitch error becomes
tens of centimetres of pose error at the far end of the field**, and it looks exactly like a
calibration problem rather than a measurement one.

> **Sign: a camera tilted UP has a NEGATIVE pitch.** In WPILib's right-handed frame a positive
> rotation about +y takes the nose down, so up is negative. The placeholder in the file is `-15.0`
> for a camera tilted up 15°. This is asserted by `PitchConventionTest` so it cannot drift.

**Method 1 — the bracket.** Put a digital angle finder, or a phone inclinometer, flat against a
surface of the camera or its mount that is parallel to the optical axis. Read the angle off level.
Negate it if the camera points up. Quick, and enough to get within a degree or two.

**Method 2 — the crosshair.** Slower, better, and it measures the optical axis itself rather than a
surface you hope is parallel to it. Do this one as well and reconcile.

1. Park the robot on flat floor facing a wall, roughly 3 m away, square to the wall.
2. Measure **h_lens** — floor to lens centre. (You already have this: it is *z*.)
3. Measure **d** — horizontal distance from the lens to the wall.
4. Open the PhotonVision stream. Identify what sits at the **exact vertical centre** of the image.
   Have someone hold a marker against the wall and move it until it is dead centre.
5. Mark the wall there and measure **h_target** — floor to that mark.
6. Then:

   ```
   pitch_degrees = -atan((h_target - h_lens) / d) in degrees
   ```

   Camera tilted up puts the crosshair above the lens height, so `h_target > h_lens`, and the minus
   sign makes pitch negative. Which is the convention.

7. Worked example: lens at 0.20 m, crosshair lands at 1.00 m on a wall 3.00 m away.
   `-atan(0.80 / 3.00)` = `-atan(0.2667)` = **−14.9°**. Enter `-14.9`.

If the two methods disagree by more than about 2°, trust the crosshair and find out why the bracket
is lying — usually the surface you measured is not parallel to the optical axis.

#### Also do this while you are here

8. Copy the camera name **exactly** as PhotonVision's web UI shows it into `CAMERA_NAME`. A mismatch
   fails **silently**: no error, vision simply never contributes.
9. Check the bumper does not clip the bottom of the camera's view. Bumpers sit high enough to matter
   for a low-mounted camera, and an occluded lower field of view means close tags disappear exactly
   when you most want them.

---

### 0e. Bump band → `FieldRegions`

Two numbers, both **distances along the field from the blue alliance wall**, in metres. The reverse-
crossing state fires off these, so if they are wrong the robot either turns round for no reason or
fails to turn when it matters.

1. Work in the WPILib field frame: **x is measured from the blue alliance perimeter wall**, and
   increases toward the red end. This holds regardless of which alliance you are on — the code
   mirrors for red rather than keeping two sets of numbers.
2. Run a tape from the **blue alliance wall**, along the length of the field, to the **near edge of
   the bump** — the point where the floor first starts to rise, not the crest. That is
   `BUMP_NEAR_EDGE_METERS`.
3. Continue to the **far edge**, where the floor returns to flat. That is `BUMP_FAR_EDGE_METERS`.
4. Sanity check: `BUMP_FAR_EDGE_METERS - BUMP_NEAR_EDGE_METERS` should equal the physical width of
   the bump. The placeholders give 2.2 m, which is a guess.

> **With only half a field of carpet you may not be able to measure this at all.** If the bump is not
> on your half, take both numbers from the official field drawings rather than estimating. Leaving
> the placeholders in place is the one option that is definitely wrong: they describe a generic
> mid-field band and the state machine will act on them as though they were measured.

---

## 0-NET. Two NICs and a VPN — check this before you trust anything

Driver Station on a laptop with **one NIC on the robot and one on a VPN**. That combination has a
specific failure mode: robot traffic silently leaves via the wrong interface, and the Driver Station
shows no robot — or worse, an intermittent one — with nothing in the routing table looking wrong.

**Run this first, once both NICs are up:**

```powershell
pwsh -File tools/robot_preflight.ps1
```

It reports which interface would actually carry traffic to `10.17.45.2`, flags competing routes,
and — the part that matters — **pings**, because the check has to be empirical.

### Why routing usually saves you, and when it does not

Windows picks a route by **longest prefix match first**, then by metric. Your robot NIC gets a
connected `/24` for `10.17.45.0`, which beats a VPN's `0.0.0.0/0`, beats the `0.0.0.0/1` +
`128.0.0.0/1` pair full-tunnel VPNs install, and beats even a corporate `10.0.0.0/8`. So in most
setups it just works.

**What it does not survive is a VPN client enforcing full tunnelling below the routing layer.**
Zscaler, GlobalProtect and some AnyConnect profiles have a kill-switch mode that drops non-tunnel
traffic regardless of routes. Routes look perfect, packets die. No route fixes it — disconnect the
VPN while testing. The script identifies this case; it cannot work around it.

### If traffic is on the wrong interface

```powershell
# As Administrator. A /24 beats anything the VPN advertises short of the same /24.
New-NetRoute -DestinationPrefix 10.17.45.0/24 -InterfaceIndex <ifIndex> -RouteMetric 1
```

**Use the literal `10.17.45.2`, not `roborio-1745-frc.local`.** mDNS is unreliable with several NICs
up: the query goes out every interface and the first answer wins, which may be the wrong one.

---

## 0-LIVE. Live telemetry and live gain tuning

Everything the robot logs through `Logger.recordOutput` is published to NT4, so it is readable live
rather than only after pulling a WPILOG. `tools/nt_tool.py` does that.

```powershell
py -3.12 -m pip install pyntcore        # once

py -3.12 tools/nt_tool.py preflight                       # is NT up, is tuning available
py -3.12 tools/nt_tool.py list --filter sysid             # what exists
py -3.12 tools/nt_tool.py watch --preset load             # print live
py -3.12 tools/nt_tool.py capture --preset drive --preset vision --out run1.jsonl
py -3.12 tools/nt_tool.py set Shooter/kP 0.0004           # live gain write
```

Presets match the runbook's capture list: `drive`, `vision`, `sysid`, `inertia`, `traction`, `bump`,
`load`, `shooter`.

### Two things to know before relying on it

**Live gain writes need a deliberate deploy first.** `TunableNumber.TUNING_ENABLED` is a
compile-time `false`, so there are no `Tuning/` topics to write to until you set it true and
redeploy. That is intentional — a robot whose gains can be changed remotely is not something to
leave switched on by accident. `preflight` tells you which state you are in.

**NT4 is a snapshot channel, not a log.** Sampling at 10–20 Hz will alias against the robot's own
50 Hz loop, so it will miss the *shape* of a transient. For anything where that shape is the point —
the SysId acceleration step, a jam onset, a bump crossing — **trust the WPILOG on the roboRIO** and
use NT for monitoring and for tuning between runs. `capture` warns if you ask for a rate where this
starts to bite.

Also: a value written with `set` is **not persisted**. It lives until the code restarts. Paste
anything worth keeping into the constants.

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

1. Block the robot up so **all four wheels spin completely free** — at least 25 mm of air under each
   tyre. Blocks under the frame rails, not under the modules.
2. Push each wheel by hand and confirm it spins freely and the module rotates without binding.
3. Confirm nothing can reach a wheel: cables dressed, no game pieces on the floor under the robot.
4. Deploy if you have not already (step 1), then select **Test** mode on the driver station.
5. Watch the Console tab. Fourteen checks run automatically and each prints its own result.
6. Watch `Rebuilt/Validation/AllPassed` in AdvantageScope, or read the console summary.

**The robot will spin its wheels and move the intake without further warning.** Hands clear before
you enable.

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

Do this before anything that trusts odometry. A sign error here makes every later calibration
measure the wrong thing, and it is the fastest possible thing to check.

1. Robot on the floor, roughly 3 m of clear space ahead and to each side.
2. Stand **behind** the robot, looking the same way it faces. All directions below are from that
   viewpoint.
3. Enable teleop. Push the left stick **forward** — the robot must drive **away from you**.
4. Push the left stick **left** — the robot must drive **to your left**, not rotate.
5. Push the right stick **left** — the robot must rotate **counterclockwise seen from above**.
6. With the robot stationary, check `SwerveDriveSubsystem/GyroAngleDeg`. Rotate the robot
   counterclockwise by hand; the angle must **increase**.
7. Drive forward about 2 m and check the pose in AdvantageScope moved in **+x**.

Any one of these being backwards is a sign flip, not a tuning issue. Fix it before step 4 — the
calibration in step 5 compares odometry against AprilTags, and a sign error there produces a
plausible-looking wheel scale that is completely wrong.

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

## 4. Vision commissioning — camera calibration first

> ### Do 4a before step 6. This is an ordering constraint, not a suggestion.
>
> Step 6 measures wheel scale by comparing odometry against **AprilTag ground truth**. If the camera
> is not intrinsically calibrated, that ground truth carries a systematic distance error — and the
> wheel scale silently absorbs it.
>
> The arithmetic is unforgiving. A focal-length error scales estimated distance almost linearly, so a
> 5% intrinsics error becomes a 5% wheel-scale error. **The whole 10 ft / 1 inch budget is 0.833%.** An
> uncalibrated camera can therefore blow the entire budget six times over, while producing a
> perfectly plausible-looking number that sends someone hunting a mechanical fault that does not
> exist.
>
> This is the same trap as the wrong drive free speed: a real measurement of the wrong thing.

Two different jobs, two different tools:

| Job | Tool | Required? |
| --- | --- | --- |
| **Camera intrinsics** — lens distortion and optics | **PhotonVision**, Cameras → Calibration | **Yes.** 4a |
| **Field layout** — where your tags actually are | [wpical](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/wpical/index.html) | Optional. 4c |

The intrinsics go in PhotonVision, so calibrate them there — that is where they have to live and it
saves moving a file between tools. wpical does something PhotonVision does not: measure the *field*.

---

### 4a. Camera intrinsics in PhotonVision — required

Corrects for lens distortion and optics so that distances in the image mean something. Done in
**PhotonVision's own calibration tool** (Cameras tab → Calibration), which is where the intrinsics
have to end up anyway — see the
[PhotonVision calibration docs](https://docs.photonvision.org/en/latest/docs/calibration/calibration.html).

Your **9×11″ 8×8 ChArUco target** looks like PhotonVision's own default board: an 8×8 grid of 1″
squares with 0.75″ ArUco markers. That is convenient but **verify rather than assume** — a wrong
square size scales every distance the camera reports.

#### Board parameters

1. **Board type: ChArUco.** Not chessboard. The docs are explicit that chessboards give bad results
   when several similar images get taken, and ChArUco is more robust because each marker is uniquely
   identifiable.
2. **Squares across and down: 8 and 8.** Count them on the board to be sure.
3. **Square size:** measure it. Put a ruler across **several squares and divide** rather than
   measuring one — that averages out both your reading error and any print scaling. Should come out
   at 1.000″.
4. **Marker size:** measure one ArUco marker the same way. Expect 0.75″.
5. **ArUco dictionary:** must match what the board was generated with. Try PhotonVision's default
   first; detection either works or it does not, so this is seconds to test rather than something to
   agonise over.
6. **Legacy OpenCV pattern:** leave off initially. **If the board is not detected at all, this is the
   first thing to toggle** — targets generated before OpenCV 4.6.0 use a different marker layout and
   an Etsy board could be either vintage. Total failure to detect is the symptom; it is not a
   gradual degradation.

#### Taking the snapshots

7. **Set the resolution you will actually run in a match, and calibrate at that resolution.**
   Calibration is specific to each camera *and each resolution* — a model taken at 1280×720 is wrong
   at 640×480. If you switch resolution later you must recalibrate.
8. Mount the board **flat on something rigid**. Tape it to hardboard or a clipboard. A board with any
   bow in it calibrates the bow into your camera model, permanently.
9. Take **at least 12 snapshots**, and more is better. Vary:
   - **Distance** — near and far
   - **Angle** — tilt the board, up to about **45°**. Do not leave it parallel to the lens; parallel
     views carry almost no information about distortion
   - **Position in frame** — get the board into the **corners**, not just the middle. Corner coverage
     is what pins down distortion, and it is the thing people skip
10. **Your board is on the small side at 9×11″.** The docs recommend the largest target you can
    manage, so compensate: work closer to the camera, and take extra snapshots to get proper corner
    coverage. A small board held at distance sits in the middle of the frame and tells you nothing
    about the edges.

#### Checking it worked

11. **Mean reprojection error should be under 1 pixel.** Above that, take more and better-varied
    snapshots — usually the fix is more tilt and more corner coverage.
12. **Check the computed FOV against the camera's spec sheet.** Within about ±10° is fine. Wildly off
    means the board parameters are wrong — most often the square size.
13. **Independent sanity check, worth the two minutes:** put a tag at a measured distance, say 3.00 m
    from the lens, and compare what PhotonVision reports. Within a couple of centimetres is good.
    Consistently 5% out means the calibration did not take, and everything downstream inherits it.

> **The intrinsics live in PhotonVision, not in this repo.** No test here can detect a missing or bad
> calibration, because the robot code never sees the camera model — it only sees the poses that come
> out of it. That is exactly why this is called out so loudly rather than left to the build to catch.

---

### 4b. Verify the pipeline

Practice field, tags in view. Schedule `getVisionValidationCommand()`.

Five checks: camera connected, field layout loaded, a tag sighting accepted within 5 s, fused pose
agrees with wheel-only pose to within half a metre, latency under 200 ms.

- If `Rejected` is climbing fast, suspect `ROBOT_TO_CAMERA` or the field layout — the plausibility
  gates are probably throwing out poses that land off the field. With the camera 90° off (step 0c),
  a wrong yaw sign is the first thing to check.
- `SwerveDrive/VisionCorrectionMeters` shows how hard vision is pulling. Large and sustained means
  one of the two is wrong.

While commissioning, `VISION_SUBSYSTEM.setFuseIntoPoseEstimate(false)` lets you gather every
calibration figure without vision moving the robot.

---

### 4c. Field layout calibration — optional, and it cuts both ways

wpical's second job is measuring where your field's tags **actually are**, rather than where the
official layout says they should be, and writing a corrected layout.

Worth doing on a practice field: the welded and AndyMark official layouts already differ by up to
**3.6 cm**, and a hand-assembled practice field can be further out than either.

1. Finish 4a first. Field calibration consumes a camera model, so a bad one produces a bad field.
2. **wpical wants its own `cameracalibration.json`.** Whether PhotonVision's calibration can be
   exported into a form wpical accepts, or whether you have to run wpical's camera calibration
   separately on the same board, is something to establish at the laptop — I have not verified the
   formats are interchangeable. If in doubt, run wpical's own camera step with the same board; it
   costs one more video and removes the question.
3. Record video of the field's tags from several angles.
4. Supply the ideal field map as the starting reference. wpical **refines** a layout that is already
   roughly right; it cannot fix a tag that is grossly misplaced.
5. Pin one tag as the reference. Every other tag is measured relative to it, so the pinned tag's own
   position is inherited by the whole layout — pick one you are confident about.
6. **With only half a field of carpet**, calibrate the half you have. wpical supports combining
   calibrations from sectioned fields if you get access to the rest later.
7. Deploy the output as **`src/main/deploy/calibrated_field_layout.json`**. `FieldLayoutLoader` picks
   it up automatically and prints which layout is active at startup.

#### What the loader does with it

| Situation | Behaviour |
| --- | --- |
| No file deployed | Uses the compiled-in official layout. Normal, not an error |
| File present, tags within 30 cm of official | **Accepted.** Logs max and mean deviation |
| File present, any tag past 30 cm | **Rejected**, official layout used, worst tag named. A correction that large means the calibration failed rather than that your field is unusual |
| File malformed, or shares no tag IDs with official | Rejected, official layout used. It will not stop the robot booting |

Check `Vision/Layout/Provenance` and the `[vision]` line at startup.

> ### A calibrated practice-field layout is WRONG at competition
>
> The point of calibrating is to describe *your* field, including its assembly errors. An official
> event field does not have your field's errors — it has its own. Taking a practice-calibrated layout
> to an event makes vision **worse** than the official layout would have been, by exactly the amount
> your practice field is out of spec.
>
> And it fails silently: the file just sits in the deploy directory. **Delete
> `calibrated_field_layout.json` before an event**, or knowingly accept the error. The startup log
> line says `THIS IS YOUR PRACTICE FIELD` for this reason.
>
> Also remember `VisionConstants.FIELD_LAYOUT` is set to **AndyMark**, and official events are
> normally **welded**. Two separate things to change when you travel.

---

## 5. Drivetrain auto-calibration

> **Prerequisite: step 4a.** This step measures wheel scale against AprilTag ground truth, so it
> inherits any error in the camera's intrinsic calibration. Doing it with an uncalibrated camera
> yields a plausible number that is wrong by however much the camera is, which is the most expensive
> possible outcome — it looks like a measurement.
>
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

## 7b. Rotational inertia — the number CAD cannot give you

`settings.json` carries `robotMOI = 3.733 kg·m²` and PathPlanner uses it to plan rotational
acceleration. It is normally a CAD figure, and **this CAD cannot produce it** — the assembly is large
enough that the mass-properties tool crashes.

So measure it from the robot's own dynamics. `I = torque / angular acceleration`, with torque from the
measured motor current and angular acceleration from the gyro. Both are things the robot already
reports every loop, and it has the advantage over CAD of including the wire, tape and fasteners a
model never has.

1. **Robot on the floor with about 2 m clear all round.** It spins in place, twice.
2. Schedule `RebuiltContainer.getInertiaCalibrationCommand()`.
3. It stows the intake, spins, stops; deploys the intake, spins, stops; then reports both.
4. Each spin is only 0.6 s at 2.5 V. Short on purpose: the inertia is read from the **initial** slope
   of the angular rate, before drag matters. A long spin reaches a terminal rate governed by drag and
   would measure the wrong thing.

### Why both intake positions

Deploying moves several kilograms from tucked against the frame to well outside it, and inertia goes
as **mass × radius²**. Moving 5 kg from 0.20 m to 0.45 m adds 0.81 kg·m² — **22%** of the figure
currently in the file.

**So one `robotMOI` is necessarily wrong for one of the two states**, and no gain tuning fixes it:
PathPlanner will either plan rotations the robot cannot achieve, or under-drive it. The report gives
both values and the difference. If it exceeds 10%, pick the state your autos actually run in — usually
**deployed**, since that is when pieces are being collected.

### What invalidates a run

**Wheel slip.** The torque figure assumes every newton reaches the carpet, so a slipping wheel makes
the robot accelerate less than the current implies and the inertia comes out **too high** — plausibly
so, which is why it has to be a rejection rather than a warning. During a pure spin every wheel is
tangential, so a wheel's speed should be exactly the angular rate times the drive radius; anything
more than 15% faster is slipping and the run is refused.

2.5 V keeps well clear of it: the spin demands a coefficient of friction of about 0.26 against a
carpet that manages roughly 1.0.

### Sanity check on the existing number

Before you run anything, 3.733 is not obviously wrong. A lumped estimate — four 2.5 kg modules at the
0.422 m drive radius plus the rest as a plate over the inner frame — gives about **4.0 kg·m²**, and a
uniform plate over the whole bumper perimeter gives 5.6 as an upper bound. So it is in the right
range, just unverified. Expect the measurement to land near 4, and treat anything under 2 or over 7 as
a sign the run went wrong rather than a discovery.

---

## 8. Traction and the ramp bog-down — the drive current limit

### 8a. Traction — find the limit

1. Pick a **solid wall** — the field perimeter or a shop wall that will not move. Not a door, not
   shelving.
2. Drive the robot slowly up to it until the **bumper face contacts flat across its whole width**.
   Bumpers are what touch, so square the bumper, not the frame.
3. Check squareness by eye along the bumper face: no daylight at either end. A robot cocked at an
   angle will push itself sideways and the run aborts as "robot moved".
4. Fit a **freshly charged battery**. A sagging pack under-reports the traction limit, and the report
   will flag it but a good battery avoids the wasted run.
5. Keep hands and feet clear. The robot pushes at full output for 0.75 s at a time.
6. Schedule `RebuiltContainer.getTractionCalibrationCommand()`.
7. Watch the console. Each step prints one line as it completes.

Bumper compression lets the robot creep a centimetre or two as it loads up. That is expected and is
well inside the 10 cm the run allows before it decides the robot is not against the wall.

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

**Predicted result, so you have something to check against.** A NEO Vortex makes 3.60 N·m at its
211 A stall, so about 0.0171 N·m per amp. Through the 4.714 reduction onto a 0.0381 m wheel radius,
four modules give:

| Per-motor limit | Total push force | Coefficient of friction it demands |
| --- | --- | --- |
| 40 A | 338 N | 0.72 |
| **50 A** *(current setting)* | **422 N** | **0.90** |
| 55 A | 464 N | 0.99 |
| 60 A | 507 N | 1.08 |
| 80 A | 676 N | 1.45 |

At 47.6 kg the robot weighs 467 N, so **if `wheelCOF: 1.0` in `settings.json` is right, traction
should break at about 55 A.** That makes the current 50 A setting sit at 0.90 — only 10% below the
traction limit, with almost no margin.

If the sweep breaks traction well below 55 A, the real coefficient is lower than 1.0 and `wheelCOF`
wants updating too. If it holds past 60 A, the carpet grips better than assumed and there is real
headroom to take.

---

### 8b. The ramp bog-down — find out what is actually limiting you

At competition the chassis slowed crossing the field ramps and sometimes failed to get over, despite
having far more motor power than the job needs. Three candidates, and **they need opposite fixes**, so
guessing is expensive. `BumpCrossingDiagnostic` measures which it is.

1. **Run it where an AprilTag is visible.** Slip is wheel speed against chassis speed, and the only
   chassis speed on this robot that is independent of the wheels comes from the tags. Without one the
   run reports `TRACTION_NOT_MEASURABLE` rather than a false all-clear.
2. Schedule `RebuiltContainer.getBumpDiagnosticCommand()`. It **watches rather than driving**, so it
   measures the crossing as you actually take it.
3. Drive over the ramp normally, within the 6 s window.
4. Read the verdict from the console.

| Verdict | Fix |
| --- | --- |
| `CURRENT_LIMITED` | **Raise** the limit — run 8a first to find out whether headroom exists |
| `TRACTION_LIMITED` | **Lower** it. More current makes slip worse |
| `VOLTAGE_LIMITED` | Battery first. Current headroom is academic until then |
| `NOT_LIMITED` | Look at geometry — see below |
| `TRACTION_NOT_MEASURABLE` | No tag was in view. Re-run somewhere it is |

#### Why 3″ wheels make this hard, and why `NOT_LIMITED` may still fail to climb

A smooth incline is easy. A **sharp lip** is not, and the difference is dramatic. Taking moments about
the lip edge, a wheel of radius *r* meeting a step of height *h* needs a horizontal force of
`N × √(2rh − h²) / (r − h)`, which grows without bound as *h* approaches *r*. With 0.0381 m
(1.5″) wheels:

| Lip height | Force needed | vs 422 N at 50 A | vs 676 N at 80 A |
| --- | --- | --- | --- |
| 0.50″ | 1.12× load = 261 N | fine | fine |
| 0.75″ | 1.73× load = 405 N | **marginal** | fine |
| 1.00″ | 2.83× load = 661 N | **not enough** | just enough |
| 1.25″ | 5.92× load = 1382 N | no | no |

Compare a smooth ramp: a 15° incline needs only **121 N**. So the ramp's *slope* is nothing and its
*lip* is everything.

**This is the quantitative case for the current-limited prior.** A 1″ lip needs 661 N and 50 A
delivers 422 N — so the robot would bog exactly as described, while the motors sit at their limit and
the wheels grip. Measure the lip height on the field and read off the table.

It also means **more current may not be the answer**: if the lip is 1.25″, no per-motor limit inside
the 80 A cap gets you over it and the fix is larger wheels or a different approach angle. That is what
`NOT_LIMITED` is telling you when it appears.

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

### Running it

1. Robot on blocks, as step 2. Mechanisms will run without warning.
2. Put **at least 20 game pieces** within arm's reach of whoever is feeding.
3. Two people: one at the driver station, one at the mechanism. The console prints each phase as it
   starts and the person feeding needs to hear it.
4. Schedule `RebuiltContainer.getLoadCalibrationCommand()`.
5. **Empty phase, 4 s** — hands and pieces clear. Do not touch the mechanism.
6. **Loaded phase, 8 s** — feed pieces through continuously, one after another, for the whole phase.
   Gaps between pieces are expected and handled. Feeding only two pieces in eight seconds is the one
   way to waste the run.
7. **Obstructed phase, 2 s** — hold the mechanism so it cannot move product. Use a piece of wood or a
   spare game piece, **not your hand**. Two seconds only; a near-stalled motor turns almost all its
   input into heat.
8. Read the line printed for that mechanism before moving on. If it says `INCOMPLETE`, re-run just
   that mechanism rather than continuing.

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

## 9b. Intake arm travel — and knowing "deployed" from "hit a ball"

**On blocks, and with no game pieces anywhere in the robot.** Schedule
`RebuiltContainer.getDeployTravelCommand()`.

Drives the arm gently onto the stowed stop, then the deployed stop, then the stowed stop again, and
reports the real travel.

### Why current alone cannot tell you the arm is deployed

Reaching the end of travel and pushing a ball look identical to a current sensor — both are "current
up, speed down". The discriminator is mechanical:

| | Position | Clears? |
| --- | --- | --- |
| **Hard stop** | Frozen to encoder noise, indefinitely | Never |
| **Ball** | Keeps **creeping** as it squashes or rolls | Usually within a few hundred ms |

So `HardStopDetector` watches whether position is **frozen or creeping** while the motor pushes.
`isFullyDeployed()` and `isFullyStowed()` mean the arm physically ran out of travel;
`isDeployPushingBall()` means it stalled on something that is still moving — which is the case a
jostle can clear.

There is a **third** case that current gets backwards. A **soft limit** also freezes position, but the
SPARK enforces it by cutting output, so current **falls** rather than rising. A detector looking only
for "stopped" would call it a hard stop. It is reported as `AT_SOFT_LIMIT` and deliberately never
learned as a position reference.

### The latent problem this fixes

`Intake`'s constructor calls `deployEncoder.setPosition(0)` — it **assumes the arm is stowed when the
code starts**. If it ever boots part-way, from a mid-match reboot, a brownout, or someone moving the
arm by hand with the robot off, then:

- every position afterwards carries that offset, and
- **the soft limits carry it too**, because they are expressed in the same encoder units.

Soft limits referenced to a relative encoder are only as trustworthy as the boot assumption. A
confirmed hard stop is an absolute reference: `Intake.rezeroDeployAtStowedStop()` corrects the encoder
against the physical stop instead. It refuses to act unless the stop is confirmed, because otherwise it
would be writing the very assumption it exists to replace.

Watch `Intake/Deploy/EncoderDrift` — non-zero at the stowed stop means the boot assumption was wrong
by that much.

### Reading the report

| Line | What to do |
| --- | --- |
| `MEASURED TRAVEL x.xxx rotations` | Compare against `DEPLOY_POSITION_ROTATIONS = 10` |
| `ASKS FOR MORE TRAVEL THAN EXISTS` | The deploy target is past the stop, so the arm sits against steel drawing current on every deploy. Lower it |
| forward limit `OUTSIDE the physical stop` | The soft limit protects nothing — the arm reaches steel first |
| `the stowed stop moved x.xxx between visits` | Not a calibration problem. Either the encoder is losing count or a fastener is backing out. Fix that first |
| `INCOMPLETE` | A stop was not found. Usually a game piece was loaded, or a soft limit stopped the arm first |

**This closes the "do the soft limits match real travel" item** that has been on the unverified list
since the first review — without needing CAD, and describing the arm as built rather than as drawn.

> One honest limit: a ball wedged so hard it cannot move at all **is** mechanically a hard stop, and no
> signal distinguishes them. That is why the instruction is to run with the robot empty.

---

## 9c. Intake arm motion profile

The arm used to be driven by plain `kPosition` on the SPARK — output proportional to error, so a
full-travel move began at **maximum error and therefore maximum output** and decelerated only as the
error shrank. It slammed at both ends and the hard stop was what caught it.

It now follows a **trapezoid profile with PID**: a position *and velocity* setpoint respecting a
velocity and an acceleration limit, so the controller only ever chases a nearby target and the
mechanism sees bounded acceleration instead of a step.

### Calibrating it

Schedule `RebuiltContainer.getArmProfileCommand()` — or `getSuperstructureCalibrationCommand()`,
which runs travel, profile and load thresholds in the right order.

**Run 9b first.** The gravity phase drives to fractions of the measured travel, so it is skipped if
the travel is unknown.

| Phase | Measures |
| --- | --- |
| Break-away ramp, **both directions** | The voltage at which the arm starts to move |
| Voltage steps to steady state | `DEPLOY_kV` |
| Holding voltage at five positions | The gravity signature |
| Near-full-output move | **Achievable** velocity and acceleration |

### The check that matters most

**Are the configured constraints achievable?** If the profile asks for more velocity or acceleration
than the arm can deliver, the controller saturates, the arm falls behind its own setpoint, and the
following error grows through the whole move — **which looks exactly like a badly tuned gain.** No
amount of PID tuning fixes a profile asking for the impossible.

The report compares achievable against configured and, if configured is higher, tells you to drop to
about 80% of achievable. Watch `Intake/Deploy/Profile/FollowingError`: it should stay small
*throughout* the move, not only at the end.

### Break-away is measured both ways on purpose

Gravity helps one direction and opposes the other, so an arm has **two** break-away voltages. Their
average is friction; **half their difference is gravity.** A single kS would be an average of two
unlike things, and the report separates them.

### Why gravity is a table, not a cosine

The textbook arm feedforward is `kG x cos(angle from horizontal)`. Fitting that needs the arm's angle,
which needs both the deploy reduction and where horizontal falls on the encoder — and the reduction
cannot be measured, only counted from CAD.

So instead the holding voltage is measured **at positions**, with no geometry involved: whatever the
profiled controller has settled on to hold station *is* the answer. Read the spread:

- **Under 0.3 V** — gravity barely varies across the travel. A constant bias is enough and
  `DEPLOY_HOLD_SPEED` already provides one. Not worth chasing the geometry.
- **Large** — gravity matters, a constant bias is wrong at one end, and `DEPLOY_kG` plus the arm angle
  are worth having. The number tells you what is at stake before anyone spends time on it.

`DEPLOY_kG` is currently **0**, so the gravity term is inert. Deliberately: a gravity feedforward built
on a guessed angle pushes hardest in the wrong place.

### If the arm barely moves, check this before the gearbox

The gains shipped are **conservative on purpose**, so the first symptom you may see is the arm
**lagging behind its setpoint** rather than tracking it. That is expected and it is a gain problem, not
a mechanical one.

| Symptom | Likely cause | What to do |
| --- | --- | --- |
| Arm moves slowly or not at all | `DEPLOY_kP` too low, `kV` under-estimated | Raise `kP` in steps while watching `FollowingError` |
| Following error grows through the move, then recovers at the end | Profile asking for more than the arm can do | Lower `MaxAccelRps2` first, then `MaxVelRps` |
| Arm overshoots and settles back | `kP` too high, or `kV` too high | Lower `kP`; add a little `kD` only if it oscillates *while following* |
| Arm lurches on the first move after enabling | Should not happen — this was a bug, now fixed and tested | Report it, do not tune around it |

> **A note on why `kP` starts where it does.** The gain used to live on the SPARK in `kPosition` mode,
> where output is a **duty cycle** — 0.05 there means 0.6 V. The profiled controller commands **volts**,
> so the same 0.05 would have meant 0.05 V per rotation: it would have needed **76 rotations of error on
> a mechanism with about 10 rotations of travel.** The arm would have barely moved and it would have
> looked like a mechanical fault. `kV` is now derived from free speed so the feedforward carries the
> velocity, and `kP` at 1.0 only corrects error. Say so to whoever tunes it, because the units of that
> number changed and its old value is meaningless now.

### Tuning it live

`IntakeDeploy/kP`, `kD`, `MaxVelRps` and `MaxAccelRps2` are `TunableNumber`s, so with
`TUNING_ENABLED` true they can be changed over NT while the arm moves — see 0-LIVE. A move takes about
half a second, so the effect is immediate.

**Lower acceleration first if the arm is harsh at the ends.** Acceleration is what the mechanism feels;
the velocity limit only sets how long the move takes.

---

## 10. SysId — the feedforward, including kA

1. Clear the **full 28 ft** of carpet. Nothing on it, nobody standing on it.
2. Place the robot at **one end**, facing down the length of the carpet, with its rear bumper roughly
   200 mm from the wall behind it. The forward runs go away from that wall.
3. Fit a **freshly charged battery**. kV is voltage-referenced, so a sagging pack biases the fit.
4. Check the robot is square to the carpet's length. It drives in a straight line with no correction,
   so a few degrees of yaw at the start becomes a metre of lateral drift by the end.
5. Schedule `RebuiltContainer.getSysIdCommand()`.
6. Watch it. **The robot drives itself at up to about 2.4 m/s and does not steer.** Be ready to
   disable.
7. The console prints each run's name as it starts and the distance it used as it finishes.

Budget check you can do on the day: the longest run is planned at about 4.1 m, aborts at 6.0 m, and
your carpet is 8.53 m. Starting 0.2 m off the back wall, a worst-case aborted run leaves roughly
2.3 m in front of the robot to stop in from 2.4 m/s, in brake mode. Comfortable, but this is why the
carpet needs to be clear rather than mostly clear.

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
- [ ] Intake arm gains from step 9c: `DEPLOY_kS`, `DEPLOY_kV`, and the profile constraints if they
      turned out to be unachievable
- [ ] Noted whether the arm's gravity spread was small (constant bias is fine) or large (the arm
      geometry is worth getting from CAD after all)
- [ ] `settings.json` `robotMOI` updated from step 7b, and a note of **which intake state** it
      describes — a single value cannot serve both
- [ ] SysId kS/kV cross-checked against the auto-calibrator's figures
- [ ] `Shooter/Sensors/AnalogRPM` confirmed reading 0
- [ ] Camera intrinsics calibrated in PhotonVision, at match resolution, mean reprojection error
      under 1 px (step 4a)
- [ ] `Vision/Layout/Provenance` noted — and if it says CALIBRATED, a reminder set to delete
      `calibrated_field_layout.json` before travelling to an event
- [ ] Decisions 1 and 2 resolved, or at least measured
- [ ] `settings.json` and `CommonConstants` reconciled — then tighten the two
      `KNOWN_*_DIVERGENCE` constants in `PathPlannerSettingsConsistencyTest` to zero
- [ ] Bump band measured and entered in `FieldRegions`
- [ ] **Field ramp lip height measured** — it is what decides whether 8b's verdict can be fixed with
      current at all (see the table in 8b)
- [ ] `MechanismRatios` filled in from CAD, especially `DRIVE_PINION_TEETH` counted on the robot
- [ ] `settings.json` `robotTrackwidth` and `maxDriveSpeed` reconciled — see 0-CAD
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
- **Camera intrinsic calibration** — nothing in this repo can detect whether it has been done, since
  the intrinsics live in PhotonVision. Every distance-derived figure inherits it (step 4a)
- **Every intake arm gain.** `DEPLOY_kP` is a conservative 1.0 V/rotation and `DEPLOY_kV` is derived
  from free speed rather than measured, so expect the arm to follow but lag (step 9c)
- **Whether the arm's profile constraints are achievable.** 30 rot/s and 150 rot/s² are chosen, not
  measured. Unachievable constraints make the following error grow through every move, which reads as a
  tuning problem and is not one
- **The frozen-band and pushing-current thresholds** that separate a hard stop from a ball — reasoned
  from encoder noise against ball compliance, never watched on a real arm with a real ball under it
- **That the arm has two hard stops inside its soft limits at all.** If a soft limit sits inside a
  stop, step 9b reports the stop as never found — and travel, goal clamping and the encoder re-zero all
  depend on finding it
- **The drive pinion tooth count.** 14T is assumed. 12T would change free speed by 17%, which is the
  same magnitude and the same failure mode as the wrong-motor bug. Count it (0-CAD)
- **Every reduction in `MechanismRatios`** — placeholders of 1.0, which makes the conversions the
  identity and changes no behaviour, but means the intake deploy's travel is still expressed in opaque
  motor rotations rather than arm degrees
- **`robotMOI` = 3.733 kg·m²** — provenance unknown and CAD cannot check it on this robot. A lumped
  estimate says it is in the right range; step 7b measures it
- **That `robotMOI` describes only one intake state.** Deploying the intake changes rotational inertia
  by an estimated 13–45%, so whichever value goes in the file, PathPlanner is wrong about the other
  configuration. Not fixable by tuning — a real limit on rotational path-following accuracy
- **Centre of mass, in either state.** Not measured, not in the code, and it governs weight transfer
  on the ramp — which is the mechanism behind step 8b
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