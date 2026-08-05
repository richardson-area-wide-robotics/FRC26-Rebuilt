package frc.robot.rebuilt.subsystems;

import frc.robot.CommonConstants;
import frc.robot.common.annotations.NamedAuto;
import frc.robot.common.components.diagnostics.GamePieceCounter;
import frc.robot.common.components.diagnostics.HardStopDetector;
import frc.robot.common.components.diagnostics.TunableNumber;
import frc.robot.common.components.diagnostics.MotorLoadMonitor;
import frc.robot.rebuilt.RebuiltConstants.IntakeConstants;
import frc.robot.rebuilt.RebuiltConstants.LoadConstants;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.common.components.EasyMotor;
import frc.robot.common.components.RobotUtils;
import frc.robot.common.subsystems.DashboardSubsystem;

/**
 * Floor intake: two counter-rotating rollers on a motorised deploy arm.
 *
 * <p>Note the distinction between {@link #stopRollers()}, which genuinely stops, and
 * {@link #holdRollers()}, which applies a small retention bias. Both behaviours existed
 * before, but the biasing one was called {@code stop()}, which hid the fact that the rollers
 * were drawing current all match.
 */
public class Intake extends DashboardSubsystem {

    private final SparkFlex intakeMotor1;
    private final SparkFlex intakeMotor2;
    private final SparkMax deployMotor;
    private final RelativeEncoder deployEncoder;
    private boolean intakeRunning;

    public Intake(int intakeID1, int intakeID2, int deployID) {
        intakeMotor1 = new SparkFlex(intakeID1, SparkLowLevel.MotorType.kBrushless);
        intakeMotor2 = new SparkFlex(intakeID2, SparkLowLevel.MotorType.kBrushless);

        SparkFlexConfig intakeConfig1 = new SparkFlexConfig();
        intakeConfig1.idleMode(IdleMode.kCoast);
        intakeConfig1.smartCurrentLimit(CommonConstants.SUPERSTRUCTURE_CURRENT_LIMIT);
        intakeMotor1.configure(intakeConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig intakeConfig2 = new SparkFlexConfig();
        intakeConfig2.idleMode(IdleMode.kCoast);
        intakeConfig2.smartCurrentLimit(CommonConstants.SUPERSTRUCTURE_CURRENT_LIMIT);
        intakeMotor2.configure(intakeConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        deployMotor = EasyMotor.createEasySparkMax(deployID, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kBrake);
        deployEncoder = deployMotor.getEncoder();

        SparkMaxConfig deployConfig = new SparkMaxConfig();

        // MAXMotion: the trapezoid profile runs on the controller at its own rate, not in robot code
        // at 50 Hz. Two reasons, both the team's:
        //
        //   CAN traffic. A reference persists until changed, so holding position costs zero frames.
        //   A roboRIO-side controller has to write an output every loop forever. On a single bus with
        //   fifteen devices, saturation is a real failure mode.
        //
        //   Rate. The controller closes the loop far faster than a 50 Hz robot loop can.
        //
        // Note the units MAXMotion wants: velocity in RPM and acceleration in RPM per second, not the
        // rotations per second the calibration reports. Converted here rather than stored twice.
        populateDeployClosedLoop(deployConfig);
        deployConfig.smartCurrentLimit(CommonConstants.SUPERSTRUCTURE_CURRENT_LIMIT);

        // Soft limits enforced by the controller itself, so they also bound open-loop
        // manual jogging. Previously these bounds existed only as commented-out checks in
        // periodic(), leaving nothing to stop the arm over-travelling into the frame.
        deployConfig.softLimit
            .reverseSoftLimit(IntakeConstants.DEPLOY_REVERSE_SOFT_LIMIT)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimit(IntakeConstants.DEPLOY_FORWARD_SOFT_LIMIT)
            .forwardSoftLimitEnabled(true);

        deployMotor.configure(deployConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        deployEncoder.setPosition(0);
    }

    public void intake() {
        intakeMotor1.set(-IntakeConstants.ROLLER_SPEED);
        intakeMotor2.set(IntakeConstants.ROLLER_SPEED);
        intakeRunning = true;
    }

    public void outtake() {
        intakeMotor1.set(IntakeConstants.ROLLER_SPEED);
        intakeMotor2.set(-IntakeConstants.ROLLER_SPEED);
        intakeRunning = true;
    }

    /**
     * Stops the rollers completely.
     *
     * <p>Use this whenever the intake should be genuinely idle — it draws no current.
     */
    public void stopRollers() {
        intakeMotor1.stopMotor();
        intakeMotor2.stopMotor();
        intakeRunning = false;
    }

    /**
     * Applies a light inward bias to keep a held game piece from falling out.
     *
     * <p>This is what the old {@code stop()} actually did. It is intentional, but it is not
     * a stop, and it costs current for as long as it is applied.
     */
    public void holdRollers() {
        intakeMotor1.set(-0.1);
        intakeMotor2.set(0.1);
        intakeRunning = false;
    }

    /** @return true while the rollers are commanded to intake or eject. */
    public boolean isRunning() {
        return intakeRunning;
    }

    /** @return deploy arm position in motor rotations. */
    public double getDeployPosition() {
        return deployEncoder.getPosition();
    }

    /** @return true when the deploy arm has reached the deployed position. */
    public boolean isDeployed() {
        return Math.abs(getDeployPosition() - IntakeConstants.DEPLOY_POSITION_ROTATIONS)
            <= IntakeConstants.DEPLOY_TOLERANCE_ROTATIONS;
    }

    /** @return true when the deploy arm has reached the stowed position. */
    public boolean isStowed() {
        return Math.abs(getDeployPosition() - IntakeConstants.STOW_POSITION_ROTATIONS)
            <= IntakeConstants.DEPLOY_TOLERANCE_ROTATIONS;
    }

    @NamedAuto(value = "Enable Intake")
    public Command intakeCommand() {
        return Commands.runOnce(this::intake, this);
    }

    @NamedAuto(value = "Disable Intake")
    public Command stopIntakeCommand() {
        return Commands.runOnce(this::holdRollers, this);
    }

    public Command deploy() {
        return Commands.runOnce(
            () -> deployToGoal(IntakeConstants.DEPLOY_POSITION_ROTATIONS), this);
    }

    /**
     * Holds the deploy arm against its stop.
     *
     * <p>Not a true stop — see {@code IntakeConstants.DEPLOY_HOLD_SPEED}.
     */
    public void stopDeploy() {
        leaveProfiledMode();
        deployDemand = IntakeConstants.DEPLOY_HOLD_SPEED;
        deployMotor.set(IntakeConstants.DEPLOY_HOLD_SPEED);
    }

    public void manualDeploy() {
        leaveProfiledMode();
        deployDemand = IntakeConstants.MANUAL_DEPLOY_SPEED;
        deployMotor.set(IntakeConstants.MANUAL_DEPLOY_SPEED);
    }

    public void manualReverseDeploy() {
        leaveProfiledMode();
        deployDemand = IntakeConstants.MANUAL_RETRACT_SPEED;
        deployMotor.set(IntakeConstants.MANUAL_RETRACT_SPEED);
    }

    public Command reverseDeploy() {
        return Commands.runOnce(
            () -> deployToGoal(IntakeConstants.STOW_POSITION_ROTATIONS), this);
    }

    @NamedAuto(value = "Deploy Intake")
    public Command manualDeployCommand() {
        return Commands.runOnce(this::manualDeploy, this);
    }

    @NamedAuto(value = "Reverse Deploy Intake")
    public Command manualReverseDeployCommand() {
        return Commands.runOnce(this::manualReverseDeploy, this);
    }

    /**
     * Pumps the deploy arm a fixed number of times to shake a jammed piece loose.
     *
     * <p>Was an unbounded {@code repeatingSequence}: it never finished, so it would have jostled
     * until something interrupted it. Acceptable behind a held button, but not for anything a
     * sensor can trigger — so it is now bounded.
     *
     * <p>For automatic clearing prefer {@link JamClearing#intakeJostle}, which also re-checks
     * between attempts and gives up rather than pumping a mechanism that is not going to free.
     *
     * @return a bounded jostle command.
     */
    public Command jiggleItALittleCommand() {
        return Commands.sequence(
            RobotUtils.timedCommand(0.35, Commands.run(this::manualReverseDeploy), stopIntakeCommand()),
            RobotUtils.timedCommand(0.25, Commands.run(this::manualDeploy), stopIntakeCommand()),
            RobotUtils.timedCommand(0.35, Commands.run(this::manualReverseDeploy), stopIntakeCommand()),
            RobotUtils.timedCommand(0.25, Commands.run(this::manualDeploy), stopIntakeCommand()))
            .withName("JiggleIntake");
    }

    /** @return roller speed in motor RPM, averaged across the two rollers. */
    public double getRollerVelocity() {
        return (Math.abs(intakeMotor1.getEncoder().getVelocity())
            + Math.abs(intakeMotor2.getEncoder().getVelocity())) / 2.0;
    }

    /** @return combined roller current in amps, which is what a game piece shows up in. */
    public double getRollerCurrent() {
        return intakeMotor1.getOutputCurrent() + intakeMotor2.getOutputCurrent();
    }

    /** @return deploy arm current in amps. */
    public double getDeployCurrent() {
        return deployMotor.getOutputCurrent();
    }

    /** @return deploy arm speed in motor RPM. */
    public double getDeployVelocity() {
        return deployEncoder.getVelocity();
    }

    /**
     * Detects game pieces and jams from roller current and speed.
     *
     * <p>The robot has no game-piece sensor — {@code EasyBreakBeam} exists in the framework and is
     * never instantiated — so current is the sensor that is already fitted.
     */
    private final MotorLoadMonitor rollerLoad = new MotorLoadMonitor(
        "Intake/Rollers",
        LoadConstants.INTAKE_WORK_EXCESS_AMPS,
        LoadConstants.INTAKE_EXPECTED_RPM,
        LoadConstants.JAM_SPEED_FRACTION,
        LoadConstants.JAM_CONFIRM_LOOPS);

    private final GamePieceCounter pieceCounter = new GamePieceCounter(
        "Intake", rollerLoad,
        LoadConstants.PIECE_SUSTAIN_LOOPS,
        LoadConstants.PIECE_REFRACTORY_LOOPS);

    /** @return true while a game piece appears to be going through the rollers. */
    public boolean isIntakingPiece() {
        return rollerLoad.isDoingWork();
    }

    /** @return true when the rollers are loaded but not turning. */
    public boolean isJammed() {
        return rollerLoad.isJammed();
    }

    /** @return pieces detected since the last reset. A strong hint, not ground truth. */
    public int getPieceCount() {
        return pieceCounter.getCount();
    }

    /** Zeroes the piece count, e.g. at the start of a match. */
    public void resetPieceCount() {
        pieceCounter.reset();
    }

    /**
     * Tells the arm reaching its end of travel apart from it pushing a ball.
     *
     * <p>Both look identical to current alone. The discriminator is whether position freezes or keeps
     * creeping — see {@link HardStopDetector}. This is what makes {@link #isFullyDeployed()} mean
     * something more than "the encoder says so".
     */
    private final HardStopDetector deployStops = new HardStopDetector(
        "Intake/Deploy",
        LoadConstants.DEPLOY_FROZEN_BAND_ROTATIONS,
        LoadConstants.DEPLOY_PUSHING_AMPS,
        LoadConstants.DEPLOY_STOP_SUSTAIN_LOOPS);

    /** What the deploy motor was last told to do. The detector needs the direction. */
    private double deployDemand;

    /** How the deploy arm is currently being driven. */
    public enum DeployMode {
        /** Driven directly by an operator or a jostle. */
        MANUAL,
        /** Following a trapezoid profile toward a goal. */
        PROFILED
    }

    private DeployMode deployMode = DeployMode.MANUAL;

    /**
     * Goal last sent to the controller, so it is only re-sent when it actually changes.
     *
     * <p>This is the point of running the profile on the SPARK. A reference persists on the controller
     * until it is changed, so holding a position costs <b>zero</b> CAN frames — where a roboRIO-side
     * controller has to write an output every single loop, forever, including while merely holding
     * station. On a single bus with fifteen devices that difference is worth having.
     */
    private double lastCommandedGoal = Double.NaN;

    /**
     * Live-tunable profile and gain values.
     *
     * <p>The arm is a good candidate for live tuning: a move takes about half a second, so the effect
     * of a change is visible immediately, and a bad value gives a harsh move rather than anything
     * dangerous. Inert unless {@code TunableNumber.TUNING_ENABLED} is true.
     *
     * <p>A change here reconfigures the controller, which is a CAN write — so it is deliberately done
     * only when a value actually changes, and with no-persist so it does not burn flash. Reconfiguring
     * every loop would flood the bus, which is the thing running the profile on the SPARK is meant to
     * avoid.
     */
    private final TunableNumber tunableP =
        new TunableNumber("IntakeDeploy/kP", IntakeConstants.DEPLOY_kP);
    private final TunableNumber tunableD =
        new TunableNumber("IntakeDeploy/kD", IntakeConstants.DEPLOY_kD);
    private final TunableNumber tunableMaxVel =
        new TunableNumber("IntakeDeploy/MaxVelRps", IntakeConstants.DEPLOY_MAX_VELOCITY_RPS);
    private final TunableNumber tunableMaxAccel =
        new TunableNumber("IntakeDeploy/MaxAccelRps2", IntakeConstants.DEPLOY_MAX_ACCEL_RPS2);

    /**
     * Drives the arm toward a goal along a trapezoid profile.
     *
     * <p>The goal is clamped to the learned hard stops when they are known, so a target past the end of
     * travel becomes a target at the end of travel. Without that the profile drives into steel and
     * holds there — which is how a deploy target of 10 rotations against 9 rotations of real travel
     * turns into current draw for the rest of the match.
     *
     * @param goalRotations Desired arm position, in motor rotations.
     */
    public void deployToGoal(double goalRotations) {
        double goal = clampToLearnedStops(goalRotations);

        boolean modeChanged = deployMode != DeployMode.PROFILED;
        boolean goalChanged = Double.isNaN(lastCommandedGoal)
                || Math.abs(goal - lastCommandedGoal) > 1e-6;

        deployMode = DeployMode.PROFILED;

        if (!modeChanged && !goalChanged) {
            // Nothing to say. The controller is already executing this reference and re-sending it
            // every call would spend CAN frames to change nothing.
            return;
        }

        // setSetpoint, not setReference: the latter is deprecated in REVLib 2026.
        //
        // No arbitrary feedforward argument. Gravity is configured as kCos on the controller instead,
        // which is strictly better than computing a cosine in robot code and shipping it over CAN
        // every time the goal changes — the controller applies it continuously at its own rate, and it
        // stays correct while merely holding position, which is exactly when gravity matters most.
        deployMotor.getClosedLoopController().setSetpoint(
            goal, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);

        lastCommandedGoal = goal;
    }

    /**
     * @param goal Requested goal, in motor rotations.
     * @return the goal limited to the measured travel, if it has been measured.
     *
     *     <p>Uses the <em>learned</em> stops rather than the configured soft limits on purpose: the
     *     soft limits are expressed in encoder units and inherit any error in the encoder zero, while
     *     a learned stop is a physical reference.
     */
    private double clampToLearnedStops(double goal) {
        double low = deployStops.getLearnedStop(HardStopDetector.End.LOW);
        double high = deployStops.getLearnedStop(HardStopDetector.End.HIGH);

        double limited = goal;
        if (!Double.isNaN(low)) {
            limited = Math.max(limited, low);
        }
        if (!Double.isNaN(high)) {
            limited = Math.min(limited, high);
        }
        return limited;
    }

    /** Hands the arm back to direct control, so profile output stops fighting it. */
    private void leaveProfiledMode() {
        deployMode = DeployMode.MANUAL;
        // Forget the cached goal, or returning to the same goal later would be suppressed as
        // unchanged and the reference would never be re-sent.
        lastCommandedGoal = Double.NaN;
    }

    /**
     * Fills in the deploy arm's closed loop, profile and feedforward.
     *
     * <p>One method so the constructor and live tuning cannot drift apart — a tuning path that
     * configured a subset would silently reset whatever it omitted.
     *
     * <p><b>Everything is slot-explicit.</b> REVLib's single-argument overloads are deprecated in
     * favour of ones naming a {@code ClosedLoopSlot}, and being explicit is worth more than avoiding a
     * warning: the slot the config writes and the slot {@code setSetpoint} reads have to be the same
     * one, and a mismatch would leave the gains configured somewhere nothing consults.
     *
     * @param config The config to populate.
     */
    private void populateDeployClosedLoop(SparkMaxConfig config) {
        config.closedLoop
            .p(tunableP.get(), ClosedLoopSlot.kSlot0)
            .d(tunableD.get(), ClosedLoopSlot.kSlot0)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot0);

        // The controller's own feedforward, which in REVLib 2026 is a full arm model rather than a
        // single velocity term. Running it here means it is applied at the controller's rate and stays
        // correct between commands, with no robot-code loop involved at all.
        //
        // In VOLTS, which matters on a robot that runs anywhere from 6 to 16 V: a voltage feedforward
        // is divided by the measured bus internally, so it delivers what it asks for regardless of the
        // pack. The deprecated duty-cycle velocityFF would have quietly weakened as the battery drained
        // — the same class of error as everything else here that assumed 12 V.
        config.closedLoop.feedForward
            .kS(IntakeConstants.DEPLOY_kS, ClosedLoopSlot.kSlot0)
            .kV(IntakeConstants.DEPLOY_kV_VOLTS_PER_RPM, ClosedLoopSlot.kSlot0)
            .kA(IntakeConstants.DEPLOY_kA, ClosedLoopSlot.kSlot0)
            .kCos(IntakeConstants.DEPLOY_kG, ClosedLoopSlot.kSlot0)
            .kCosRatio(IntakeConstants.DEPLOY_COS_RATIO, ClosedLoopSlot.kSlot0);

        // MAXMotion wants velocity in RPM and acceleration in RPM per second, where the calibration
        // reports rotations per second. Converted here rather than stored in two unit systems.
        //
        // cruiseVelocity, not maxVelocity: REVLib 2026 deprecated the latter. The new name is the more
        // honest one anyway — it is the velocity the profile cruises AT, not a ceiling the mechanism is
        // prevented from exceeding.
        config.closedLoop.maxMotion
            .cruiseVelocity(tunableMaxVel.get() * 60.0, ClosedLoopSlot.kSlot0)
            .maxAcceleration(tunableMaxAccel.get() * 60.0, ClosedLoopSlot.kSlot0)
            .allowedProfileError(IntakeConstants.DEPLOY_TOLERANCE_ROTATIONS, ClosedLoopSlot.kSlot0);
    }

    /**
     * Re-applies the deploy closed loop after a live tuning change.
     *
     * <p>No-reset and no-persist: a tweak rather than a reconfiguration, so it leaves the current limit
     * and soft limits alone and does not burn a flash erase cycle. Persisting on every tuning nudge
     * would wear the controller out, and it is a CAN write either way — which is why it only happens
     * when a value actually changes.
     */
    private void applyDeployClosedLoop() {
        SparkMaxConfig config = new SparkMaxConfig();
        populateDeployClosedLoop(config);

        deployMotor.configure(config, ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters);

        // Re-send the reference: a reconfigure can clear the executing profile.
        lastCommandedGoal = Double.NaN;
    }

    /**
     * Applies a raw voltage to the deploy motor, leaving profiled mode.
     *
     * <p>For calibration only. Voltage rather than duty cycle because every gain the calibration
     * produces is in volts, and a duty cycle would rescale itself as the battery sags between 6 and
     * 16 V — the fit would be against a moving input.
     *
     * @param volts Output voltage.
     */
    public void setDeployVoltage(double volts) {
        leaveProfiledMode();
        deployDemand = volts / Math.max(1.0, deployMotor.getBusVoltage());
        deployMotor.setVoltage(volts);
    }

    /**
     * @return the voltage currently being applied to hold the arm where it is.
     *
     *     <p>Only meaningful while the controller is holding station. This is the direct measurement of
     *     what gravity and friction cost at this position — no angle, no reduction, no geometry needed.
     *     Whatever the controller has settled on IS the answer.
     */
    public double getDeployHoldVolts() {
        return deployMotor.getAppliedOutput() * deployMotor.getBusVoltage();
    }

    /** @return whether the arm is following a profile or being driven directly. */
    public DeployMode getDeployMode() {
        return deployMode;
    }

    /**
     * @return the goal the controller is driving toward, in motor rotations, or NaN if none is set.
     *
     *     <p>This is the goal <em>after</em> clamping to the learned stops, not what was requested — so
     *     it is what the arm will actually try to reach.
     */
    public double getDeployGoal() {
        return lastCommandedGoal;
    }

    /**
     * @return the arm's error against its goal, in motor rotations, or 0 if no goal is set.
     *
     *     <p>MAXMotion's internal profile setpoint is not reported over CAN, so what is available is
     *     error against the <em>goal</em> rather than against the moving setpoint. That is the more
     *     useful of the two anyway for judging whether the arm arrives; what it cannot show is
     *     following error <em>during</em> the move, which is the visibility given up by moving the
     *     profile onto the controller.
     */
    public double getDeployGoalError() {
        return Double.isNaN(lastCommandedGoal) ? 0.0 : lastCommandedGoal - getDeployPosition();
    }

    /** @return true when the arm has reached its goal within tolerance. */
    public boolean isDeployAtGoal() {
        return deployMode == DeployMode.PROFILED
            && !Double.isNaN(lastCommandedGoal)
            && Math.abs(getDeployGoalError()) <= IntakeConstants.DEPLOY_TOLERANCE_ROTATIONS;
    }

    /**
     * @return true when the arm is against its <b>deployed</b> hard stop.
     *
     *     <p>Distinct from {@link #isDeployed()}, which only asks whether the encoder is near the
     *     target. This asks whether the arm has physically run out of travel — so a ball wedged under
     *     the arm reports false here and {@link #isDeployPushingBall()} true, where the encoder check
     *     alone would call it arrived.
     */
    public boolean isFullyDeployed() {
        return deployStops.isAtHardStop() && deployDemand > 0;
    }

    /** @return true when the arm is against its <b>stowed</b> hard stop. */
    public boolean isFullyStowed() {
        return deployStops.isAtHardStop() && deployDemand < 0;
    }

    /**
     * @return true when the arm has stalled against something that is still moving — a ball.
     *
     *     <p>Actionable rather than informational: this is the case a jostle can clear, where a hard
     *     stop is not.
     */
    public boolean isDeployPushingBall() {
        return deployStops.isObstructed();
    }

    /**
     * @return how far the deploy encoder has drifted from where the stowed stop should be, or NaN if
     *     the arm is not currently against it.
     *
     *     <p>The constructor zeroes the encoder assuming the arm starts stowed. If the code ever
     *     starts with the arm part-way — a mid-match reboot, a brownout, someone moving it by hand —
     *     every position afterwards carries that offset, and <b>the soft limits carry it too</b>,
     *     because they are expressed in the same units. This is the measurement of that error.
     */
    public double getDeployEncoderDrift() {
        return deployStops.getEncoderDrift(
            HardStopDetector.End.LOW, IntakeConstants.STOW_POSITION_ROTATIONS);
    }

    /**
     * Re-zeroes the deploy encoder against the stowed hard stop.
     *
     * <p>Only acts while the arm is confirmed against that stop, because otherwise it would be
     * writing an assumption rather than a measurement — exactly the problem it exists to fix.
     *
     * @return true if the encoder was corrected.
     */
    public boolean rezeroDeployAtStowedStop() {
        if (!isFullyStowed()) {
            return false;
        }

        double drift = getDeployEncoderDrift();
        deployEncoder.setPosition(IntakeConstants.STOW_POSITION_ROTATIONS);
        System.out.printf("[intake] deploy encoder re-zeroed at the stowed stop, correcting %+.3f "
            + "rotations%n", drift);
        return true;
    }

    /** @return the hard stop detector, for calibration and diagnostics. */
    public HardStopDetector getDeployStops() {
        return deployStops;
    }

    /** @return the roller load monitor, for jam-clearing routines and diagnostics. */
    public MotorLoadMonitor getRollerLoad() {
        return rollerLoad;
    }

    @Override
    public void periodic() {
        rollerLoad.update(getRollerCurrent(), getRollerVelocity(), intakeRunning);
        pieceCounter.update();
        // No-ops entirely when tuning is disabled. Each reconfigures the controller, so they only
        // fire on an actual change.
        tunableP.ifChanged(p -> applyDeployClosedLoop());
        tunableD.ifChanged(d -> applyDeployClosedLoop());
        tunableMaxVel.ifChanged(v -> applyDeployClosedLoop());
        tunableMaxAccel.ifChanged(a -> applyDeployClosedLoop());

        // Nothing to write here. MAXMotion executes the reference on the controller until it is
        // changed, so holding position costs no CAN traffic at all — and the profile stops with the
        // controller on disable, then restarts from wherever the arm actually is. The
        // runs-away-while-disabled problem a roboRIO-side profile has simply cannot arise.
        //
        // What does have to be forgotten is the cached goal, so the reference is re-sent on the next
        // request rather than suppressed as unchanged.
        // Forget the cached goal while disabled, so the reference is re-sent rather than suppressed as
        // unchanged when the robot comes back. Nothing else is needed: the profile lives on the
        // controller, which stops on disable and restarts from wherever the arm actually is — the
        // run-away-while-disabled problem a robot-code profile has cannot arise here at all.
        if (!DriverStation.isEnabled() && deployMode == DeployMode.PROFILED) {
            lastCommandedGoal = Double.NaN;
        }

        deployStops.update(getDeployPosition(), getDeployCurrent(), deployDemand);
        Logger.recordOutput(getName() + "/Deploy/Mode", deployMode.name());
        Logger.recordOutput(getName() + "/Deploy/Goal", getDeployGoal());
        Logger.recordOutput(getName() + "/Deploy/GoalError", getDeployGoalError());
        Logger.recordOutput(getName() + "/Deploy/AtGoal", isDeployAtGoal());

        Logger.recordOutput(getName() + "/Deploy/FullyDeployed", isFullyDeployed());
        Logger.recordOutput(getName() + "/Deploy/FullyStowed", isFullyStowed());
        Logger.recordOutput(getName() + "/Deploy/PushingBall", isDeployPushingBall());
        Logger.recordOutput(getName() + "/Deploy/EncoderDrift", getDeployEncoderDrift());

        Logger.recordOutput(getName() + "/Encoder/Position", getDeployPosition());
        Logger.recordOutput(getName() + "/Deploy/OutputCurrent", deployMotor.getOutputCurrent());
        Logger.recordOutput(getName() + "/Deploy/Deployed", isDeployed());
        Logger.recordOutput(getName() + "/Deploy/Stowed", isStowed());
        Logger.recordOutput(getName() + "/Activity/Intake", intakeRunning);
        Logger.recordOutput(getName() + "/Roller1/OutputCurrent", intakeMotor1.getOutputCurrent());
        Logger.recordOutput(getName() + "/Roller2/OutputCurrent", intakeMotor2.getOutputCurrent());
    }
}
