package frc.robot.common.components.hardware.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.CommonConstants;
import frc.robot.CommonConstants.ModuleConstants;
import lombok.Getter;

public class MAXSwerveModule {


    @Getter private final SparkFlex drivingSpark;
    @Getter private final SparkMax turningSpark;

    @Getter private final RelativeEncoder drivingEncoder;
    @Getter private final AbsoluteEncoder turningEncoder;

    private final SparkClosedLoopController drivingClosedLoopController;
    private final SparkClosedLoopController turningClosedLoopController;

    private final double chassisAngularOffset;
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {

        drivingSpark = new SparkFlex(drivingCANId, MotorType.kBrushless);
        turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

        drivingEncoder = drivingSpark.getEncoder();
        turningEncoder = turningSpark.getAbsoluteEncoder();

        drivingClosedLoopController = drivingSpark.getClosedLoopController();
        turningClosedLoopController = turningSpark.getClosedLoopController();

        SparkMaxConfig drivingConfig = new SparkMaxConfig();
        SparkMaxConfig turningConfig = new SparkMaxConfig();

        double drivingFactor =
                ModuleConstants.kWheelDiameterMeters * Math.PI
                        / ModuleConstants.kDrivingMotorReduction;

        double turningFactor = 2 * Math.PI;
        double nominalVoltage = 12.0;

        double drivingVelocityFeedForward =
                nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

        // Driving config
        drivingConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50);

        drivingConfig.encoder
                .positionConversionFactor(drivingFactor)
                .velocityConversionFactor(drivingFactor / 60.0);

        drivingConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(CommonConstants.DriveConstants.drivePID.getKP(), CommonConstants.DriveConstants.drivePID.getKI(), CommonConstants.DriveConstants.drivePID.getKD())
                .outputRange(-1, 1)
                .feedForward.kV(drivingVelocityFeedForward);

        // Turning config
        turningConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20);

        turningConfig.absoluteEncoder
                .inverted(true)
                .positionConversionFactor(turningFactor)
                .velocityConversionFactor(turningFactor / 60.0)
                .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

        turningConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(CommonConstants.DriveConstants.turnPID.getKP(), CommonConstants.DriveConstants.turnPID.getKI(), CommonConstants.DriveConstants.turnPID.getKD())
                .outputRange(-1, 1)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, turningFactor);

        // Apply configs
        drivingSpark.configure(drivingConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        turningSpark.configure(turningConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        this.chassisAngularOffset = chassisAngularOffset;
        desiredState.angle = new Rotation2d(turningEncoder.getPosition());
        drivingEncoder.setPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                drivingEncoder.getVelocity(),
                new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                drivingEncoder.getPosition(),
                new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle =
                desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

        correctedDesiredState.optimize(new Rotation2d(turningEncoder.getPosition()));

        drivingClosedLoopController.setSetpoint(
                correctedDesiredState.speedMetersPerSecond,
                ControlType.kVelocity);

        turningClosedLoopController.setSetpoint(
                correctedDesiredState.angle.getRadians(),
                ControlType.kPosition);

        this.desiredState = desiredState;
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        drivingEncoder.setPosition(0);
    }
}