package frc.robot.common.swerve;

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

public class MAXSwerveModule {

    private final SparkFlex m_drivingSpark;
    private final SparkMax m_turningSpark;

    private final RelativeEncoder m_drivingEncoder;
    private final AbsoluteEncoder m_turningEncoder;

    private final SparkClosedLoopController m_drivingClosedLoopController;
    private final SparkClosedLoopController m_turningClosedLoopController;

    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {

        m_drivingSpark = new SparkFlex(drivingCANId, MotorType.kBrushless);
        m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

        m_drivingEncoder = m_drivingSpark.getEncoder();
        m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

        m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
        m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

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
        m_drivingSpark.configure(drivingConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_turningSpark.configure(turningConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_chassisAngularOffset = chassisAngularOffset;
        m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
        m_drivingEncoder.setPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                m_drivingEncoder.getVelocity(),
                new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                m_drivingEncoder.getPosition(),
                new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
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
                desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

        correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

        m_drivingClosedLoopController.setSetpoint(
                correctedDesiredState.speedMetersPerSecond,
                ControlType.kVelocity);

        m_turningClosedLoopController.setSetpoint(
                correctedDesiredState.angle.getRadians(),
                ControlType.kPosition);

        m_desiredState = desiredState;
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        m_drivingEncoder.setPosition(0);
    }
}