package frc.robot.common.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import lombok.Getter;

public class MAXSwerveDrivetrain {

    @Getter private final MAXSwerveModule frontLeft;
    @Getter private final MAXSwerveModule frontRight;
    @Getter private final MAXSwerveModule rearLeft;
    @Getter private final MAXSwerveModule rearRight;

    public MAXSwerveDrivetrain(
            int flDrive, int flTurn, double flOffset,
            int frDrive, int frTurn, double frOffset,
            int rlDrive, int rlTurn, double rlOffset,
            int rrDrive, int rrTurn, double rrOffset) {

        frontLeft = new MAXSwerveModule(flDrive, flTurn, flOffset);
        frontRight = new MAXSwerveModule(frDrive, frTurn, frOffset);
        rearLeft = new MAXSwerveModule(rlDrive, rlTurn, rlOffset);
        rearRight = new MAXSwerveModule(rrDrive, rrTurn, rrOffset);
    }

    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition()
        };
    }

    public void setStates(SwerveModuleState[] states) {
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        rearLeft.setDesiredState(states[2]);
        rearRight.setDesiredState(states[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        frontLeft.resetEncoders();
        rearLeft.resetEncoders();
        frontRight.resetEncoders();
        rearRight.resetEncoders();
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setXLock() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

}
