package frc.robot.common.components.hardware;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.common.interfaces.IMU;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.common.swerve.RAWRSwerveModule;


/**
 * Drive hardware for a full robot with swerve drive
 *
 * @author PurpleLib
 * @author Alan Trinh
 * @author Hudson Strub
 *
 * @since 2025
 */
public record SwerveHardware(IMU gyro, RAWRSwerveModule lFrontModule, RAWRSwerveModule rFrontModule,
                             RAWRSwerveModule lRearModule, RAWRSwerveModule rRearModule) {

    public SwerveHardware {
        while (gyro.isCalibrating()) {
           if(!RobotBase.isSimulation()){ //Would Crash in Sim
               stop(); // Stops all modules while gyro calibrates
           }
        }
        gyro.reset();
    }

    public void lock() {
        lFrontModule.lock();
        rFrontModule.lock();
        lRearModule.lock();
        rRearModule.lock();
    }

    /**
     * Set swerve modules
     * @param moduleStates Array of calculated module states
     */
    public void setSwerveModules(SwerveModuleState[] moduleStates) {
        lFrontModule.set(moduleStates);
        rFrontModule.set(moduleStates);
        lRearModule.set(moduleStates);
        rRearModule.set(moduleStates);
    }

    /**
     * Get current module states
     * @return Array of swerve module states
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                lFrontModule.getState(),
                rFrontModule.getState(),
                lRearModule.getState(),
                rRearModule.getState()
        };
    }

    /**
     * Get current module positions
     * @return Array of swerve module positions
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                lFrontModule.getPosition(),
                rFrontModule.getPosition(),
                lRearModule.getPosition(),
                rRearModule.getPosition()
        };
    }

    public void stop() {
        lFrontModule.stop();
        rFrontModule.stop();
        lRearModule.stop();
        rRearModule.stop();
    }

    public void toggleTractionControl() {
        lFrontModule.toggleTractionControl();
        rFrontModule.toggleTractionControl();
        lRearModule.toggleTractionControl();
        rRearModule.toggleTractionControl();
    }

    public void enableTractionControl() {
        lFrontModule.enableTractionControl();
        rFrontModule.enableTractionControl();
        lRearModule.enableTractionControl();
        rRearModule.enableTractionControl();
    }

    public void disableTractionControl() {
        lFrontModule.disableTractionControl();
        rFrontModule.disableTractionControl();
        lRearModule.disableTractionControl();
        rRearModule.disableTractionControl();
    }

    public void close() {
        try { // TODO EVIL
            gyro.close();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        lFrontModule.close();
        rFrontModule.close();
        lRearModule.close();
        rRearModule.close();
    }

    public Translation2d[] getModuleCoordinates() {
        return new Translation2d[]{
                lFrontModule.getModuleCoordinate(),
                rFrontModule.getModuleCoordinate(),
                lRearModule.getModuleCoordinate(),
                rRearModule.getModuleCoordinate()
        };
    }
}