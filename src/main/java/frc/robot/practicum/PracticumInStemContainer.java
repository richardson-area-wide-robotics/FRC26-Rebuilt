package frc.robot.practicum;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import frc.robot.CommonConstants;
import frc.robot.common.components.EasyMotor;
import frc.robot.common.components.hardware.TankHardware;
import frc.robot.common.gyro.RAWRNavX2;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;
import frc.robot.common.interfaces.IRobotContainer;
import frc.robot.common.subsystems.drive.TankDriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommonConstants.HIDConstants;
import frc.robot.common.annotations.Robot;

import java.util.Collections;

@NoArgsConstructor(access = AccessLevel.PRIVATE)
@Robot(team = 9991)
public class PracticumInStemContainer implements IRobotContainer {

  public static final TankDriveSubsystem DRIVE_SUBSYSTEM = new TankDriveSubsystem(
          new TankHardware(
                  new RAWRNavX2(CommonConstants.DriveHardwareConstants.NAVX_NAME),
                  Collections.singletonList(EasyMotor.createEasySparkMax(1, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kCoast)),
                  Collections.singletonList(EasyMotor.createEasySparkMax(2, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kCoast))));


    public static IRobotContainer createContainer(){
      configureBindings();

      return new PracticumInStemContainer();
    }

    private static void configureBindings() {
      DRIVE_SUBSYSTEM.setDefaultCommand(
              DRIVE_SUBSYSTEM.arcadeDriveCommand(
                      HIDConstants.DRIVER_CONTROLLER::getLeftY,  // forward/back
                      HIDConstants.DRIVER_CONTROLLER::getRightX  // turning
              )
      );

    }
    
    
    @Override    
    public Command getAutonomousCommand() {
        return null;
    }

    @Override
    public void simulationPeriodic() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
    }
    
}
