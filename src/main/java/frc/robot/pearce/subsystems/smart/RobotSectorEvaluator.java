package frc.robot.pearce.subsystems.smart;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.common.subsystems.DashboardSubsystem;
import frc.robot.common.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.pearce.components.RobotSector;
import org.littletonrobotics.junction.Logger;

public class RobotSectorEvaluator extends DashboardSubsystem {
    private RobotSector sectorArr[] = new RobotSector[10];
    private int sectorArrPtr = 0;
    SwerveDriveSubsystem drive;

    public RobotSectorEvaluator(SwerveDriveSubsystem drive) {
    this.drive = drive;
    }
    public RobotSector getSector(){ //
        Pose2d pose = drive.getPose();
        for (RobotSector robotSector : sectorArr) {
            if(robotSector == null) break;
            if ((Math.abs(robotSector.center.getX() - pose.getX()) < robotSector.width) &&
                    (Math.abs(robotSector.center.getY() - pose.getY()) < robotSector.hight)) {
                return robotSector;
            }
        }
        return new RobotSector(RobotSector.baseSector.NONE, RobotSector.sectorType.NONE,new Pose2d(67.,67.,new Rotation2d()), 0,0,-1);
    }

    public boolean createSector(RobotSector.baseSector base , RobotSector.sectorType type, Pose2d center, double width, double hight){
       try {
           sectorArr[sectorArrPtr] = new RobotSector(base, type, center, width, hight,sectorArrPtr);
           sectorArrPtr++;
       } catch (Exception e) {
           return false;
       }
       return true;
    }
    @Override
    public void periodic() {
        getSector().updateAdvantage();
    }

}
