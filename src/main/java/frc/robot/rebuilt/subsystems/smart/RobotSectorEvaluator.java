package frc.robot.rebuilt.subsystems.smart;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.common.subsystems.DashboardSubsystem;
import frc.robot.common.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.rebuilt.components.RobotSector;
import org.littletonrobotics.junction.Logger;

public class RobotSectorEvaluator extends DashboardSubsystem {
    private final RobotSector[] sectorArr = new RobotSector[999];
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
                    (Math.abs(robotSector.center.getY() - pose.getY()) < robotSector.height)) {
                return robotSector;
            }
        }
        return new RobotSector(RobotSector.BaseSector.NONE, null, new Pose2d(67.,67.,new Rotation2d()), 0,0,-1);
    }

    public void createSector(RobotSector.BaseSector base , RobotSector.SectorType type, Pose2d center, double width, double hight){
           sectorArr[sectorArrPtr] = new RobotSector(base, type, center, width, hight,sectorArrPtr);
           sectorArrPtr++;
    }
    @Override
    public void periodic() {
        updateAdvantage();
    }

    public void updateAdvantage(){
        RobotSector sector = getSector();
        Logger.recordOutput("/RobotSectorEvaluator/CurrentSector", sector.center);
        Logger.recordOutput("/RobotSectorEvaluator/CurrentSector/info/id", sector.id);
        Logger.recordOutput("/RobotSectorEvaluator/CurrentSector/info/base", sector.base);
        Logger.recordOutput("/RobotSectorEvaluator/CurrentSector/info/type", sector.type);
        Pose2d[] poseArr = new Pose2d[5];
        poseArr[0]=sector.center.plus(new Transform2d(-sector.width, sector.height,new Rotation2d()));
        poseArr[1]=sector.center.plus(new Transform2d(sector.width, sector.height,new Rotation2d()));
        poseArr[2]=sector.center.plus(new Transform2d(sector.width,-sector.height,new Rotation2d()));
        poseArr[3]=sector.center.plus(new Transform2d(-sector.width,-sector.height,new Rotation2d()));
        poseArr[4]=sector.center.plus(new Transform2d(-sector.width, sector.height,new Rotation2d()));




        Logger.recordOutput("/RobotSectorEvaluator/CurrentSector/Bounds",poseArr);

    }

}
