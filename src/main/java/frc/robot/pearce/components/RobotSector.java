package frc.robot.pearce.components;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.common.subsystems.drive.SwerveDriveSubsystem;
import org.littletonrobotics.junction.Logger;

public class RobotSector {//
    public enum baseSector {//required
        RED, BLUE, NONE
    }
    public enum sectorType {//add as needed
        //Sector types have qualities attributed to them such as able to shoot, able to climb
        TOWER,BUMP,NONE
    }
    //base and type are used to evaluate what a robot can do while inside of a sector

    public baseSector base;
    public sectorType type;
    public Pose2d center;
    public double width;
    public double hight;
    int id;

    public RobotSector(baseSector base ,sectorType type, Pose2d center, double width, double hight, int id) {
        this.base = base;
        this.type = type;
        this.center = center;
        this.width = width;
        this.hight = hight;
        this.id = id;
    }
    public void updateAdvantage(){
        Logger.recordOutput("/RobotSectorEvaluator/CurrentSector", center);
        Logger.recordOutput("/RobotSectorEvaluator/CurrentSector/info/id", id);
        Logger.recordOutput("/RobotSectorEvaluator/CurrentSector/info/base", base);
        Logger.recordOutput("/RobotSectorEvaluator/CurrentSector/info/type", type);
        Pose2d[] poseArr = new Pose2d[5];
        poseArr[0]=center.plus(new Transform2d(-width,hight,new Rotation2d()));
        poseArr[1]=center.plus(new Transform2d(width,hight,new Rotation2d()));
        poseArr[2]=center.plus(new Transform2d(width,-hight,new Rotation2d()));
        poseArr[3]=center.plus(new Transform2d(-width,-hight,new Rotation2d()));
        poseArr[4]=center.plus(new Transform2d(-width,hight,new Rotation2d()));




        Logger.recordOutput("/RobotSectorEvaluator/CurrentSector/Bounds",poseArr);
        //Logger.recordOutput("/RobotSectorEvaluator/CurrentSector/Bounds/0", center.plus(new Transform2d(-width,-hight,new Rotation2d())));
        //Logger.recordOutput("/RobotSectorEvaluator/CurrentSector/Bounds/1", center.plus(new Transform2d(-width,hight,new Rotation2d())));
        //Logger.recordOutput("/RobotSectorEvaluator/CurrentSector/Bounds/2", center.plus(new Transform2d(width,-hight,new Rotation2d())));
        //Logger.recordOutput("/RobotSectorEvaluator/CurrentSector/Bounds/3", center.plus(new Transform2d(width,hight,new Rotation2d())));

    }
}
