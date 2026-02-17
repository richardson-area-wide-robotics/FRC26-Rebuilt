package frc.robot.pearce.components;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.common.subsystems.drive.SwerveDriveSubsystem;

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

    public RobotSector(baseSector base ,sectorType type, Pose2d center, double width, double hight) {
        this.base = base;
        this.type = type;
        this.center = center;
        this.width = width;
        this.hight = hight;
    }
}
