package frc.robot.rebuilt.components;

import edu.wpi.first.math.geometry.Pose2d;

import javax.annotation.Nullable;
import java.util.Objects;

public class RobotSector {//
    public enum BaseSector {//required
        RED, BLUE, NONE
    }
    public enum SectorType {//add as needed
        //Sector types have qualities attributed to them such as able to shoot, able to climb
        TOWER,BUMP,NONE
    }
    //base and type are used to evaluate what a robot can do while inside of a sector

    public BaseSector base;
    public SectorType type;
    public Pose2d center;
    public double width;
    public double height;
    public int id;

    public RobotSector(BaseSector base, @Nullable SectorType type, Pose2d center, double width, double height, int id) {
        this.base = base;

        this.type = Objects.requireNonNullElse(type, SectorType.NONE);
        this.center = center;
        this.width = width;
        this.height = height;
        this.id = id;
    }
}
