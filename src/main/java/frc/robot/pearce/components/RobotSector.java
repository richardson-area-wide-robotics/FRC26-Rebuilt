package frc.robot.pearce.components;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.common.subsystems.drive.SwerveDriveSubsystem;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class RobotSector {//
    public enum baseSector {//required
        RED, BLUE, NONE
    }
    public enum sectorType {//add as needed
        //Sector types have qualities attributed to them such as able to shoot, able to climb
        TOWER,BUMP,NONE
    }
    //base and type are used to evaluate what a robot can do while inside of a sector

    private static LoggedMechanism2d sectorMech0;
    private static LoggedMechanismLigament2d sectorLig0;
    private static LoggedMechanism2d sectorMech1;
    private static LoggedMechanismLigament2d sectorLig1;

    public baseSector base;
    public sectorType type;
    public Pose2d center;
    public double width;
    public double hight;
    public int id;

    public RobotSector(baseSector base ,sectorType type, Pose2d center, double width, double hight, int id) {
        this.base = base;
        this.type = type;
        this.center = center;
        this.width = width;
        this.hight = hight;
        this.id = id;
    }

    public static void updateAdvantage(RobotSector sector){
        sectorMech0 = new LoggedMechanism2d(0, 0);
        LoggedMechanismRoot2d root0 =
                sectorMech0.getRoot("SectorRoot"+sector.id, -sector.width, 0);

        sectorLig0 = root0.append(
                new LoggedMechanismLigament2d(
                        "SectorLig0_"+sector.id,
                        2* sector.width,
                        0
                )
        );


        sectorMech1 = new LoggedMechanism2d(0, 0);
        LoggedMechanismRoot2d root1 =
                sectorMech1.getRoot("SectorRoot"+sector.id, -sector.hight, 0);

        sectorLig1 = root1.append(
                new LoggedMechanismLigament2d(
                        "SectorLig1_"+sector.id,
                        2* sector.hight,
                        0
                )
        );




        Logger.recordOutput("/RobotSectorEvaluator/CurrentSector/mech/0", sectorMech0);
        Logger.recordOutput("/RobotSectorEvaluator/CurrentSector/mech/1", sectorMech1);
    }
}
