package frc.robot.pearce.subsystems.smart;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.common.subsystems.DashboardSubsystem;
import frc.robot.common.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.pearce.components.RobotSector;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class RobotSectorEvaluator extends DashboardSubsystem {
    private RobotSector sectorArr[] = new RobotSector[10];
    private int sectorArrPtr = 0;
    SwerveDriveSubsystem drive;


    // AdvantageKit visualization
    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismLigament2d mechanismLigament;

    public RobotSectorEvaluator(SwerveDriveSubsystem drive) {
<<<<<<< Updated upstream
    this.drive = drive;

        mechanism = new LoggedMechanism2d(2.0, 2.0);

        LoggedMechanismRoot2d root =
                mechanism.getRoot("ClimberRoot", 1.0, 0.0);

        mechanismLigament = root.append(
                new LoggedMechanismLigament2d(
                        "Climber",
                        3,
                        90
                )
        );
=======
        this.drive = drive;
>>>>>>> Stashed changes
    }

    public RobotSector getSector() { //
        Pose2d pose = drive.getPose();
        for (RobotSector robotSector : sectorArr) {
            if (robotSector == null) break;
            if ((Math.abs(robotSector.center.getX() - pose.getX()) < robotSector.width) &&
                    (Math.abs(robotSector.center.getY() - pose.getY()) < robotSector.hight)) {
                return robotSector;
            }
        }
        return new RobotSector(RobotSector.baseSector.NONE, RobotSector.sectorType.NONE, new Pose2d(67., 67., new Rotation2d()), 0, 0, -1);
    }

    public boolean createSector(RobotSector.baseSector base, RobotSector.sectorType type, Pose2d center, double width, double hight) {
        try {
            sectorArr[sectorArrPtr] = new RobotSector(base, type, center, width, hight, sectorArrPtr);
            sectorArrPtr++;
        } catch (Exception e) {
            return false;
        }
        return true;
    }

    @Override
    public void periodic() {
<<<<<<< Updated upstream
        if(getSector().center != null){
            Logger.recordOutput(getName() +"/Sector ", getSector().center);


            mechanismLigament.setLength(getSector().hight);

            Logger.recordOutput(getName() + "/3d", mechanism);
        }

=======
        if (getSector().center != null){
            Logger.recordOutput(getName() + "/CurrentSector/info/center ", getSector().center);
            Logger.recordOutput(getName() + "/CurrentSector/info/width ", getSector().width);
            Logger.recordOutput(getName() + "/CurrentSector/info/hight ", getSector().hight);
            RobotSector.updateAdvantage(getSector());
        }
>>>>>>> Stashed changes
    }
}
