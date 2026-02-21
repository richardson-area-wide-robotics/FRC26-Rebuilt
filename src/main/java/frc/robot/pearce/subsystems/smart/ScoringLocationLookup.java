package frc.robot.pearce.subsystems.smart;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.pearce.components.RobotSector;
import lombok.AllArgsConstructor;

public class ScoringLocationLookup {
    private ScoringLocation scoringLocations[] = new ScoringLocation[999];


    public ScoringLocation[] buildScoreingLocations(){
        ScoringLocation scoringLocations[] = new ScoringLocation[999];


        scoringLocations[0] = new ScoringLocation("hub", new Pose2d(13,4, new Rotation2d()), null);
        scoringLocations[1] = new ScoringLocation("left_corner", new Pose2d(16,0.5, new Rotation2d()), null);
        scoringLocations[2] = new ScoringLocation("climber", new Pose2d(14.5,4, new Rotation2d()), null);
        scoringLocations[3] = new ScoringLocation("right_corner", new Pose2d(16,7.5, new Rotation2d()), null);
        scoringLocations[4] = new ScoringLocation("rightside_hub", new Pose2d(13,5, new Rotation2d()), null);
        scoringLocations[4] = new ScoringLocation("leftside_hub", new Pose2d(13,3, new Rotation2d()), null);




        return scoringLocations;
    }


    @AllArgsConstructor
    static class ScoringLocation {
        public String name;
        public Pose2d redPose2d;
        public Pose2d bluePose2d;

    }

}