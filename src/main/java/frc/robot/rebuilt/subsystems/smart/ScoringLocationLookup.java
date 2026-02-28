package frc.robot.rebuilt.subsystems.smart;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import lombok.AllArgsConstructor;

public class ScoringLocationLookup {
    private static final ScoringLocation[] scoringLocations = new ScoringLocation[8];
    public static Boolean team = null;



    public static void buildScoringLocations(){

        scoringLocations[0] = new ScoringLocation("right_corner", new Pose2d(16,7.5, new Rotation2d()), new Pose2d(0.5,7.5, new Rotation2d()));
        scoringLocations[1] = new ScoringLocation("left_corner", new Pose2d(16,0.5, new Rotation2d()), new Pose2d(0.5,0.5, new Rotation2d()));
        scoringLocations[2] = new ScoringLocation("left_trench", new Pose2d(13.2,0.5, new Rotation2d()), new Pose2d(3.5,7.5, new Rotation2d()));
        scoringLocations[3] = new ScoringLocation("right_trench", new Pose2d(13.2,7.5, new Rotation2d()), new Pose2d(3.5,0.5, new Rotation2d()));
        scoringLocations[4] = new ScoringLocation("climber", new Pose2d(14.5,4, new Rotation2d()), new Pose2d(2,3.5, new Rotation2d()));
        scoringLocations[5] = new ScoringLocation("rightside_hub", new Pose2d(13,5, new Rotation2d()), new Pose2d(3.5,5, new Rotation2d()));
        scoringLocations[6] = new ScoringLocation("hub", new Pose2d(13,4, new Rotation2d()), new Pose2d(3.5,4, new Rotation2d()));
        scoringLocations[7] = new ScoringLocation("leftside_hub", new Pose2d(13,3, new Rotation2d()), new Pose2d(3.5,3, new Rotation2d()));

    }
    public static Pose2d findClosest(Pose2d robotPose){//false for blue
        if(team== null) return new Pose2d();
        double closestDist = 999;
        Pose2d closestPose = new Pose2d();


        if(team){
            for (ScoringLocation scoringLocationFinalized : scoringLocations) {
                if (robotPose.getTranslation().getDistance(scoringLocationFinalized.redPose2d.getTranslation()) < closestDist) {
                    closestDist = robotPose.getTranslation().getDistance(scoringLocationFinalized.redPose2d.getTranslation());
                    closestPose = scoringLocationFinalized.redPose2d;

                }
            }
        }
        else{
            for (ScoringLocation scoringLocationFinalized : scoringLocations) {
                if (robotPose.getTranslation().getDistance(scoringLocationFinalized.bluePose2d.getTranslation()) < closestDist) {
                    closestDist = robotPose.getTranslation().getDistance(scoringLocationFinalized.bluePose2d.getTranslation());
                    closestPose = scoringLocationFinalized.bluePose2d;
                }
            }
        }



        return closestPose;

        }


    @AllArgsConstructor
    static class ScoringLocation {
        public String name;
        public Pose2d redPose2d;
        public Pose2d bluePose2d;

    }

}