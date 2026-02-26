package frc.robot.rebuilt.subsystems.smart;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import lombok.AllArgsConstructor;

public class ScoringLocationLookup {
    private static final ScoringLocationFinalized[] scoringLocationsRed = new ScoringLocationFinalized[8];
    private static final ScoringLocationFinalized[] scoringLocationsBlue = new ScoringLocationFinalized[8];
    public static Boolean team = null;



    public static void buildScoringLocations(){
        ScoringLocation[] scoringLocationsRaw = new ScoringLocation[8];

        scoringLocationsRaw[0] = new ScoringLocation("right_corner", new Pose2d(16,7.5, new Rotation2d()), new Pose2d(0.5,7.5, new Rotation2d()));
        scoringLocationsRaw[1] = new ScoringLocation("left_corner", new Pose2d(16,0.5, new Rotation2d()), new Pose2d(0.5,0.5, new Rotation2d()));
        scoringLocationsRaw[2] = new ScoringLocation("left_trench", new Pose2d(13.2,0.5, new Rotation2d()), new Pose2d(3.5,7.5, new Rotation2d()));
        scoringLocationsRaw[3] = new ScoringLocation("right_trench", new Pose2d(13.2,7.5, new Rotation2d()), new Pose2d(3.5,0.5, new Rotation2d()));
        scoringLocationsRaw[4] = new ScoringLocation("climber", new Pose2d(14.5,4, new Rotation2d()), new Pose2d(2,3.5, new Rotation2d()));
        scoringLocationsRaw[5] = new ScoringLocation("rightside_hub", new Pose2d(13,5, new Rotation2d()), new Pose2d(3.5,5, new Rotation2d()));
        scoringLocationsRaw[6] = new ScoringLocation("hub", new Pose2d(13,4, new Rotation2d()), new Pose2d(3.5,4, new Rotation2d()));
        scoringLocationsRaw[7] = new ScoringLocation("leftside_hub", new Pose2d(13,3, new Rotation2d()), new Pose2d(3.5,3, new Rotation2d()));


        for (int i = 0; i < scoringLocationsRaw.length; i++) {
            ScoringLocation scoringLocation = scoringLocationsRaw[i];
            scoringLocationsRed[i] = new ScoringLocationFinalized(scoringLocation.name, scoringLocation.redPose2d);
            scoringLocationsBlue[i] = new ScoringLocationFinalized(scoringLocation.name, scoringLocation.bluePose2d);

        }

    }
    public static Pose2d findClosest(Pose2d robotPose){//false for blue
        if(team== null) return new Pose2d();
        double closestDist = 999;
        int closestI = -1;

        ScoringLocationFinalized[] arrayInQuestion;


        if(team){
            arrayInQuestion = scoringLocationsRed;
        }
        else{
            arrayInQuestion = scoringLocationsBlue;
        }


        for(int i = 0; i<arrayInQuestion.length;i++){
            ScoringLocationFinalized scoringLocationFinalized = arrayInQuestion[i];
            if(robotPose.getTranslation().getDistance(scoringLocationFinalized.finalizedPose2d.getTranslation()) < closestDist){
                closestDist = robotPose.getTranslation().getDistance(scoringLocationFinalized.finalizedPose2d.getTranslation());
                closestI = i;
            };
        }
        return arrayInQuestion[closestI].finalizedPose2d;

        }


    @AllArgsConstructor
    static class ScoringLocation {
        public String name;
        public Pose2d redPose2d;
        public Pose2d bluePose2d;

    }

    @AllArgsConstructor
    static class ScoringLocationFinalized {
        public String name;
        public Pose2d finalizedPose2d;

    }

}