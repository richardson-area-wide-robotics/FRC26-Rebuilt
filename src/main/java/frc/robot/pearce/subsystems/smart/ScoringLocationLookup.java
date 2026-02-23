package frc.robot.pearce.subsystems.smart;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import lombok.AllArgsConstructor;

public class ScoringLocationLookup {
    private static final ScoringLocationFinalized[] scoringLocationsRed = new ScoringLocationFinalized[8];
    private static final ScoringLocationFinalized[] scoringLocationsBlue = new ScoringLocationFinalized[8];
    public static Boolean team = null;


    private static final ScoringLocation[] SCORING_LOCATIONS = {
            new ScoringLocation("right_corner", new Pose2d(16,7.5,new Rotation2d()), new Pose2d(0.5,7.5,new Rotation2d())),
            new ScoringLocation("left_corner", new Pose2d(16,0.5,new Rotation2d()), new Pose2d(0.5,0.5,new Rotation2d())),
            new ScoringLocation("left_trench", new Pose2d(13.2,0.5,new Rotation2d()), new Pose2d(3.5,7.5,new Rotation2d())),
            new ScoringLocation("right_trench", new Pose2d(13.2,7.5,new Rotation2d()), new Pose2d(3.5,0.5,new Rotation2d())),
            new ScoringLocation("climber", new Pose2d(14.5,4,new Rotation2d()), new Pose2d(2,3.5,new Rotation2d())),
            new ScoringLocation("rightside_hub", new Pose2d(13,5,new Rotation2d()), new Pose2d(3.5,5,new Rotation2d())),
            new ScoringLocation("hub", new Pose2d(13,4,new Rotation2d()), new Pose2d(3.5,4,new Rotation2d())),
            new ScoringLocation("leftside_hub", new Pose2d(13,3,new Rotation2d()), new Pose2d(3.5,3,new Rotation2d()))
    };

    static { //This will happen when the class is first loaded
        for (int i = 0; i < SCORING_LOCATIONS.length; i++) {
            ScoringLocation loc = SCORING_LOCATIONS[i];
            scoringLocationsRed[i] = new ScoringLocationFinalized(loc.name, loc.redPose2d);
            scoringLocationsBlue[i] = new ScoringLocationFinalized(loc.name, loc.bluePose2d);
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


        for (int i = 0; i < arrayInQuestion.length; i++) {
            Pose2d pose = arrayInQuestion[i].finalizedPose2d;
            double dist = robotPose.getTranslation().getDistance(pose.getTranslation());

            if (dist < closestDist) {
                closestDist = dist;
                closestI = i;
            }
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