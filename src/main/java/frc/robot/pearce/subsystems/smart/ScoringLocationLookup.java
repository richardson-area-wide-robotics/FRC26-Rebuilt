package frc.robot.pearce.subsystems.smart;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import lombok.AllArgsConstructor;

public class ScoringLocationLookup {
    private ScoringLocationFinalized scoringLocations[] = new ScoringLocationFinalized[999];


    public void buildScoringLocations(){
        ScoringLocation scoringLocationsRaw[] = new ScoringLocation[999];


        scoringLocationsRaw[0] = new ScoringLocation("hub", new Pose2d(13,4, new Rotation2d()), null);
        scoringLocationsRaw[1] = new ScoringLocation("left_corner", new Pose2d(16,0.5, new Rotation2d()), null);
        scoringLocationsRaw[2] = new ScoringLocation("climber", new Pose2d(14.5,4, new Rotation2d()), null);
        scoringLocationsRaw[3] = new ScoringLocation("right_corner", new Pose2d(16,7.5, new Rotation2d()), null);
        scoringLocationsRaw[4] = new ScoringLocation("rightside_hub", new Pose2d(13,5, new Rotation2d()), null);
        scoringLocationsRaw[5] = new ScoringLocation("leftside_hub", new Pose2d(13,3, new Rotation2d()), null);


        DriverStation.Alliance alliance = DriverStation.getAlliance().get();

        for (int i = 0; i < scoringLocationsRaw.length; i++) {
            if(alliance == DriverStation.Alliance.Red){
                ScoringLocation scoringLocation = scoringLocationsRaw[i];

                scoringLocations[i] = new ScoringLocationFinalized(scoringLocation.name, scoringLocation.redPose2d);
            }
            else if(alliance == DriverStation.Alliance.Blue){
                ScoringLocation scoringLocation = scoringLocationsRaw[i];

                scoringLocations[i] = new ScoringLocationFinalized(scoringLocation.name, scoringLocation.bluePose2d);
            }
        }

    }
    public Pose2d findClosest(Pose2d robotPose){
        double closestDist = Double.MAX_VALUE;
        int closestI = -1;
        for(int i = 0; i<scoringLocations.length;i++){
            ScoringLocationFinalized scoringLocationFinalized = scoringLocations[i];
            if(robotPose.getTranslation().getDistance(scoringLocationFinalized.finalizedPose2d.getTranslation()) > closestDist){
                closestDist = robotPose.getTranslation().getDistance(scoringLocationFinalized.finalizedPose2d.getTranslation());
                closestI = i;
            };
        }
        return scoringLocations[closestI].finalizedPose2d;

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