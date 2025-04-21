package frc.robot.utilities.field;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public final class CoralStationUtilities {
    /**
     * 
     * @return An array with the coral station tags for the robots alliance
     */
    public static int[] getTagIDs() {
        return RobotContainer.isBlueAlliance() 
                ? Constants.CoralStationConstants.FieldConstants.BLUE_ALLIANCE_CORAL_STATION_TAG_IDS
                : Constants.CoralStationConstants.FieldConstants.RED_ALLIANCE_CORAL_STATION_TAG_IDS;
    }
    
    /**
     * Returns the closest tag to a given pose. If there is not any close tag it returns -1
     * 
     * @param pose
     * @return
     */
    public static int getClosestCoralStation(Pose2d pose) {
        int[] tagPoses = getTagIDs();

        // Set the lowest distance to the max possible value and the lowest tag id to -1
        double lowestTagDistance = Double.MAX_VALUE;
        int lowestTagID = -1;

        for (int tagID : tagPoses) {
            Pose2d tagPose = TagUtilities.getTagPose(tagID).toPose2d();

            // Compute the distance bettween the robot and the tag in 2d space
            double tagDistance = tagPose.getTranslation().getDistance(pose.getTranslation());
            if (tagDistance < lowestTagDistance) {
                lowestTagDistance = tagDistance;
                lowestTagID = tagID;
            }
        }
        
        return lowestTagID;
    }


    /**
     * 
     * @param tagPose The pose of the tag when facing the tag
     * @param right True if the tag is on the right side of the field, false if it is on the left side of the field
     */
    public static Pose2d getPickupPose(Pose2d tagPose, boolean right) {
        Transform2d pickupTransform = new Transform2d(
                Constants.CoralStationConstants.FOWARD_OFFSET.in(Meter),
                Constants.CoralStationConstants.RIGHT_OFFSET.in(Meter) * ((right) ? -1 : 1),
                Rotation2d.fromDegrees(0));
        return tagPose.transformBy(pickupTransform);
    }
}
