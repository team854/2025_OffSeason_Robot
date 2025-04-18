package frc.robot.utilities.field;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ReefUtilities {

    /**
     * 
     * @return An array with the reef tags for the robots alliance
     */
    public static int[] getTagIDs() {
        return RobotContainer.isBlueAlliance() 
                ? Constants.ReefConstants.FieldConstants.BLUE_ALLIANCE_REEF_TAG_IDS
                : Constants.ReefConstants.FieldConstants.RED_ALLIANCE_REEF_TAG_IDS;
    }

    /**
     * Returns the closest tag to a given pose. If there is not any close tag it returns -1
     * 
     * @param pose
     * @return
     */
    public static int getClosestReef(Pose2d pose) {
        int[] tagPoses = getTagIDs();

        // Set the lowest distance to the max possible value and the lowest tag id to -1
        double lowestTagDistance = Double.MAX_VALUE;
        int lowestTagID = -1;

        for (int tagID : tagPoses) {
            Pose2d tagPose = TagUtilities.getReefTagPose(tagID).toPose2d();

            // Compute the distance bettween the robot and the tag is 2d space
            double tagDistance = tagPose.getTranslation().getDistance(pose.getTranslation());
            if (tagDistance < Constants.ReefConstants.CLOSE_DISTANCE.in(Meter) && tagDistance < lowestTagDistance) {
                lowestTagDistance = tagDistance;
                lowestTagID = tagID;
            }
        }

        return lowestTagID;
    }
}
