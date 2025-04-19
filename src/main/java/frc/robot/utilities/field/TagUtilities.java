package frc.robot.utilities.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class TagUtilities {
    /**
     * Gets the pose of an april tag
     * 
     * @param tagID The id of the tag to get the pose from
     * @param facingTag If the rotation of the pose should face the tag
     * @return The pose of the tag
     */
    public static Pose3d getTagPose(int tagID, boolean facingTag) {
        Pose3d tagPose = Constants.APRIL_TAG_FIELD_LAYOUT.getTagPose(tagID).get();

        // By default the tag poses face away from the tag
        // If facingTag is true add a 3d rotation of 180 in the yaw axis
        if (facingTag) {
            tagPose = tagPose.transformBy(new Transform3d(0,0,0, new Rotation3d(0, 0, Units.degreesToRadians(180))));
        }
        return tagPose;
    }

    /**
     * Gets the pose of an april tag
     * 
     * @param tagID The id of the tag to get the pose from
     * @return The pose of the tag
     */
    public static Pose3d getTagPose(int tagID) {
        Pose3d tagPose = Constants.APRIL_TAG_FIELD_LAYOUT.getTagPose(tagID).get();
        return tagPose;
    }
}
