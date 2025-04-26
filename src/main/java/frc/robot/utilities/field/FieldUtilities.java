package frc.robot.utilities.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public final class FieldUtilities {
    /**
     * 
     * @param pose The pose to check which side of the field it is on
     * @return True if the pose is on the right side of the field, false if it is on the left side of the field
     */
    public static boolean getFieldSide(Pose2d pose) {
        return pose.getY() < (Constants.APRIL_TAG_FIELD_LAYOUT.getFieldWidth() / 2);
    }

    public static Pose2d mirrorPoseIfRed(Pose2d pose) {
        if (!RobotContainer.isBlueAlliance()) {
            return mirrorPose(pose);
        }
        return pose;
    }

    private static Pose2d mirrorPose(Pose2d pose) {
        return pose.rotateAround(new Translation2d(Constants.APRIL_TAG_FIELD_LAYOUT.getFieldLength() / 2, pose.getY()), Rotation2d.fromDegrees(180));
    }
}
