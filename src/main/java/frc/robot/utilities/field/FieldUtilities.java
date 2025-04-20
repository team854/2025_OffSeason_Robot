package frc.robot.utilities.field;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;

public final class FieldUtilities {
    /**
     * 
     * @param pose The pose to check which side of the field it is on
     * @return True if the pose is on the right side of the field, false if it is on the left side of the field
     */
    public static boolean getFieldSide(Pose2d pose) {
        return pose.getY() < (Constants.APRIL_TAG_FIELD_LAYOUT.getFieldWidth() / 2);
    }
}
