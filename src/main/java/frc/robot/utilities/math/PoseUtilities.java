package frc.robot.utilities.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class PoseUtilities {
    public static double[] convertPoseToNumbers(Pose3d pose) {
        return new double[] {
            pose.getX(),
            pose.getY(),
            pose.getZ(),
            pose.getRotation().getQuaternion().getW(),
            pose.getRotation().getQuaternion().getX(),
            pose.getRotation().getQuaternion().getY(),
            pose.getRotation().getQuaternion().getZ()
        };
    }

    public static double[] convertPoseToNumbers(Pose2d pose) {
        return new double[] {
            pose.getX(),
            pose.getY(),
            pose.getRotation().getRadians()
        };
    }
}
