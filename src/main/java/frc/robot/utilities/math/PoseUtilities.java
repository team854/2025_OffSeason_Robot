package frc.robot.utilities.math;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Distance;

public final class PoseUtilities {
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

    public static Distance calculatePoseDistance(Pose3d pose1, Pose3d pose2) {
        return Meter.of(pose1.getTranslation().getDistance(pose2.getTranslation()));
    }
}
