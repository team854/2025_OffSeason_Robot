package frc.robot.utilities.saftey;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radian;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotKinematicConstants;
import frc.robot.utilities.math.PoseUtilities;

public final class ArmSafteyUtilities {
    /**
     * Returns the minium height that the end effector is allowed to be at when its at a speecific distance from the center of the robot
     * 
     * @param X The horizontal distance from the center of the robot
     * @return The minimum distance from the ground that the end effector is allowed to be at
     */
    public static Distance getMinimumEndEffecotrHeight(Distance X) {
        double meterX = X.in(Meter);

        // Calculate the bumper width and add an offset to it
        double halfBumperWidth = (Constants.RobotKinematicConstants.LENGTH.in(Meter) / 2) + Constants.ArmConstants.OUT_BUMPER_OFFSET.in(Meter);

        if (meterX > halfBumperWidth) {
            double angleHeight = (meterX - halfBumperWidth) * Math.tan(Constants.ArmConstants.OUT_BUMPER_ANGLE.in(Radian));
            return Meter.of(Constants.ArmConstants.MINIMUM_HEIGHT_IN_BUMPER.in(Meter) + angleHeight);
        }
        
        return Constants.ArmConstants.MINIMUM_HEIGHT_IN_BUMPER;
    }

    public static Double[] generateDebugLine(Pose3d robotPose) {
        List<Pose3d> outputPoses = new ArrayList<>();

        for (double offset = 0; offset < 1; offset+=0.025) {
            outputPoses.add(robotPose.plus(new Transform3d(offset, 0.0, getMinimumEndEffecotrHeight(Meter.of(offset)).in(Meter), new Rotation3d())));
        }

        return PoseUtilities.convertPoseArrayToNumbers(outputPoses.toArray(new Pose3d[0]));
    }

    public static Double[] generateDebugLine() {
        Pose3d robotPose = new Pose3d(RobotContainer.swerveSubsystem.getPose());
        return generateDebugLine(robotPose);
    }
}
