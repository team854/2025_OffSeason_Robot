package frc.robot.utilities.field;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public final class ReefUtilities {

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
            Pose2d tagPose = TagUtilities.getTagPose(tagID).toPose2d();

            // Compute the distance bettween the robot and the tag in 2d space
            double tagDistance = tagPose.getTranslation().getDistance(pose.getTranslation());
            if (tagDistance < Constants.ReefConstants.CLOSE_DISTANCE.in(Meter) && tagDistance < lowestTagDistance) {
                lowestTagDistance = tagDistance;
                lowestTagID = tagID;
            }
        }

        return lowestTagID;
    }

    /**
     * 
     * @param tagPose The pose of the tag when facing the tag
     * @param right If the branch is to the left or right of the tag
     */
    public static Pose2d getBranchPose(Pose2d tagPose, boolean right) {
        Transform2d branchTransform = new Transform2d(
                Constants.ReefConstants.FieldConstants.BRANCH_FOWARD_OFFSET.in(Meter),
                Constants.ReefConstants.FieldConstants.BRANCH_LEFT_OFFSET.in(Meter) * ((right) ? -1 : 1),
                Rotation2d.fromDegrees(0));
        return tagPose.transformBy(branchTransform);
    }

    public static Distance getBranchHeight(int branchLevel) {
        switch (branchLevel) {
            case 0:
                return Constants.ReefConstants.FieldConstants.L1.MAX_HEIGHT;
            case 1:
                return Constants.ReefConstants.FieldConstants.L2.MAX_HEIGHT;
            case 2:
                return Constants.ReefConstants.FieldConstants.L3.MAX_HEIGHT;
            case 3:
                return Constants.ReefConstants.FieldConstants.L4.MAX_HEIGHT;
            default:
                System.err.println("Unknown branch level " + branchLevel + ". Returning default of L3");
                return Constants.ReefConstants.FieldConstants.L3.MAX_HEIGHT;
        }
    }

    public static Angle getBranchAngle(int branchLevel) {
        switch (branchLevel) {
            case 0:
                return Constants.ReefConstants.FieldConstants.L1.BRANCH_ANGLE;
            case 1:
                return Constants.ReefConstants.FieldConstants.L2.BRANCH_ANGLE;
            case 2:
                return Constants.ReefConstants.FieldConstants.L3.BRANCH_ANGLE;
            case 3:
                return Constants.ReefConstants.FieldConstants.L4.BRANCH_ANGLE;
            default:
                System.err.println("Unknown branch level " + branchLevel + ". Returning default of L3");
                return Constants.ReefConstants.FieldConstants.L3.BRANCH_ANGLE;
        }
    }

    public static Angle getWristAngle(int branchLevel) {
        switch (branchLevel) {
            case 0:
                return Constants.ReefConstants.FieldConstants.L1.WRIST_ANGLE;
            case 1:
                return Constants.ReefConstants.FieldConstants.L2.WRIST_ANGLE;
            case 2:
                return Constants.ReefConstants.FieldConstants.L3.WRIST_ANGLE;
            case 3:
                return Constants.ReefConstants.FieldConstants.L4.WRIST_ANGLE;
            default:
                System.err.println("Unknown branch level " + branchLevel + ". Returning default of L3");
                return Constants.ReefConstants.FieldConstants.L3.WRIST_ANGLE;
        }
    }
}
